/* A minimal CPU hog: `cpu_hog <workers> <seconds>` forks <workers> busy-looping children.
 *
 * Stands in for `stress-ng --cpu $(nproc)`, which is not installed on this box (and the
 * benchmark is not allowed to install system packages). Each worker runs a floating-point
 * busy loop at the default scheduling policy, so a SCHED_FIFO control loop should still
 * preempt it; the point is to saturate every core and to make the simulator container
 * compete for CPU. Children exit on their own after <seconds> and are also killed when the
 * parent is signalled.
 */

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stop_now = 0;

static void on_signal(int sig) {
  (void)sig;
  stop_now = 1;
}

static double now_seconds(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(int argc, char** argv) {
  int workers = (argc > 1) ? atoi(argv[1]) : 1;
  double seconds = (argc > 2) ? atof(argv[2]) : 60.0;
  if (workers < 1) {
    workers = 1;
  }

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  pid_t* pids = calloc((size_t)workers, sizeof(pid_t));
  if (pids == NULL) {
    return 1;
  }

  for (int i = 0; i < workers; ++i) {
    pid_t pid = fork();
    if (pid == 0) {
      /* Die with the parent even if it is killed hard. */
      prctl(PR_SET_PDEATHSIG, SIGKILL);
      signal(SIGINT, on_signal);
      signal(SIGTERM, on_signal);
      double x = 1.000001 + (double)i;
      double deadline = now_seconds() + seconds;
      while (!stop_now && now_seconds() < deadline) {
        for (int k = 0; k < 2000000; ++k) {
          x = x * 1.0000001 + 0.000001;
          x = sqrt(x * x + 1.0);
        }
      }
      /* Keep the optimiser honest. */
      if (x == 42.0) {
        printf("%f\n", x);
      }
      _exit(0);
    }
    if (pid < 0) {
      perror("fork");
      break;
    }
    pids[i] = pid;
  }

  int status = 0;
  for (int i = 0; i < workers; ++i) {
    if (pids[i] > 0) {
      waitpid(pids[i], &status, 0);
    }
  }
  free(pids);
  return 0;
}
