//! The temporary file a downloaded model library is written to before `dlopen`.

use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::error::{FrankaError, FrankaResult};

/// A temporary file holding a downloaded model library, removed when dropped.
///
/// Port of `franka::LibraryDownloader`, which holds a `Poco::TemporaryFile` and
/// unlinks it in its destructor.
#[derive(Debug)]
pub(super) struct TempLibraryFile {
    pub(super) path: PathBuf,
}

impl Drop for TempLibraryFile {
    fn drop(&mut self) {
        // libfranka swallows every error here too (`LibraryDownloader::~LibraryDownloader`
        // is a `try { ... } catch (...) {}`): a leftover file in the temp directory is not
        // worth failing a drop over.
        let _ = std::fs::remove_file(&self.path);
    }
}

/// Writes `bytes` to a fresh `0600` file under [`std::env::temp_dir`].
///
/// The name is unique per process, per nanosecond and per call, so two models
/// downloaded concurrently (or two processes on the same machine) never collide
/// the way a fixed name would. `create_new` makes the create-or-fail atomic, so
/// an attacker cannot pre-create the path and have the library written through
/// a symlink.
pub(super) fn write_temp_library(bytes: &[u8]) -> FrankaResult<TempLibraryFile> {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let path = std::env::temp_dir().join(format!(
        "libfcimodels-{}-{nanos}-{}.so",
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    ));

    let mut options = std::fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }

    let file = options.open(&path).map_err(|e| {
        FrankaError::Model(format!(
            "libfranka: Cannot save model library: {}: {e}",
            path.display()
        ))
    })?;
    // From here on the file exists, so bind the guard before anything can fail.
    let guard = TempLibraryFile { path };
    let mut file = file;
    file.write_all(bytes)
        .and_then(|()| file.sync_all())
        .map_err(|e| {
            FrankaError::Model(format!(
                "libfranka: Cannot save model library: {}: {e}",
                guard.path.display()
            ))
        })?;
    drop(file);
    Ok(guard)
}
