"""Offline arrival-timing analysis of Franka Emika Robot (FER, FCI v5) robot-state datagrams.

See ``analyze.py`` at the top of ``bench/fer-capture`` for the CLI entry point and
``fer_capture.fci`` for the wire-format background. This package is split by
responsibility:

  pcap.py        - reading/parsing the pcap/pcapng capture file formats and stripping
                    the link layer down to an IPv4 packet
  reassembly.py   - IPv4 fragment reassembly into complete UDP datagrams
  fci.py          - FCI v5 protocol decoding (RobotState/RobotCommand message_id,
                     flow detection, capture-clock fitting, gap/drift analysis)
  stats.py        - generic statistics helpers (percentiles, medians, unit conversion)
  report.py       - text report rendering
  selftest.py     - synthetic capture generation and the ``--self-test`` checks
"""
