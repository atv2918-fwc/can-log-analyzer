# CAN Log Analyzer ⚡

A Python tool for **rapid triage of automotive CAN bus logs** (.asc format). Parses raw bus captures, decodes signals via an inline DBC-style database, flags anomalies, and generates a plain-text report — all in seconds.

> **Background:** This is the open-source version of an automation tool I built at Mercedes-Benz R&D India. The original reduced log analysis time from 30–40 minutes to ~15 minutes across production plants in Germany and China, earning the **Spontan Award**.

## Features

- `.asc` log file parsing (Vector CANalyzer format)
- Signal decoding via inline DBC-style signal definitions (factor, offset, range, signed/unsigned)
- Per-message statistics: frame count, average cycle time, jitter
- Bus load estimation (500 kbps baseline)
- Automatic issue detection:
  - Missing expected message IDs
  - Cycle time deviations (>20% from expected)
  - Signal out-of-range violations
- Signal trace extraction for any defined signal
- Synthetic log generator for testing and demo

## Project Structure

```
can-log-analyzer/
├── can_log_analyzer.py     # Core parser + analyzer
├── tests/
│   └── test_analyzer.py    # Unit tests
├── sample_logs/
│   └── sample.asc          # Example log file
└── README.md
```

## Quick Start

```bash
python can_log_analyzer.py
```

Or point it at a real `.asc` file:

```python
from can_log_analyzer import ASCParser, CANLogAnalyzer

parser = ASCParser()
frames = parser.parse_file("my_capture.asc")
analyzer = CANLogAnalyzer(frames)
print(analyzer.report())
```

## Sample Output

```
=================================================================
  CAN Log Analysis Report
=================================================================
  Log duration  : 2.000 s
  Total frames  : 318
  Unique IDs    : 3
  Bus load est. : 4.2%  (@ 500 kbps)

── Per-Message Statistics ─────────────────────────────────
  0x180  EPS_Status       cnt=  200  cycle=10.01 ms   jitter=±0.12 ms
  0x300  ECM_Status       cnt=  100  cycle=20.00 ms   jitter=±0.08 ms
  0x400  BCM_Warning      cnt=   20  cycle=100.2 ms   jitter=±0.45 ms  ⚠ 2 OOR

── Issues Found ───────────────────────────────────────────
  ⚠  SIGNAL OOR: SoundWarningReq = 3.0  at t=0.847s (range 0.0–0.0)
=================================================================
```

## Extending the Signal Database

Add entries to `SIGNAL_DB` in `can_log_analyzer.py`:

```python
SIGNAL_DB[0x250] = {
    "BatteryVoltage": SignalDef("BatteryVoltage", 0, 16, 0.001, 0.0, "V", 9.0, 16.0),
}
```

## Standards Referenced

- Vector CANalyzer .asc log format
- ISO 11898-1 (CAN data link layer)
- CAN DBC signal definition conventions
