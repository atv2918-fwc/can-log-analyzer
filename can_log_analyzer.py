"""
CAN Log Analyzer
================
Parses, filters, decodes, and reports on automotive CAN bus log files
(.asc / .blf-like text format). Designed for high-speed triage of
production-plant diagnostic logs.

Inspired by the automation tool built at Mercedes-Benz R&D India that
reduced log analysis time from 30–40 minutes to ~15 minutes.

Author: Anurag Thaliyil Veedu
"""

import re
import time
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
from collections import defaultdict
from enum import IntEnum


# ─── Signal Definitions (DBC-like inline DB) ─────────────────────────────────

@dataclass
class SignalDef:
    name: str
    start_bit: int
    length: int         # bits
    factor: float
    offset: float
    unit: str
    min_val: float
    max_val: float
    is_signed: bool = False

    def decode(self, raw_bytes: bytes) -> float:
        """Extract signal value from CAN data bytes."""
        # Build integer from bytes
        raw_int = int.from_bytes(raw_bytes, "big")
        # Extract bits (simplified: byte-aligned signals only for clarity)
        byte_idx = self.start_bit // 8
        bit_shift = 8 - (self.start_bit % 8) - self.length
        mask = (1 << self.length) - 1
        extracted = (int.from_bytes(raw_bytes[byte_idx:byte_idx + (self.length // 8 + 1)], "big") >> max(bit_shift, 0)) & mask
        if self.is_signed and (extracted >> (self.length - 1)):
            extracted -= (1 << self.length)
        return round(extracted * self.factor + self.offset, 3)

    def in_range(self, value: float) -> bool:
        return self.min_val <= value <= self.max_val


# Minimal inline signal database (mimics a real DBC file subset)
SIGNAL_DB: dict[int, dict[str, SignalDef]] = {
    0x180: {  # EPS Status Frame
        "SteeringTorque":    SignalDef("SteeringTorque",    0, 16, 0.1, -3276.8, "Nm",  -50.0,  50.0, True),
        "SteeringAngle":     SignalDef("SteeringAngle",    16, 16, 0.1, -3276.8, "deg", -780.0, 780.0, True),
        "EPSMotorCurrent":   SignalDef("EPSMotorCurrent",  32,  8, 0.5,     0.0, "A",     0.0, 100.0),
        "EPSSystemStatus":   SignalDef("EPSSystemStatus",  40,  4, 1.0,     0.0, "",      0.0,  15.0),
    },
    0x300: {  # ECM Status
        "EngineSpeed":       SignalDef("EngineSpeed",       0, 16, 0.25,    0.0, "rpm",   0.0, 8000.0),
        "ThrottlePosition":  SignalDef("ThrottlePosition", 16,  8, 0.4,     0.0, "%",     0.0,  100.0),
        "CoolantTemp":       SignalDef("CoolantTemp",      24,  8, 1.0,   -40.0, "°C",  -40.0,  130.0),
    },
    0x400: {  # BCM / Sound Warning
        "TrunkStatus":       SignalDef("TrunkStatus",       0,  2, 1.0,     0.0, "",      0.0,    3.0),
        "SoundWarningReq":   SignalDef("SoundWarningReq",   2,  4, 1.0,     0.0, "",      0.0,   15.0),
        "InteriorLighting":  SignalDef("InteriorLighting",  6,  8, 0.4,     0.0, "%",     0.0,  100.0),
    },
}

FRAME_NAMES = {
    0x180: "EPS_Status",
    0x300: "ECM_Status",
    0x400: "BCM_Warning",
    0x500: "Gateway_Diag",
    0x700: "Network_Mgmt",
}


# ─── Log Entry ────────────────────────────────────────────────────────────────

@dataclass
class CANFrame:
    timestamp: float
    can_id: int
    dlc: int
    data: bytes
    channel: int = 1
    line_no: int = 0

    @property
    def frame_name(self) -> str:
        return FRAME_NAMES.get(self.can_id, f"CAN_{self.can_id:03X}")

    def decode_signals(self) -> dict[str, float]:
        sigs = SIGNAL_DB.get(self.can_id, {})
        result = {}
        for name, sig in sigs.items():
            try:
                result[name] = sig.decode(self.data)
            except Exception:
                result[name] = None
        return result


# ─── ASC Parser ──────────────────────────────────────────────────────────────

class ASCParser:
    """
    Parses Vector CANalyzer .asc log files.
    Format: <timestamp>  <channel>  <CAN-ID>  Rx/Tx  d  <DLC>  <bytes...>
    """

    LINE_RE = re.compile(
        r"^\s*([\d.]+)\s+(\d+)\s+([0-9A-Fa-f]+)\s+[RTx]+\s+d\s+(\d)\s+((?:[0-9A-Fa-f]{2}\s*)+)"
    )

    def parse_file(self, path: str) -> list[CANFrame]:
        frames = []
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"Log file not found: {path}")

        with open(p, "r", errors="replace") as f:
            for line_no, line in enumerate(f, 1):
                frame = self._parse_line(line, line_no)
                if frame:
                    frames.append(frame)
        return frames

    def parse_text(self, text: str) -> list[CANFrame]:
        frames = []
        for line_no, line in enumerate(text.strip().splitlines(), 1):
            frame = self._parse_line(line, line_no)
            if frame:
                frames.append(frame)
        return frames

    def _parse_line(self, line: str, line_no: int) -> Optional[CANFrame]:
        m = self.LINE_RE.match(line)
        if not m:
            return None
        ts, ch, can_id_str, dlc_str, data_str = m.groups()
        data_bytes = bytes(int(b, 16) for b in data_str.split())
        return CANFrame(
            timestamp=float(ts),
            can_id=int(can_id_str, 16),
            dlc=int(dlc_str),
            data=data_bytes[:int(dlc_str)],
            channel=int(ch),
            line_no=line_no,
        )


# ─── Analyzer ────────────────────────────────────────────────────────────────

@dataclass
class FrameStats:
    can_id: int
    count: int
    intervals: list[float] = field(default_factory=list)
    out_of_range_signals: list[tuple] = field(default_factory=list)

    @property
    def avg_cycle_ms(self) -> Optional[float]:
        if len(self.intervals) < 2:
            return None
        return round(statistics.mean(self.intervals) * 1000, 2)

    @property
    def jitter_ms(self) -> Optional[float]:
        if len(self.intervals) < 3:
            return None
        return round(statistics.stdev(self.intervals) * 1000, 2)


class CANLogAnalyzer:
    """
    Analyzes parsed CAN frames for:
    - Bus load estimation
    - Cycle time and jitter per message ID
    - Signal range violations
    - Missing expected messages
    - Suspicious patterns (burst errors, gap detection)
    """

    EXPECTED_IDS = {0x180, 0x300, 0x400}
    EXPECTED_CYCLE_MS = {
        0x180: 10.0,   # EPS: 10 ms
        0x300: 20.0,   # ECM: 20 ms
        0x400: 100.0,  # BCM: 100 ms
    }

    def __init__(self, frames: list[CANFrame]):
        self.frames = frames
        self._stats: dict[int, FrameStats] = {}
        self._issues: list[str] = []
        self._analyzed = False

    def analyze(self):
        if not self.frames:
            self._issues.append("WARNING: No frames found in log.")
            return

        # Group frames by ID
        by_id: dict[int, list[CANFrame]] = defaultdict(list)
        for f in self.frames:
            by_id[f.can_id].append(f)

        # Compute per-ID stats
        for can_id, flist in by_id.items():
            flist.sort(key=lambda x: x.timestamp)
            intervals = [
                flist[i].timestamp - flist[i-1].timestamp
                for i in range(1, len(flist))
            ]
            stats = FrameStats(can_id=can_id, count=len(flist), intervals=intervals)

            # Signal range check
            if can_id in SIGNAL_DB:
                for frame in flist:
                    decoded = frame.decode_signals()
                    for sig_name, value in decoded.items():
                        if value is None:
                            continue
                        sig_def = SIGNAL_DB[can_id][sig_name]
                        if not sig_def.in_range(value):
                            stats.out_of_range_signals.append(
                                (frame.timestamp, sig_name, value)
                            )
            self._stats[can_id] = stats

        # Check for missing expected IDs
        for eid in self.EXPECTED_IDS:
            if eid not in by_id:
                self._issues.append(
                    f"MISSING: 0x{eid:03X} ({FRAME_NAMES.get(eid,'?')}) — "
                    f"expected every {self.EXPECTED_CYCLE_MS.get(eid,0):.0f} ms but never seen."
                )

        # Cycle time deviation check
        for can_id, expected_ms in self.EXPECTED_CYCLE_MS.items():
            stats = self._stats.get(can_id)
            if stats and stats.avg_cycle_ms:
                deviation = abs(stats.avg_cycle_ms - expected_ms)
                if deviation > expected_ms * 0.20:  # >20% deviation
                    self._issues.append(
                        f"TIMING ANOMALY: 0x{can_id:03X} ({FRAME_NAMES.get(can_id,'?')}) "
                        f"— expected {expected_ms} ms, measured {stats.avg_cycle_ms} ms "
                        f"(Δ {deviation:.1f} ms)"
                    )

        # Signal OOR issues
        for can_id, stats in self._stats.items():
            for ts, sig, val in stats.out_of_range_signals[:3]:   # cap at 3 per ID
                sig_def = SIGNAL_DB[can_id][sig]
                self._issues.append(
                    f"SIGNAL OOR: {sig} = {val} {sig_def.unit} "
                    f"at t={ts:.3f}s (range {sig_def.min_val}–{sig_def.max_val})"
                )

        self._analyzed = True

    def report(self) -> str:
        if not self._analyzed:
            self.analyze()

        duration = (
            self.frames[-1].timestamp - self.frames[0].timestamp
            if len(self.frames) > 1 else 0.0
        )
        total_bits = sum(
            (f.dlc * 8 + 47) for f in self.frames  # approx bits per CAN frame
        )
        bus_load = (total_bits / (duration * 500_000) * 100) if duration > 0 else 0.0

        lines = [
            "=" * 65,
            "  CAN Log Analysis Report",
            "=" * 65,
            f"  Log duration  : {duration:.3f} s",
            f"  Total frames  : {len(self.frames)}",
            f"  Unique IDs    : {len(self._stats)}",
            f"  Bus load est. : {bus_load:.1f}%  (@ 500 kbps)",
            "",
            "── Per-Message Statistics ─────────────────────────────────",
        ]

        for can_id in sorted(self._stats):
            s = self._stats[can_id]
            name = FRAME_NAMES.get(can_id, f"0x{can_id:03X}")
            cycle = f"{s.avg_cycle_ms} ms" if s.avg_cycle_ms else "N/A"
            jitter = f"±{s.jitter_ms} ms" if s.jitter_ms else "N/A"
            oor = f"  ⚠ {len(s.out_of_range_signals)} OOR" if s.out_of_range_signals else ""
            lines.append(
                f"  0x{can_id:03X}  {name:<16}  cnt={s.count:>5}  "
                f"cycle={cycle:<10}  jitter={jitter}{oor}"
            )

        if self._issues:
            lines += ["", "── Issues Found ───────────────────────────────────────────"]
            for issue in self._issues:
                lines.append(f"  ⚠  {issue}")
        else:
            lines.append("\n  ✔  No issues detected.")

        lines += ["", "=" * 65]
        return "\n".join(lines)

    def get_signal_trace(self, can_id: int, signal_name: str) -> list[tuple[float, float]]:
        """Return (timestamp, value) pairs for a given signal."""
        sig_def = SIGNAL_DB.get(can_id, {}).get(signal_name)
        if not sig_def:
            return []
        return [
            (f.timestamp, sig_def.decode(f.data))
            for f in self.frames if f.can_id == can_id
        ]


# ─── Sample Log Generator (for demo / testing) ───────────────────────────────

def generate_sample_log(duration_s: float = 2.0) -> str:
    """Generate a synthetic .asc-style log for testing."""
    import random
    lines = ["date Fri Dec 01 10:00:00 2023", "base hex  timestamps absolute", ""]
    t = 0.0
    t180, t300, t400 = 0.0, 0.0, 0.0

    while t < duration_s:
        step = 0.005
        t = round(t + step, 4)

        if t >= t180:
            torque = random.randint(-200, 200)
            angle  = random.randint(-3000, 3000)
            current = random.randint(0, 180)
            data = (
                torque.to_bytes(2, "big", signed=True) +
                angle.to_bytes(2, "big", signed=True) +
                bytes([current, 0x01, 0x00, 0x00])
            )
            lines.append(f"   {t:.4f} 1 180 Rx d 8 " + " ".join(f"{b:02X}" for b in data))
            t180 = round(t + 0.010, 4)

        if t >= t300:
            rpm = random.randint(600, 3000)
            tps = random.randint(0, 200)   # 0–200 → /0.4 = 0–500% capped below 256
            temp = random.randint(50, 110) + 40   # 90–150 → fits in byte
            tps_raw = min(255, int(tps / 0.4))
            temp_raw = min(255, temp)
            data = (
                int(rpm / 0.25).to_bytes(2, "big") +
                bytes([tps_raw, temp_raw, 0x00, 0x00, 0x00, 0x00])
            )
            lines.append(f"   {t:.4f} 1 300 Rx d 8 " + " ".join(f"{b:02X}" for b in data))
            t300 = round(t + 0.020, 4)

        if t >= t400:
            # Occasionally inject an anomaly
            trunk = 0x01 if random.random() < 0.05 else 0x00
            warn  = 0x03 if random.random() < 0.03 else 0x00
            light = random.randint(0, 250)
            status_byte = (trunk & 0x03) | ((warn & 0x0F) << 2)
            data = bytes([status_byte, light, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            lines.append(f"   {t:.4f} 1 400 Rx d 8 " + " ".join(f"{b:02X}" for b in data))
            t400 = round(t + 0.100, 4)

    return "\n".join(lines)


# ─── Entry Point ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Generating sample CAN log (2 seconds of bus traffic)...\n")
    log_text = generate_sample_log(2.0)

    parser = ASCParser()
    frames = parser.parse_text(log_text)

    analyzer = CANLogAnalyzer(frames)
    print(analyzer.report())

    # Show signal trace snippet
    trace = analyzer.get_signal_trace(0x180, "SteeringTorque")[:5]
    print("\nSteering Torque trace (first 5 samples):")
    for ts, val in trace:
        print(f"  t={ts:.4f}s  →  {val} Nm")
