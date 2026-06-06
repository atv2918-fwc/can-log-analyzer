"""Unit tests for CAN Log Analyzer."""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from can_log_analyzer import (
    ASCParser, CANLogAnalyzer, CANFrame, SignalDef,
    generate_sample_log, SIGNAL_DB, FRAME_NAMES
)


@pytest.fixture
def sample_frames():
    log = generate_sample_log(1.0)
    return ASCParser().parse_text(log)


def test_parser_produces_frames(sample_frames):
    assert len(sample_frames) > 0


def test_parser_frame_ids(sample_frames):
    ids = {f.can_id for f in sample_frames}
    assert 0x180 in ids
    assert 0x300 in ids
    assert 0x400 in ids


def test_parser_frame_data_length(sample_frames):
    for f in sample_frames:
        assert len(f.data) == f.dlc


def test_parser_timestamps_ascending(sample_frames):
    ts = [f.timestamp for f in sample_frames]
    assert ts == sorted(ts)


def test_analyzer_report_generated(sample_frames):
    analyzer = CANLogAnalyzer(sample_frames)
    report = analyzer.report()
    assert "CAN Log Analysis Report" in report
    assert "EPS_Status" in report


def test_analyzer_bus_load_reasonable(sample_frames):
    analyzer = CANLogAnalyzer(sample_frames)
    analyzer.analyze()
    # bus load should be between 0 and 100%
    report = analyzer.report()
    load_line = [l for l in report.splitlines() if "Bus load" in l][0]
    load_val = float(load_line.split(":")[1].strip().split("%")[0])
    assert 0.0 < load_val < 100.0


def test_signal_decode_steering_torque():
    # 0x0064 = 100 raw → 100 * 0.1 - 3276.8 = -3266.8... wait that's not right
    # SteeringTorque: factor=0.1, offset=-3276.8, 16 bits signed
    # Let's use 32768 raw (0x8000) → should be close to 0 Nm
    sig = SIGNAL_DB[0x180]["SteeringTorque"]
    # raw bytes for value 0 Nm: (0 - (-3276.8)) / 0.1 = 32768 = 0x8000
    data = b'\x80\x00\x00\x00\x00\x00\x00\x00'
    val = sig.decode(data)
    assert abs(val) < 1.0   # near zero


def test_signal_range_check():
    sig = SIGNAL_DB[0x180]["EPSMotorCurrent"]
    assert sig.in_range(50.0)
    assert not sig.in_range(-1.0)
    assert not sig.in_range(200.0)


def test_signal_trace_returns_data(sample_frames):
    analyzer = CANLogAnalyzer(sample_frames)
    trace = analyzer.get_signal_trace(0x300, "EngineSpeed")
    assert len(trace) > 0
    for ts, val in trace:
        assert isinstance(ts, float)
        assert isinstance(val, float)


def test_signal_trace_unknown_signal(sample_frames):
    analyzer = CANLogAnalyzer(sample_frames)
    trace = analyzer.get_signal_trace(0x180, "NonExistentSignal")
    assert trace == []


def test_empty_log_no_crash():
    analyzer = CANLogAnalyzer([])
    report = analyzer.report()
    assert "No frames" in report or "WARNING" in report


def test_missing_expected_id_flagged():
    # Only include ECM frames, expect EPS and BCM missing
    log = generate_sample_log(0.5)
    frames = [f for f in ASCParser().parse_text(log) if f.can_id == 0x300]
    analyzer = CANLogAnalyzer(frames)
    analyzer.analyze()
    issues = analyzer._issues
    missing_ids = [i for i in issues if "MISSING" in i]
    assert len(missing_ids) >= 2
