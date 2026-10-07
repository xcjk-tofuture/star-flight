import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from star_protocol import Frame, Parser
from serial_cli import main, prepare_operation
from telemetry import (PLEIADES, TM4C, FLIGHT, parameter_payload, decode_status,
                       decode_diagnostics, decode_u32)
from replay_log import replay, FlightLog

class ScriptPort:
    def __init__(self, device, capabilities):
        self.device, self.capabilities = device, capabilities
        self.requests = []; self.data = bytearray()
    def __enter__(self): return self
    def __exit__(self, *args): pass
    @property
    def in_waiting(self): return len(self.data)
    def write(self, data):
        frame, = Parser().feed(data, 0); self.requests.append(frame)
        replies = {2: struct.pack('<I', self.device), 3: struct.pack('<I', self.capabilities),
                   5: struct.pack('<H', 20), 6: b''}
        self.data.extend(Frame(1, frame.sequence, frame.command, b'\x00'+replies[frame.command]).encode())
        return len(data)
    def read(self, n):
        out = bytes(self.data[:min(n, 3)]); del self.data[:len(out)]; return out

class BenchToolsTests(unittest.TestCase):
    def test_capability_gate_prevents_operation_write(self):
        port = ScriptPort(TM4C, 3)
        with patch.dict(sys.modules, serial=types.SimpleNamespace(Serial=lambda *a, **k: port)), \
             patch.object(sys, 'argv', ['serial_cli.py', '--port', 'fake', 'save']):
            with self.assertRaises(ValueError): main()
        self.assertEqual([q.command for q in port.requests], [2, 3])

    def test_uav_period_write_and_decoded_read(self):
        for args in (['write', '1', '20'], ['read', '1']):
            port = ScriptPort(FLIGHT, 19); out = io.StringIO()
            with patch.dict(sys.modules, serial=types.SimpleNamespace(Serial=lambda *a, **k: port)), \
                 patch.object(sys, 'argv', ['serial_cli.py', '--port', 'fake', *args]), \
                 contextlib.redirect_stdout(out): main()
            self.assertEqual([q.sequence for q in port.requests], [1, 2, 3])
            if args[0] == 'write': self.assertEqual(port.requests[-1].payload, b'\x01\x00\x14\x00')
            else: self.assertEqual(json.loads(out.getvalue())['data']['value'], 20)

    def test_parameter_ranges_and_device_isolation(self):
        for device, period in [(PLEIADES, 49), (TM4C, 49), (FLIGHT, 19), (FLIGHT, 20.5)]:
            with self.assertRaises(ValueError): parameter_payload(device, 31, 1, period)
        for parameter, value in [(0x100, 1e100), (0x104, 5), (0x105, 50.5), (0x106, float('nan'))]:
            with self.assertRaises(ValueError): parameter_payload(PLEIADES, 31, parameter, value)
        for device in (FLIGHT, TM4C):
            with self.assertRaises(ValueError): parameter_payload(device, 31, 0x100, 1)
        for op in ['status', 'save', 'diagnostics']:
            with self.assertRaises(ValueError): prepare_operation(PLEIADES, 0, op)
        with self.assertRaises(ValueError): decode_u32(b'\x00')

    def test_typed_status_and_diagnostics(self):
        status = struct.pack('<BfffBf6h', 4, .1, 0, -.2, 2, 12, *([0]*6))
        self.assertEqual(decode_status(TM4C, status)['state_name'], 'uncalibrated')
        with self.assertRaises(ValueError): decode_status(PLEIADES, status)
        with self.assertRaises(ValueError): decode_status(FLIGHT, struct.pack('<Bfff', 0, float('nan'), 0, 0))
        diag = struct.pack('<B5IB', 3, 4, 5, 6, 7, 8, 1)
        self.assertEqual(decode_diagnostics(FLIGHT, diag)['sequences']['pid'], 8)
        with self.assertRaises(ValueError): decode_diagnostics(FLIGHT, diag[:-1]+b'\x02')

class BenchReplayTests(unittest.TestCase):
    def run_log(self, rows):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'capture.jsonl'
            path.write_text('\n'.join(json.dumps(row) for row in rows), encoding='utf-8')
            return replay(path)
    def row(self, sequence=0, time=0):
        return dict(time_s=time, flags=2, sequence=sequence, command=4,
                    payload_hex=(b'\x00'+struct.pack('<Bfff', 0, 0, 0, 0)).hex())
    def test_wrap_gap_and_errors(self):
        rows = [dict(format='star-log-v1', device=hex(FLIGHT))]
        rows += [self.row(65535, 0), self.row(0, .1), self.row(3, .5), self.row(3, .6)]
        rows.append(dict(time_s=.7, flags=1, sequence=9, command=6, payload_hex='04'))
        result = self.run_log(rows)
        self.assertEqual(result['estimated_missing_status_events'], 2)
        self.assertEqual(result['event_sequence_discontinuities'], 2)
        self.assertAlmostEqual(result['max_status_gap_s'], .4)
        self.assertEqual(result['error_frames'], 1)

    def test_telemetry_interleaved_with_utf8_log_chunks(self):
        message = '[0000001234 ms][INFO] 启动完成\r\n'
        payload = message.encode('utf-8')
        split = payload.index('启'.encode('utf-8')) + 1
        rows = [dict(format='star-log-v1', device=hex(FLIGHT)), self.row(0, 0)]
        rows += [dict(time_s=.1, flags=2, sequence=0, command=0x20f0, payload_hex=payload[:split].hex()),
                 self.row(1, .2),
                 dict(time_s=.3, flags=2, sequence=1, command=0x20f0, payload_hex=payload[split:].hex())]
        result = self.run_log(rows)
        self.assertEqual(result['last_log'], message.rstrip())
        self.assertEqual((result['status_frames'], result['log_frames'], result['log_lines']), (2, 2, 1))
        self.assertEqual(result['event_sequence_discontinuities'], 0)

    def test_log_retry_gap_reboot_and_sequence_wrap(self):
        log = FlightLog()
        log.feed(65535, b'wrap ')
        log.feed(0, b'works\n')
        self.assertEqual(log.last_line, 'wrap works')
        log.feed(1, b'prefix ')
        log.feed(1, b'prefix ')
        log.feed(3, b'tail\ncomplete\n')
        self.assertEqual(log.last_line, 'complete')
        self.assertEqual((log.gaps, log.duplicates), (1, 1))
        log.feed(4, b'old boot partial')
        log.feed(0, b'new boot\n')
        self.assertEqual(log.last_line, 'new boot')
        self.assertEqual(log.gaps, 1)

    def test_log_payload_and_memory_bounds(self):
        log = FlightLog()
        for payload in [b'', b'x' * 65]:
            with self.assertRaises(ValueError): log.feed(0, payload)
        for sequence in range(100): log.feed(sequence, b'x' * 64)
        self.assertLessEqual(len(log.pending), 4096)
        log.feed(100, b'tail\nvalid\n')
        self.assertEqual(log.last_line, 'valid')
    def test_malformed_records_fail_with_line_number(self):
        header = dict(format='star-log-v1', device=hex(FLIGHT))
        malformed = [[], {}, dict(self.row(), time_s=True), dict(self.row(), flags=True),
                     dict(self.row(), sequence=True), dict(self.row(), command=65536),
                     dict(self.row(), time_s=float('inf')), dict(self.row(), payload_hex='00 '*13),
                     dict(self.row(), payload_hex='a'*258), dict(self.row(), payload_hex='00'),
                     dict(self.row(), payload_hex='0700')]
        for row in malformed:
            with self.subTest(row=row), self.assertRaisesRegex(ValueError, 'line 2'):
                self.run_log([header, row])
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'large.jsonl'; path.write_text('a'*10000)
            with self.assertRaisesRegex(ValueError, 'line 1: line too large'): replay(path)

if __name__ == '__main__': unittest.main()
