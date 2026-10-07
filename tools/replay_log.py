"""Bounded offline log validation and decoded summary; never opens a port."""
import json
import math
from telemetry import DEVICE_NAMES, PLEIADES, FLIGHT, decode_status, decode_diagnostics

class FlightLog:
    """LOG events carry raw UTF-8 chunks, without a response status byte."""
    def __init__(self):
        self.pending = bytearray()
        self.last_sequence = None
        self.discard_tail = False
        self.frames = self.lines = self.gaps = self.duplicates = 0
        self.last_line = None

    def feed(self, sequence, payload):
        if not 1 <= len(payload) <= 64:
            raise ValueError('LOG payload must be 1..64 bytes')
        self.frames += 1
        if sequence != 0 and sequence == self.last_sequence:
            self.duplicates += 1
            return
        if self.last_sequence is not None and sequence != (self.last_sequence + 1) & 65535:
            self.pending.clear()
            self.discard_tail = sequence != 0
            self.gaps += sequence != 0
        self.last_sequence = sequence
        if self.discard_tail:
            end = payload.find(b'\n')
            if end < 0:
                return
            self.discard_tail = False
            payload = payload[end + 1:]
        self.pending.extend(payload)
        while b'\n' in self.pending:
            end = self.pending.index(10) + 1
            self.last_line = self.pending[:end].decode('utf-8', errors='replace').rstrip('\r\n')
            del self.pending[:end]
            self.lines += 1
        if len(self.pending) > 4096:
            self.pending.clear()
            self.discard_tail = True

def integer(value, maximum):
    return type(value) is int and 0 <= value <= maximum

def records(stream):
    line_no = 0
    while True:
        line = stream.readline(4097)
        if not line:
            return
        line_no += 1
        try:
            if len(line) > 4096:
                raise ValueError('line too large')
            record = json.loads(line)
            if type(record) is not dict:
                raise ValueError('expected an object')
        except (ValueError, RecursionError) as error:
            raise ValueError(f'line {line_no}: {error}') from error
        yield line_no, record

def replay(path):
    result = dict(frames=0, events=0, error_frames=0, states=[], last_time_s=None,
                  status_frames=0, max_status_gap_s=0, event_sequence_discontinuities=0,
                  estimated_missing_status_events=0)
    device = None; last_status_time = None; last_sequence = None
    flight_log = FlightLog()
    with open(path, encoding='utf-8') as f:
        for line_no, r in records(f):
            try:
                if line_no == 1:
                    if r.get('format') != 'star-log-v1' or r.get('device') not in [hex(d) for d in DEVICE_NAMES]:
                        raise ValueError('unsupported log version or device')
                    device = int(r['device'], 16)
                    if 'baud' in r and (type(r['baud']) is not int or r['baud'] <= 0):
                        raise ValueError('invalid baud')
                    if 'capabilities' in r and not integer(r['capabilities'], 0xffffffff):
                        raise ValueError('invalid capabilities')
                    if 'time_basis' in r and r['time_basis'] != 'elapsed_monotonic_s':
                        raise ValueError('unsupported time basis')
                    continue
                t, flags, sequence, command = r['time_s'], r['flags'], r['sequence'], r['command']
                if type(t) not in (float, int) or not math.isfinite(t) or t < 0 or not integer(flags, 2):
                    raise ValueError('invalid timestamp or flags')
                if not integer(sequence, 65535) or not integer(command, 65535):
                    raise ValueError('invalid frame identifier')
                encoded = r['payload_hex']
                if type(encoded) is not str or len(encoded) > 256 or len(encoded) % 2:
                    raise ValueError('invalid payload length')
                payload = bytes.fromhex(encoded)
                if payload.hex() != encoded.lower():
                    raise ValueError('payload must be contiguous hexadecimal')
                if result['last_time_s'] is not None and t < result['last_time_s']:
                    raise ValueError('timestamps out of order')
                result['frames'] += 1; result['events'] += flags == 2; result['last_time_s'] = t
                if flags == 0:
                    continue
                if device == FLIGHT and command == 0x20f0 and flags == 2:
                    flight_log.feed(sequence, payload)
                    continue
                if not payload or payload[0] > 6:
                    raise ValueError('invalid response status')
                if payload[0]:
                    if len(payload) != 1:
                        raise ValueError('error response has unexpected data')
                    result['error_frames'] += 1; continue
                if command == 4 or (device == FLIGHT and command == 0x2000):
                    status = decode_status(device, payload[1:])
                    result['last_status'] = status; result['status_frames'] += 1
                    if command == 4 and flags == 2:
                        if last_status_time is not None:
                            result['max_status_gap_s'] = max(result['max_status_gap_s'], t-last_status_time)
                        if last_sequence is not None:
                            delta = (sequence-last_sequence) & 65535
                            if delta != 1:
                                result['event_sequence_discontinuities'] += 1
                                if 1 < delta < 32768:
                                    result['estimated_missing_status_events'] += delta-1
                        last_sequence = sequence; last_status_time = t
                    if not result['states'] or result['states'][-1]['state'] != status['state']:
                        result['states'].append(dict(time_s=t, state=status['state'], state_name=status['state_name']))
                elif command == (0x1002 if device == PLEIADES else 0x2001) and device in (PLEIADES, FLIGHT):
                    result['last_diagnostics'] = decode_diagnostics(device, payload[1:])
            except (ValueError, KeyError, TypeError, OverflowError) as error:
                raise ValueError(f'line {line_no}: {error}') from error
    if device is None:
        raise ValueError('empty log')
    result['device'] = hex(device)
    if device == FLIGHT:
        result.update(log_frames=flight_log.frames, log_lines=flight_log.lines,
                      log_sequence_gaps=flight_log.gaps, log_duplicates=flight_log.duplicates,
                      last_log=flight_log.last_line)
    return result

def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('log');a=p.parse_args()
    try: print(json.dumps(replay(a.log),indent=2))
    except (OSError,ValueError) as error: p.exit(1,str(error)+'\n')
if __name__=='__main__':main()
