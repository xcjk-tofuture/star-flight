"""Host codec for Common/protocol v1; independent of serial and ROS."""
from dataclasses import dataclass
import struct

MAX_PAYLOAD = 128
TIMEOUT_MS = 100
REQUEST, RESPONSE, EVENT = 0, 1, 2
VERSION, IDENTIFY, CAPABILITIES, STATUS, PARAM_READ, PARAM_WRITE = range(1, 7)
CHASSIS_VELOCITY = 0x1000

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc

@dataclass(frozen=True)
class Frame:
    flags: int
    sequence: int
    command: int
    payload: bytes = b''

    def encode(self):
        if self.flags not in (REQUEST, RESPONSE, EVENT) or len(self.payload) > MAX_PAYLOAD:
            raise ValueError('invalid flags or payload length')
        body = struct.pack('<BBHHH', 1, self.flags, self.sequence, self.command, len(self.payload)) + self.payload
        return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))

class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.last_ms = 0
        self.rejected = 0
        self.timed_out = 0

    def expire(self, now_ms):
        if self.buffer and ((now_ms - self.last_ms) & 0xFFFFFFFF) >= TIMEOUT_MS:
            self.buffer.clear()
            self.timed_out += 1

    def feed(self, data, now_ms):
        self.expire(now_ms)
        frames = []
        for byte in data:
            if len(self.buffer) == MAX_PAYLOAD + 12:
                del self.buffer[0]
                self.rejected += 1
            self.buffer.append(byte)
            self.last_ms = now_ms
            while self.buffer:
                if self.buffer[0] != 0xA5:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 2:
                    break
                if self.buffer[1] != 0x5A:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 10:
                    break
                version, flags, sequence, command, length = struct.unpack_from('<BBHHH', self.buffer, 2)
                if version != 1 or flags > EVENT or length > MAX_PAYLOAD:
                    del self.buffer[0]
                    self.rejected += 1
                    continue
                total = length + 12
                if len(self.buffer) < total:
                    break
                if struct.unpack_from('<H', self.buffer, total - 2)[0] != crc16(self.buffer[2:total - 2]):
                    del self.buffer[0]
                    self.rejected += 1
                    continue
                frames.append(Frame(flags, sequence, command, bytes(self.buffer[10:total - 2])))
                del self.buffer[:total]
        return frames

def decode_chassis_status(payload):
    if len(payload) != 31 or payload[0] != 0:
        raise ValueError('invalid chassis telemetry')
    return (payload[1], *struct.unpack_from('<fff', payload, 2), payload[14],
            struct.unpack_from('<f', payload, 15)[0], *struct.unpack_from('<hhhhhh', payload, 19))
