"""Typed protocol-v1 data, independent of serial I/O and firmware internals."""
import math
import struct

PLEIADES, TM4C, FLIGHT = 0x4D530001, 0x4D540001, 0x58550001
CAP_STATUS, CAP_PERIOD, CAP_CHASSIS, CAP_STORE, CAP_DIAGNOSTICS = 1, 2, 4, 8, 16
DEVICE_NAMES = {PLEIADES: 'StarPleiades STM32', TM4C: 'StarPleiades TM4C', FLIGHT: 'StarFlight'}
PARAMETERS = {
    0x100: ('kp', 0, 10000), 0x101: ('ki', 0, 2000),
    0x102: ('span_m', .05, 1), 0x103: ('pwm_limit', 1, 16700),
    0x104: ('minimum_voltage_v', 6, 24), 0x105: ('command_timeout_ms', 50, 500),
    0x106: ('speed_scale', .25, 4),
}

def require_length(data, length):
    if len(data) != length:
        raise ValueError(f'expected {length} data bytes, got {len(data)}')

def decode_u32(data):
    require_length(data, 4)
    return struct.unpack('<I', data)[0]

def decode_status(device, data):
    """Data excludes the response status byte; IMU counts remain unscaled."""
    if device in (PLEIADES, TM4C):
        require_length(data, 30)
        state, vx, vy, wz, wheels, voltage, *imu = struct.unpack('<BfffBf6h', data)
        states = ['idle', 'active', 'undervoltage', 'timeout', 'uncalibrated']
        if state >= (5 if device == TM4C else 4) or wheels not in (2, 4):
            raise ValueError('invalid chassis state or wheel count')
        result = dict(state=state, state_name=states[state], vx_m_s=vx, vy_m_s=vy,
                      wz_rad_s=wz, wheels=wheels, voltage_v=voltage,
                      acceleration_counts=imu[:3], gyro_counts=imu[3:])
        values = (vx, vy, wz, voltage)
    elif device == FLIGHT:
        require_length(data, 13)
        state, roll, pitch, yaw = struct.unpack('<Bfff', data)
        if state > 3:
            raise ValueError('invalid flight state')
        result = dict(state=state, state_name=['locked', 'armed', 'stabilize', 'emergency'][state],
                      roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw)
        values = (roll, pitch, yaw)
    else:
        raise ValueError('unsupported status device')
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite telemetry')
    return result

def decode_diagnostics(device, data):
    if device == PLEIADES:
        require_length(data, 29)
        values = struct.unpack('<7IB', data)
        if values[-1] not in (0, 1):
            raise ValueError('invalid loaded flag')
        return dict(zip(['revision', 'saved_sequence', 'rx_lost', 'parser_rejected',
                         'parser_timeouts', 'tx_failed', 'control_overruns', 'loaded'], values))
    if device == FLIGHT:
        require_length(data, 22)
        fault, transitions, rejected, imu, remote, pid, busy = struct.unpack('<B5IB', data)
        if fault & ~31 or busy not in (0, 1):
            raise ValueError('invalid fault or busy flags')
        return dict(fault=fault, transitions=transitions, rejected_writes=rejected,
                    sequences=dict(imu=imu, remote=remote, pid=pid), storage_busy=bool(busy))
    raise ValueError('unsupported diagnostics device')

def parameter_payload(device, capabilities, parameter, value=None):
    if type(parameter) is not int or not 0 <= parameter <= 65535:
        raise ValueError('parameter id out of range')
    if parameter == 1:
        if not capabilities & CAP_PERIOD or device not in DEVICE_NAMES:
            raise ValueError('telemetry period unsupported')
        minimum = 20 if device == FLIGHT else 50
        if value is not None and (not math.isfinite(value) or
                                 not minimum <= value <= 1000 or int(value) != value):
            raise ValueError(f'telemetry period must be integer {minimum}..1000 ms')
        return struct.pack('<H', parameter) + (struct.pack('<H', int(value)) if value is not None else b'')
    if device != PLEIADES or not capabilities & CAP_STORE or parameter not in PARAMETERS:
        raise ValueError('unsupported parameter')
    if value is not None:
        name, low, high = PARAMETERS[parameter]
        if not math.isfinite(value) or not low <= value <= high or (parameter == 0x105 and int(value) != value):
            raise ValueError(f'{name} must be {low}..{high}' + (' integer' if parameter == 0x105 else ''))
    return struct.pack('<H', parameter) + (struct.pack('<f', value) if value is not None else b'')

def decode_parameter(parameter, data):
    require_length(data, 2 if parameter == 1 else 4)
    value = struct.unpack('<H' if parameter == 1 else '<f', data)[0]
    if not math.isfinite(value):
        raise ValueError('nonfinite parameter')
    return dict(id=parameter, name='telemetry_period_ms' if parameter == 1 else PARAMETERS[parameter][0], value=value)
