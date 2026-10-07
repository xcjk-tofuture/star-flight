"""Protocol v1 bench tool; every command is explicit, no unlock/flight command."""
import argparse,json,math,time
from star_protocol import Frame,Parser,RESPONSE,STATUS,IDENTIFY,CAPABILITIES,PARAM_READ,PARAM_WRITE
from telemetry import (PLEIADES, FLIGHT, DEVICE_NAMES, CAP_STATUS, CAP_STORE, CAP_DIAGNOSTICS,
                       decode_u32, decode_status, decode_diagnostics, decode_parameter,
                       parameter_payload, require_length)

def prepare_operation(device, capabilities, operation, parameter=None, value=None):
    """Validate capability and value before emitting an operation request."""
    if operation == 'status' and capabilities & CAP_STATUS and device in DEVICE_NAMES:
        return STATUS, b''
    if operation == 'diagnostics' and capabilities & CAP_DIAGNOSTICS and device in (PLEIADES, FLIGHT):
        return (0x1002 if device == PLEIADES else 0x2001), b''
    if operation == 'save' and device == PLEIADES and capabilities & CAP_STORE:
        return 0x1001, b''
    if operation == 'write' and value is None:
        raise ValueError('write value required')
    if operation in ('read', 'write'):
        return (PARAM_READ if operation == 'read' else PARAM_WRITE), parameter_payload(
            device, capabilities, parameter, value if operation == 'write' else None)
    raise ValueError('operation not supported by this device/capability set')
def exchange(port,command,payload=b'',timeout=2.0,sequence=1):
    request=Frame(0,sequence,command,payload).encode()
    if port.write(request)!=len(request):raise IOError('partial serial write')
    parser=Parser();end=time.monotonic()+timeout
    while time.monotonic()<end:
        for frame in parser.feed(port.read(min(512,max(1,port.in_waiting))),int(time.monotonic()*1000)):
            if frame.flags==RESPONSE and frame.sequence==sequence and frame.command==command:
                if not frame.payload:raise ValueError('missing status')
                if frame.payload[0]:raise RuntimeError('device status '+str(frame.payload[0]))
                return frame.payload[1:]
    raise TimeoutError('no matching response; do not assume a timed-out save failed')
def main():
    p=argparse.ArgumentParser();p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=115200)
    sub=p.add_subparsers(dest='operation',required=True)
    for cmd in ['identify','capabilities','status','diagnostics','save']:sub.add_parser(cmd)
    read=sub.add_parser('read');read.add_argument('id',type=lambda v:int(v,0))
    write=sub.add_parser('write');write.add_argument('id',type=lambda v:int(v,0));write.add_argument('value',type=float)
    record=sub.add_parser('record');record.add_argument('output');record.add_argument('--seconds',type=float,default=10)
    a=p.parse_args()
    if a.baud<=0:p.error('baud must be positive')
    import serial
    with serial.Serial(a.port,a.baud,timeout=0.05,write_timeout=0.1) as port:
        ident=exchange(port,IDENTIFY);device=decode_u32(ident)
        if a.operation=='identify':print(hex(device));return
        capabilities=decode_u32(exchange(port,CAPABILITIES,sequence=2))
        if a.operation=='capabilities':print(json.dumps({'device':hex(device),'capabilities':capabilities}));return
        if a.operation=='record':
            if not math.isfinite(a.seconds) or not 0<a.seconds<=3600:p.error('record duration must be 0..3600s')
            prepare_operation(device,capabilities,'status')
            parser=Parser();start=time.monotonic();end=start+a.seconds
            with open(a.output,'x',encoding='utf-8') as f:
                f.write(json.dumps({'format':'star-log-v1','device':hex(device),'baud':a.baud,'capabilities':capabilities,'time_basis':'elapsed_monotonic_s'})+'\n')
                while time.monotonic()<end:
                    for frame in parser.feed(port.read(512),int(time.monotonic()*1000)):
                        f.write(json.dumps({'time_s':time.monotonic()-start,'flags':frame.flags,'sequence':frame.sequence,'command':frame.command,'payload_hex':frame.payload.hex()})+'\n')
            return
        command,payload=prepare_operation(device,capabilities,a.operation,getattr(a,'id',None),getattr(a,'value',None))
        result=exchange(port,command,payload,timeout=15 if a.operation=='save' else 2,sequence=3)
        if a.operation=='status':data=decode_status(device,result)
        elif a.operation=='diagnostics':data=decode_diagnostics(device,result)
        elif a.operation=='read':data=decode_parameter(a.id,result)
        else:require_length(result,0);data={'accepted':True}
        print(json.dumps({'device':hex(device),'command':hex(command),'data':data,'data_hex':result.hex()}))
if __name__=='__main__':
    try:main()
    except (OSError,ValueError,RuntimeError) as error:
        import sys
        sys.exit(str(error))
