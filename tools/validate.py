"""Reproducible host checks and ARM build report; never initializes a probe."""
from pathlib import Path
import argparse,hashlib,json,os,re,shutil,struct,subprocess,sys
def elf_layout(path,flash_end):
    b=path.read_bytes()
    if b[:7]!=b'\x7fELF\x01\x01\x01':raise ValueError('expected ELF32 little endian')
    header=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
    if header[2]!=40:raise ValueError('expected ARM machine')
    loads=[]
    for i in range(header[10]):
        p=struct.unpack_from('<IIIIIIII',b,header[5]+i*header[9])
        if p[0]!=1:continue
        _,offset,va,pa,filesz,memsz,flags,align=p
        if filesz>memsz or offset+filesz>len(b) or flags&3==3:raise ValueError('invalid segment or RWX')
        if filesz and not(0x08000000<=pa<flash_end and pa+filesz<=flash_end):raise ValueError('load image overlaps reserved flash/outside device')
        if not(0x08000000<=va<flash_end or 0x20000000<=va<0x20020000):raise ValueError('invalid virtual address')
        loads.append(dict(vaddr=hex(va),paddr=hex(pa),file_bytes=filesz,memory_bytes=memsz,flags=flags))
    if len(loads)!=2 or loads[0]['flags']!=5 or loads[1]['flags']!=6:raise ValueError('expected distinct RX and RW segments')
    # Startup explicitly copies .data and clears .bss; NOLOAD reservations are not flashed.
    return loads
def main():
    p=argparse.ArgumentParser();p.add_argument('--cc',default=shutil.which('gcc') or 'gcc')
    p.add_argument('--preset',choices=['debug','release']);p.add_argument('--cmake',default=shutil.which('cmake') or 'cmake')
    a=p.parse_args();root=Path(__file__).resolve().parents[1];report=root/'build/reports';report.mkdir(parents=True,exist_ok=True)
    def run(args):
        r=subprocess.run(args,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        if r.returncode:print(r.stdout.decode(errors='replace'));raise subprocess.CalledProcessError(r.returncode,args)
        return r.stdout.decode(errors='replace')
    if not a.preset:
        logs=[]
        for script in ['run_host.py','run_pid.py','run_math.py','run_todo.py']:
            logs.append(run([sys.executable,str(root/'tests'/script),'--cc',a.cc]))
        logs.append(run([sys.executable,'-m','unittest','discover','-s','tests','-p','test_*tools.py']))
        if (root/'ros').exists():logs.append(run([sys.executable,'-m','unittest','discover','-s','ros/ws_starbot/starbot_serial/test','-p','test_star_protocol.py']))
        (report/'host-tests.txt').write_text(''.join(logs),encoding='utf-8');print(''.join(logs));return
    run([a.cmake,'--preset',a.preset]);log=run([a.cmake,'--build','--preset',a.preset,'--clean-first','--parallel','4'])
    (report/(a.preset+'.log')).write_text(log,encoding='utf-8')
    if re.search(r"can't be allocated|lma .* adjusted|undefined reference|overflowed",log):raise RuntimeError('linker diagnostic')
    car=(root/'firmware/services/chassis_service.c').exists();target='maoxiu_stm32' if car else 'xingxuan_uav'
    firmware=root/'build'/a.preset/(target+'.elf');layout=elf_layout(firmware,0x08040000 if car else 0x08080000)
    result={'preset':a.preset,'commit':run(['git','rev-parse','HEAD']).strip(),'dirty':bool(run(['git','status','--porcelain']).strip()),
        'flash_bytes':int(re.search(r'FLASH:\s*(\d+)',log)[1]),'ram_reserved_bytes':int(re.search(r'RAM:\s*(\d+)',log)[1]),
        'compiler_warnings':log.count('warning:'),'load_segments':layout,'hardware':'NOT RUN',
        'sha256':{x:hashlib.sha256((firmware.with_suffix('.'+x)).read_bytes()).hexdigest() for x in ['elf','hex','bin','map']}}
    (report/(a.preset+'.json')).write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
