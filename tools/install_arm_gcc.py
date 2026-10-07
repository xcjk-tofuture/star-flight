"""Install exactly ARM GNU 13.3.Rel1 for CI/local Linux x86_64, verify official SHA256."""
from pathlib import Path
import argparse,hashlib,platform,tarfile,urllib.request
p=argparse.ArgumentParser();p.add_argument('destination',type=Path);a=p.parse_args()
if platform.system()!='Linux' or platform.machine()!='x86_64':p.error('Linux x86_64 only; Windows use the documented verified package')
name='arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi'
url='https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/'+name+'.tar.xz'
expected='95c011cee430e64dd6087c75c800f04b9c49832cc1000127a92a97f9c8d83af4'
a.destination.mkdir(parents=True,exist_ok=True);archive=a.destination/(name+'.tar.xz')
urllib.request.urlretrieve(url,archive)
if hashlib.sha256(archive.read_bytes()).hexdigest()!=expected:raise RuntimeError('ARM archive SHA256 mismatch')
with tarfile.open(archive) as t:t.extractall(a.destination,filter='data')
print(a.destination.resolve()/name)
