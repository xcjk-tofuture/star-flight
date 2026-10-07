import sys,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from star_protocol import Frame,Parser
from serial_cli import exchange
class Port:
    def __init__(self,data):self.data=bytearray(data);self.written=b''
    def write(self,b):self.written=b;return len(b)
    @property
    def in_waiting(self):return len(self.data)
    def read(self,n):
        result=bytes(self.data[:min(n,3)]);del self.data[:len(result)];return result
class ToolsTests(unittest.TestCase):
    def test_matching_response_ignores_event_and_other_sequence(self):
        p=Port(Frame(2,1,4,b'\x00').encode()+Frame(1,5,4,b'\x00').encode()+Frame(1,1,4,b'\x00ok').encode())
        self.assertEqual(exchange(p,4),b'ok');self.assertEqual(p.written,Frame(0,1,4).encode())
    def test_error_and_missing_status(self):
        with self.assertRaises(RuntimeError):exchange(Port(Frame(1,1,4,b'\x04').encode()),4)
        with self.assertRaises(ValueError):exchange(Port(Frame(1,1,4).encode()),4)
    def test_bounded_stream(self):
        p=Parser();p.feed(b'\xa5'*10000,0);self.assertLessEqual(len(p.buffer),140)
    def test_timeout(self):
        with self.assertRaises(TimeoutError):exchange(Port(b''),4,timeout=0)

class ReplayTests(unittest.TestCase):
    def test_offline_timeline_and_invalid_data(self):
        import tempfile,json,struct
        from replay_log import replay
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'capture.jsonl'
            header={'format':'star-log-v1','device':'0x58550001'}
            rows=[header]
            for i,state in enumerate([0,0,1,2,3]):
                rows.append({'time_s':float(i),'flags':2,'sequence':i,'command':4,'payload_hex':(bytes([0,state])+struct.pack('<fff',0,0,0)).hex()})
            p.write_text('\n'.join(json.dumps(r) for r in rows),encoding='utf-8')
            self.assertEqual([s['state'] for s in replay(p)['states']],[0,1,2,3])
            rows[-1]['time_s']=-1
            p.write_text('\n'.join(json.dumps(r) for r in rows),encoding='utf-8')
            with self.assertRaises(ValueError):replay(p)

if __name__=='__main__':unittest.main()
