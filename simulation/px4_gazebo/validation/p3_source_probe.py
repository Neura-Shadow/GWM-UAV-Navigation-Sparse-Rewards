"""Ground evaluator-only Gazebo header ledger; never feeds the adapter."""
import json
from pathlib import Path
import signal
import sys
import threading
import time


def main():
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    run=Path(sys.argv[1]); run_id=sys.argv[2]; stopping=threading.Event()
    actual=run.parent.name if run.name=='sensors' else run.name
    if run_id!=actual or actual=='sensors': raise ValueError('source_probe_run_identity')
    stream=(run/'gazebo-source-headers.jsonl').open('x',buffering=1)
    node=Node()
    count=0
    def receive(msg):
        nonlocal count
        count+=1
        stream.write(json.dumps(dict(id=count,run_id=run_id,sample_evidence_contract='p3-sample-evidence-v2',wall_s=time.monotonic(),
            acquisition_sim_ns=int(msg.header.stamp.sec)*1000000000+int(msg.header.stamp.nsec),
            acquisition_sim_s=msg.header.stamp.sec+msg.header.stamp.nsec/1e9,
            header_data={p.key:list(p.value) for p in msg.header.data},
            width=msg.width,height=msg.height,step=msg.step,pixel_format=msg.pixel_format_type),allow_nan=False)+'\n')
    node.subscribe(Image,'/depth_camera',receive)
    def stop(*_): stopping.set()
    signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
    stopping.wait()
    node.unsubscribe('/depth_camera')
    stream.close()


if __name__=='__main__': main()
