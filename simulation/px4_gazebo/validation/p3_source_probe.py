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
    run=Path(sys.argv[1]); stopping=threading.Event()
    stream=(run/'gazebo-source-headers.jsonl').open('x',buffering=1)
    node=Node()
    count=0
    def receive(msg):
        nonlocal count
        count+=1
        stream.write(json.dumps(dict(id=count,wall_s=time.monotonic(),
            acquisition_sim_s=msg.header.stamp.sec+msg.header.stamp.nsec/1e9,
            header_data={p.key:list(p.value) for p in msg.header.data},
            width=msg.width,height=msg.height,step=msg.step,pixel_format=msg.pixel_format_type))+'\n')
    node.subscribe(Image,'/depth_camera',receive)
    def stop(*_): stopping.set()
    signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
    stopping.wait()
    node.unsubscribe('/depth_camera')
    stream.close()


if __name__=='__main__': main()
