"""Bounded load generators for the stress tests (diagnostics only). Every worker exits by itself after SECONDS.

  load.py cpu WORKERS SECONDS   busy-loop processes (run inside WSL for guest oversubscription, or with a
                                Windows Python for host contention)
  load.py gpu SECONDS           continuous large float32 matmuls on the GPU (needs torch with CUDA)
"""
import multiprocessing
import sys
import time


def spin(seconds):
    end = time.time() + seconds
    x = 0
    while time.time() < end:
        x += 1


def gpu(seconds):
    import torch
    a = torch.randn(4096, 4096, device="cuda")
    end = time.time() + seconds
    while time.time() < end:
        for _ in range(20):
            a = (a @ a).clamp_(-1, 1)
        torch.cuda.synchronize()


if __name__ == "__main__":
    if sys.argv[1] == "cpu":
        workers, seconds = int(sys.argv[2]), float(sys.argv[3])
        procs = [multiprocessing.Process(target=spin, args=(seconds,)) for _ in range(workers)]
        for p in procs:
            p.start()
        for p in procs:
            p.join()
    else:
        gpu(float(sys.argv[2]))
