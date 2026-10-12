import sys, time, json, struct
RUN = sys.argv[1] if len(sys.argv) > 1 else 'run3'
sys.path.insert(0, 'pygen')
from EMC.EmcStat import EmcStat, EmcStatT
frames = []
with open(RUN + '/frames.bin', 'rb') as f:
    while len(frames) < 4000:
        h = f.read(4)
        if not h: break
        (n,) = struct.unpack('<I', h); frames.append(f.read(n))
deltas = [l for i, l in enumerate(open(RUN + '/delta_1.jsonl')) if i < 4000]

def merge(t, p):
    for k, v in p.items():
        if v is None: t.pop(k, None)
        elif isinstance(v, dict) and isinstance(t.get(k), dict): merge(t[k], v)
        else: t[k] = v

t0 = time.perf_counter()
for b in frames:
    o = EmcStatT.InitFromObj(EmcStat.GetRootAs(b, 0))
t1 = time.perf_counter()
for b in frames:
    s = EmcStat.GetRootAs(b, 0); m = s.Motion(); tr = m.Traj(); p = tr.ActualPosition()
    _ = (p.X(), p.Y(), p.Z(), s.Task().CurrentLine(), m.Joint(0).Output())
t2 = time.perf_counter()
state = json.loads(deltas[0])
for l in deltas[1:]:
    merge(state, json.loads(l))
t3 = time.perf_counter()
n = len(frames)
print(f"python, {n} frames")
print(f"fbs full unpack      {1e6*(t1-t0)/n:8.1f} us/update")
print(f"fbs 5 field reads    {1e6*(t2-t1)/n:8.1f} us/update")
print(f"json delta + merge   {1e6*(t3-t2)/(len(deltas)-1):8.1f} us/update")
