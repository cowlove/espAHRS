#!/usr/bin/env python3
import csv, math, struct, subprocess, tempfile
import numpy as np
import matplotlib.pyplot as plt

CAL_LOG = '/home/jim/.openclaw/workspace/media/inbound/openclaw-staged-321df9d8-c9d5-495b-ae0d-176530f96964/G247C025---e4953cd3-51ef-4b2d-a987-608a9abe370f.bin'
FLIGHT_LOG = 'flight-data-latest/20260818/G247C021.bin'
H = struct.Struct('<I4xQI B3x I4x')
C = struct.Struct('<Q3fB3x')
raw = []
with open(CAL_LOG, 'rb') as f:
    while h := f.read(32):
        _, _, _, typ, n = H.unpack(h)
        d = f.read(n)
        if typ == 7:
            q = C.unpack(d)
            if q[4]: raw.append((q[0] / 1e6, np.array(q[1:4], float)))
x = np.array([v for _, v in raw])
M = np.column_stack((x[:,0]**2, x[:,1]**2, x[:,2]**2,
                     2*x[:,0]*x[:,1], 2*x[:,0]*x[:,2], 2*x[:,1]*x[:,2],
                     x[:,0], x[:,1], x[:,2], np.ones(len(x))))
_, _, vh = np.linalg.svd(M, full_matrices=False)
p = vh[-1]
Q = np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])
o = -0.5 * np.linalg.solve(Q, p[6:9])
k = 1.0 - o @ Q @ o - p[9]
Q = Q / k
ev, U = np.linalg.eigh(Q)
A = U @ np.diag(np.sqrt(np.maximum(ev, 0))) @ U.T
norm = np.linalg.norm((A @ (x - o).T).T, axis=1)
print('offset', o)
print('matrix', A)
print('normalized magnitude mean/std', norm.mean(), norm.std())
F = np.diag([-1, 1, 1])
decl = 14.89
with tempfile.NamedTemporaryFile(suffix='.csv') as tf:
    subprocess.run(['./replay', FLIGHT_LOG, '--device-mac', '247C', '--param',
                    'compass_source=0', '--heading-csv', tf.name],
                   check=True, stdout=subprocess.DEVNULL)
    rr = list(csv.DictReader(open(tf.name)))
flight_raw = []
with open(FLIGHT_LOG, 'rb') as f:
    while h := f.read(32):
        _, _, _, typ, n = H.unpack(h); d = f.read(n)
        if typ == 7:
            q = C.unpack(d)
            if q[4]: flight_raw.append((q[0] / 1e6, np.array(q[1:4], float)))
gps_t, gps, ct, ch = [], [], [], []
rows = [r for r in rr if int(r['compass_valid'])]
for row, (t, v) in zip(rows, flight_raw):
    z = F @ (A @ (v - o))
    roll = math.radians(float(row['ahrs_roll_deg']))
    pitch = math.radians(float(row['ahrs_pitch_deg']))
    ly = math.cos(roll)*z[1] - math.sin(roll)*z[2]
    rz = math.sin(roll)*z[1] + math.cos(roll)*z[2]
    lx = math.cos(pitch)*z[0] + math.sin(pitch)*rz
    ch.append((decl + math.degrees(math.atan2(-ly, lx))) % 360)
    ct.append(t)
    if int(row['gps_valid']):
        gps_t.append(t); gps.append(float(row['gps_track_deg']))
def unwrap(v):
    out = [v[0]]
    for a, b in zip(v[1:], v): out.append(out[-1] + (a-b+180) % 360 - 180)
    return out
plt.figure(figsize=(13, 5.5))
plt.plot(gps_t, unwrap(gps), label='GPS track', lw=1.1)
plt.plot(ct, unwrap(ch), label='Compass 0 new tumble fit', lw=0.8)
plt.grid(alpha=0.3); plt.legend(); plt.xlabel('log time (s)')
plt.ylabel('heading change (deg)'); plt.tight_layout()
plt.savefig('analysis-plots/G247C025-compass0-new-fit.png', dpi=150)
