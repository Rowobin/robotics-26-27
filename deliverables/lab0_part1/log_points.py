#!/usr/bin/env python3

import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from xarm.wrapper import XArmAPI
from lite6 import session

### ROBOT CODE
arm = session.safe_session()
arm.clean_warn()
arm.clean_error()
arm.motion_enable(enable=True)

arm.set_tcp_load(0.3, [0, 0, 90])

arm.set_mode(2) # manual mode
arm.set_state(0)

# wait for input
input('waiting')

t0 = time.monotonic()
times, rows = [], []
while True:
    t = time.monotonic() - t0
    if t >= 30.0:
        break
    code, angles = arm.get_servo_angle()
    if code == 0:
        times.append(t)
        rows.append(list(angles[:6]))

arm.disconnect()

### MATH
times = np.asarray(times)
rows = np.asarray(rows)
n = len(times)
span = times[-1] - times[0]
rate = (n - 1) / span 
print(f'{n} samples in {span:.2f} s.\nsample rate: {rate:.1f} Hz')

### EXPORT READINGS
with open('log_joints.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t_s', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
    for t, a in zip(times, rows):
        w.writerow([f'{t:.4f}'] + [f'{v:.3f}' for v in a])
print(f'csv saved')

### PLOT
OUT_CSV = 'log_joints.csv'
OUT_PNG = 'log_joints.png'
plt.figure(figsize=(10, 5))
for j in range(6):
    plt.plot(times, rows[:, j], label=rf'$\theta_{j + 1}$')
plt.xlabel('t (s)')
plt.ylabel('joint angle (deg)')
plt.title(f'Lite 6 joint angles, 30.0s hand-guiding 'f'({rate:.1f} Hz, {n} samples)')
plt.legend(loc='upper right', ncol=6)
plt.grid(True)
plt.tight_layout()
plt.savefig('log_joints.png', dpi=200)
print(f'png saved')
plt.show()