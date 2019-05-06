
import TrajectoryPlanning as tp
import numpy as np
import time
import matplotlib.pyplot as plt


# Quadratic
a = tp.calculateTrajectory([0, 0, 2, 0], [0, 15])


# LSPB

# V, constraints, t
# [q(t0), d(q(t0)), q(tf), d(q(tf))], t = [t0, tf]
if not tp.LSPB(-0.2, [2, 0, 0, 0], [0, 15]):
	print("This is invalid: (-0.2, [2, 0, 0, 0], [0, 15])")
	
[A0, A1, A2, tb] = tp.LSPB(-0.2, [2, 0, 0, 0], [0, 15])
print("tb = ", tb)
print("A2 = ", A2)
tf = 15

for i in range(16):
	print(i, ": ", tp.getLSPB_position(A0, A1, A2, tb, tf, i))


tstart = round(time.time(),2)
t0 = 0
tf = 15
time_aug = (round(time.time(),2) - tstart)
iterator = 0
time_list = []
data_list = []
data_list2 = []

if (0 <= time_aug < tf+1):
	while (True):
		data_list.append(tp.getLSPB_position(A0, A1, A2, tb, tf, time_aug))
		time_list.append(time_aug)
		data_list2.append(tp.getTrajectoryPosition(a, time_aug))
		print(time_aug, ": ", data_list[-1])
		time.sleep(0.1)
		time_aug = (round(time.time(),2) - tstart)
		if (time_aug > tf + 1):
			break
else:
	print("False, time_aug =", time_aug)


# PLOT: 
plt.plot(time_list, data_list)

plt.plot(time_list, data_list2, 'r')
plt.show()


# Mode 1
# Move from position to next position, r2 is controlled, using LSPB
# V, constraints, t
# [q(t0), d(q(t0)), q(tf), d(q(tf))], t = [t0, tf]
# [read_r2, 0, r2_max, 0], t = [0, 20], V = 0.5 m/s
#[A0, A1, A2, tb] = tp.LSPB(V1, [read_r2, 0, r2_max, 0], [t0, tf])

# Mode 2
# Move from position to next position, theta4 is controlled, using LSPB
# [q(t0), d(q(t0)), q(tf), d(q(tf))], t = [t0, tf]
# [read_Theta4, 0, theta4_next, 0], t = [0, 10], V = 0.2 m/s











