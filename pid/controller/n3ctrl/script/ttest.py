import numpy as np
import matplotlib
import matplotlib.pyplot as plt

dt = 1.0 / 120.0
T = 30
N = int(round(T/dt))
t = np.linspace(0, T-2*dt, N-1)
print (t.shape)
x_d = np.stack((np.zeros(N-1), np.zeros(N-1), np.ones(N-1)*.5, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))
x_d_2 = np.stack((np.sin(t*1.2-3), 1+np.cos(t*1.2-3-np.pi), np.ones(N-1)*.5, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))
x_d_2[0,:300] = 0.
x_d_2[1,:300] = 0.
x_d_2[0,3442:] = 0.
x_d_2[1,3442:] = 0.
# x_d_2[1,800:] = 2.
# print (x_d_2[0,295:320])
# print (x_d_2[1,295:320])
# print (x_d_2[0,3440:3460])
# print (x_d_2[1,3440:3460])
# print (x_d_2[1,3440-157:3460-157])

plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
# plt.subplot(211)
plt.plot(t,x_d_2[0],'r-',alpha=0.9)
plt.plot(t,x_d_2[1],'g-',alpha=0.9)
plt.plot(t,x_d_2[2],'b-',alpha=0.9)
plt.show()