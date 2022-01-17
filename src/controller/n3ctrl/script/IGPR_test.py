from IGPR import IGPR
import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import time as time
from math import pi as PI
from math import atan2, sin, cos, sqrt, fabs, floor, exp
import random

N = 1002     #dataset size
length = 30.0
t = np.arange(0, length, length/N)
xall = np.vstack((np.sin(t), np.cos(t), t**2, t**3, t+t**3, t**2-t, np.exp(t), np.log(t+1)))    #8*N
yall = (t**3 - t**2*38 + 392.03*t - 898.144) / 1675.75 + np.sin(t*2) + np.random.normal(0, 0, size=N)

#data normalization
for i in range(len(xall)):
    xmin, xmax = np.min(xall[i, :]), np.max(xall[i, :])
    xall[i, :] = (xall[i, :] - xmin) / (xmax - xmin)
ymin, ymax = np.min(yall), np.max(yall)
yall = (yall - ymin) / (ymax - ymin)

ytruehist = np.empty((0,1))
ypredhist = np.empty((0,1))
cov = np.empty((0,1))
train_time = []
pred_time = []

#fit and predict
t1 = time.time()
# igpr = IGPR(xall[:,0], yall[0], update_mode="FIFO", alpha=1e-6, optimize=False)
igpr = IGPR(update_mode="FIFO", alpha=1e-6, optimize=False)
igpr.learn(xall[:,0], yall[0])
begin_time = time.time()
for i in range(1,N-1):
    temp_time = time.time()
    igpr.learn(xall[:,i], yall[i])
    train_time.append(time.time() - temp_time)
    temp_time = time.time()
    ypredict, ycov = igpr.predict(xall[:,i+1])
    pred_time.append(time.time() - temp_time)
    ytruehist = np.vstack((ytruehist, yall[i+1]))
    ypredhist = np.vstack((ypredhist, ypredict))
    cov = np.vstack((cov, ycov))
    if i % 20 == 0:
        igpr.hyperparam_optimization()
print(time.time() - t1)


#test result
time_using = time.time() - begin_time
print("time using: ", time_using)
error = ytruehist[:,0] - ypredhist[:,0]
print("mean error:", np.mean(abs(error)))
print("max error:", max(abs(error)))
#95%: 1.96
uncertainty = 1.96 * np.sqrt(np.abs(cov))
error_out_of_confidence_interval = []
for i in range(len(uncertainty)):
    error_out_of_confidence_interval.append(max(ytruehist[i,0]-ypredhist[i,0]-uncertainty[i,0], ypredhist[i,0]-uncertainty[i,0]-ytruehist[i,0], 0))
print("mean error_out_of_confidence_interval:", np.mean(np.abs(error_out_of_confidence_interval)))
print("max error_out_of_confidence_interval:", max(np.abs(error_out_of_confidence_interval)))


plt.close('all')
plt.plot(train_time, label="train time")
plt.show()
plt.plot(pred_time, label="pred time")
plt.show()

plt.plot(range(0,ytruehist.shape[0]), ytruehist[:,0], color="red", label="true")
plt.plot(range(0,ypredhist.shape[0]), ypredhist[:,0], color="blue", label="pred")
plt.fill_between(range(0, ytruehist.shape[0]), ypredhist[:,0] + uncertainty[:,0], ypredhist[:,0] - uncertainty[:,0], alpha = 0.2)
plt.show()

plt.figure()
plt.plot(ytruehist[:,0] - ypredhist[:,0], color="blue", label="error")
plt.plot(error_out_of_confidence_interval, color="red", label="error_out_of_confidence_interval")
plt.title("predictive error")
plt.legend()
plt.show()
