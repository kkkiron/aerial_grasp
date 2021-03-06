#! /usr/bin/env python

import numpy as np
import copy
from adaptive_clbf import AdaptiveClbf
from dynamics import DynamicsQuadrotorModified
from progress.bar import Bar
import time
import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42


np.random.seed(0)
adaptive_clbf_mpc_trigger = AdaptiveClbf(use_service = False, use_mpc=True, use_trigger=True, use_IGPR=False)
clbf_mpc = AdaptiveClbf(use_service = False, use_mpc=True, use_trigger=False, use_IGPR=False)
adaptive_mpc = AdaptiveClbf(use_service = False, use_mpc=True, use_trigger=False, use_IGPR=False)
adaptive_clbf_trigger = AdaptiveClbf(use_service = False, use_mpc=False, use_trigger=True, use_IGPR=False)
adaptive_ad = AdaptiveClbf(use_service=False, use_mpc=False, use_trigger=False, use_IGPR=False)

params={}
params["a_lim"] = 13.93
params["thrust_lim"] = 25#0.5
params["kp_z"] = 1.0
params["kd_z"] = 1.0
params["clf_epsilon"] = 100.0


params["qp_u_cost"] = 100.0
params["qp_u_prev_cost"] = 1.0
# params["qp_p1_cost"] = 1.0
params["qp_p1_cost"] = 1.0e8
params["qp_p2_cost"] = 1.0e12
params["qp_max_var"] = 1.5
params["qp_verbose"] = False
params["max_velocity"] = 2.0
params["min_velocity"] = 0.5
params["barrier_vel_gamma"] = 10.0
params["use_barrier_vel"] = True
params["use_barrier_pointcloud"] = True
params["barrier_radius"] = 1
params["barrier_radius_velocity_scale"] = 0.0
params["barrier_pc_gamma_p"] = 5.0
params["barrier_pc_gamma"] = 0.08
params["verbose"] = False
params["dt"] = 0.1
params["max_error"] = 10.0

params['mass'] = 40.0/1000
params['g'] = 9.8
params['mpc_dt'] = 0.1

# alpaca params
# params["qp_ksig"] = 2.0e3
# params["measurement_noise"] = 1.0e-3

# gp params
params["qp_ksig"] = 1.0e5
params["measurement_noise"] = 1.0

# vanilla nn params
params["qp_ksig"] = 1.0e2
params["measurement_noise"] = 1.0

#600 60 500 600 with obstacles
params["N_data"] = 60#600
params["learning_verbose"] = False
params["N_updates"] = 50

params["mpc_stepsize"] = 1
params["mpc_N"] = 20

ref_traj_type = int(input("ref_traj(1~5):"))
wind_model = int(input("wind_model(1~10):"))

true_dyn = DynamicsQuadrotorModified(disturbance_scale_pos = 0.0, disturbance_scale_vel = -1.0, control_input_scale = 1.0)

clbf_mpc.update_params(params)
adaptive_mpc.update_params(params)
adaptive_clbf_mpc_trigger.update_params(params)
adaptive_clbf_trigger.update_params(params)
adaptive_ad.update_params(params)

clbf_mpc.true_dyn = true_dyn
adaptive_mpc.true_dyn = true_dyn
adaptive_clbf_mpc_trigger.true_dyn = true_dyn
adaptive_clbf_trigger.true_dyn = true_dyn
adaptive_ad.true_dyn = true_dyn

# T = 40
T = 40
dt = params["dt"]
N = int(round(T/dt))
t = np.linspace(0,T-2*dt,N-1)
xdim=6
udim=3

width = 1.0
speed = 1.0
freq = 1.0/10
if ref_traj_type == 1:
    x_d = np.stack((0.25 * t * np.cos(0.2 * np.pi * t), 0.25 * t * np.sin(0.2 * np.pi * t), 20 - 0.5 * t, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1))) # Conical Spiral
    traj_name = "Conical Spiral"
elif ref_traj_type == 2:
    x_d = np.stack((4 * np.cos(0.2 * t), 2 * np.sin(0.2 * 2 * t), 2 * np.sin(0.2 * 2 * t), np.zeros(N-1), np.zeros(N-1), np.zeros(N-1))) # Lemniscate 8
    traj_name = "Lemniscate"
elif ref_traj_type == 3:
    # x_d = np.stack((1*t, 2*t, t/(t+1), np.zeros(N-1), np.zeros(N-1), np.zeros(N-1))) # Parabola
    # traj_name = "Parabola"
    x_d = np.stack((1*t, 2*t, 1.5*t, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1))) # Line
    traj_name = "Line"
elif ref_traj_type == 4:
    x_d = np.stack((-2 * np.sin(0.5 * t), 5 + 2 * np.cos(0.5 * t), np.ones(N-1), np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))	# Circle
    traj_name = "Circle"
elif ref_traj_type == 5:
    x_d = np.stack((2 * np.sin(0.5 * t), 2 - 2 * np.cos(0.5 * t), 0.2 * t, np.zeros(N-1), np.zeros(N-1), np.zeros(N-1)))	# Cylindrical Helix
    traj_name = "Cylindrical Helix"

# x = [px, py, theta, v]
# x_d[2,:-1] = np.arctan2(np.diff(x_d[1,:]),np.diff(x_d[0,:]))
# x_d[3,:-1] = np.sqrt(np.diff(x_d[0,:])**2 + np.diff(x_d[1,:])**2)/dt
# x = [px, py, pz, vx, vy, vz]
x_d[3,:-1] = np.diff(x_d[0,:])
x_d[4,:-1] = np.diff(x_d[1,:])
x_d[5,:-1] = np.diff(x_d[2,:])
x_d[3,-1]=x_d[3,-2]
x_d[4,-1]=x_d[4,-2]
x_d[5,-1]=x_d[5,-2]


barrier_x = np.array([-1.5, -4, 2])
barrier_y = np.array([-1.39, 0, -1.7])
barrier_z = np.array([-1.39, 0, -1.7])
barrier_x = np.array([x_d[0, 100], x_d[0, 190], x_d[0, 280]])
barrier_y = np.array([x_d[1, 100], x_d[1, 190], x_d[1, 280]])
barrier_z = np.array([x_d[2, 100], x_d[2, 190], x_d[2, 280]])
barrier_r = np.array([1, 1, 1])
# barrier_x = np.array([-100])
# barrier_y = np.array([-100])
# barrier_z = np.array([-100])
# barrier_r = np.array([1])
# barrier_x = np.array([])
# barrier_y = np.array([])
# barrier_z = np.array([])
clbf_mpc.update_barrier_locations(barrier_x,barrier_y,barrier_z,barrier_r)
adaptive_mpc.update_barrier_locations(barrier_x,barrier_y,barrier_z,barrier_r)
adaptive_clbf_mpc_trigger.update_barrier_locations(barrier_x,barrier_y,barrier_z,barrier_r)
adaptive_clbf_trigger.update_barrier_locations(barrier_x,barrier_y,barrier_z,barrier_r)
adaptive_ad.update_barrier_locations(barrier_x,barrier_y,barrier_z,barrier_r)

x0=np.array([[0.0],[0.0],[0.0],[0.0001],[0.0001],[0.0001]])
z0 = true_dyn.convert_x_to_z(x0)


train_interval = 10#40
start_training = 100
# last_training = -1


x0 = x_d[:,0:1]
z0 = true_dyn.convert_x_to_z(x0)

u_1 = np.zeros((udim,N))
x_1 = np.zeros((xdim,N-1))
x_1[:,0:1] = x0
u_2 = np.zeros((udim,N))
x_2 = np.zeros((xdim,N-1))
x_2[:,0:1] = x0
u_3 = np.zeros((udim,N))
x_3 = np.zeros((xdim,N-1))
x_3[:,0:1] = x0
u_4 = np.zeros((udim,N))
x_4 = np.zeros((xdim,N-1))
x_4[:,0:1] = x0
u_ad = np.zeros((udim,N))
x_ad = np.zeros((xdim,N-1))
x_ad[:,0:1] = x0


z_d = np.zeros((xdim,N-1))
z_1 = np.zeros((xdim,N-1))
z_1[:,0:1] = z0
z_2 = np.zeros((xdim,N-1))
z_2[:,0:1] = z0
z_3 = np.zeros((xdim,N-1))
z_3[:,0:1] = z0
z_4 = np.zeros((xdim,N-1))
z_4[:,0:1] = z0
z_ad = np.zeros((xdim,N-1))
z_ad[:,0:1] = z0


z_d_dot = np.zeros((xdim,1))


prediction_error_1 = np.zeros(N)
prediction_error_true_1 = np.zeros(N)
prediction_var_1 = np.zeros((xdim//2,N))
prediction_error_2 = np.zeros(N)
prediction_error_true_2 = np.zeros(N)
prediction_var_2 = np.zeros((xdim//2,N))
prediction_error_3 = np.zeros(N)
prediction_error_true_3 = np.zeros(N)
prediction_var_3 = np.zeros((xdim//2,N))
prediction_error_4 = np.zeros(N)
prediction_error_true_4 = np.zeros(N)
prediction_var_4 = np.zeros((xdim//2,N))
prediction_error_ad = np.zeros(N)
prediction_error_true_ad = np.zeros(N)
prediction_var_ad = np.zeros((xdim//2,N))
# trGssGP_1 = np.zeros(N)
# trGssGP_2 = np.zeros(N)
# trGssGP_3 = np.zeros(N)
# trGssGP_4 = np.zeros(N)

control_time_log_1 = []
control_time_log_2 = []
control_time_log_3 = []
control_time_log_4 = []
control_time_log_ad = []

update_time_log_1 = []
update_time_log_2 = []
update_time_log_3 = []
update_time_log_4 = []
update_time_log_ad = []

trigger_iter_log_1 = []
trigger_iter_log_2 = []
trigger_iter_log_3 = []
trigger_iter_log_4 = []

constraint_value_log_1 = []
constraint_value_log_2 = []
constraint_value_log_3 = []
constraint_value_log_4 = []

i=0
z_d[:,i+1:i+2] = true_dyn.convert_x_to_z(x_d[:,i+1:i+2])
# for mpc
for i in range(N-3):
	z_d[:,i+2:i+3] = true_dyn.convert_x_to_z(x_d[:,i+2:i+3])

# adaptive_clbf_mpc_trigger.model.verbose = True
bar = Bar(max=N-1)
for i in range(N-2):

	bar.next()
	start0 = time.time()

	if i < N-3:
		z_d[:,i+2:i+3] = true_dyn.convert_x_to_z(x_d[:,i+2:i+3])
		z_d_dot = (z_d[:,i+2:i+3] - z_d[:,i+1:i+2])/dt

	if i == 0:
		add_data = False
	else:
		add_data = True

	start = time.time()
	u_1[:,i+1] = adaptive_clbf_mpc_trigger.get_control(z_1[:,i:i+1],z_d[:,i+1:i+2],z_d_dot,dt=dt,use_model=True,add_data=add_data,use_qp=True,ref_traj=z_d[:,i+1:i+21])
	control_time_log_1.append((time.time() - start)*1000)
	start = time.time()
	if (i - 1 == train_interval) or adaptive_clbf_mpc_trigger.qpsolve.triggered:
	# if True:
	# if (i - 1 == train_interval):
		# adaptive_clbf_mpc_trigger.model.update()
		trigger_iter_log_1.append(i)
		adaptive_clbf_mpc_trigger.model.train()
		adaptive_clbf_mpc_trigger.model_trained = True
	update_time_log_1.append((time.time() - start)*1000)
	constraint_value_log_1.append(adaptive_clbf_mpc_trigger.qpsolve.constraint_value)
	prediction_error_1[i] = adaptive_clbf_mpc_trigger.predict_error
	prediction_error_true_1[i] = adaptive_clbf_mpc_trigger.true_predict_error
	prediction_var_1[:,i:i+1] = np.clip(adaptive_clbf_mpc_trigger.predict_var,0,params["qp_max_var"])
	# trGssGP_mpc_trigger[i] = adaptive_clbf_mpc_trigger.qpsolve.trGssGP
	'''
	start = time.time()
	u_2[:,i+1] = clbf_mpc.get_control(z_2[:,i:i+1],z_d[:,i+1:i+2],z_d_dot,dt=dt,use_model=False,add_data=False,use_qp=True,ref_traj=z_d[:,i+1:i+21])
	control_time_log_2.append((time.time() - start)*1000)
	start = time.time()
	constraint_value_log_2.append(clbf_mpc.qpsolve.constraint_value)
	update_time_log_2.append((time.time() - start)*1000)

	start = time.time()
	u_3[:,i+1] = adaptive_mpc.get_control(z_3[:,i:i+1],z_d[:,i+1:i+2],z_d_dot,dt=dt,use_model=True,add_data=add_data,use_qp=False,ref_traj=z_d[:,i+1:i+21])
	control_time_log_3.append((time.time() - start)*1000)
	start = time.time()
	# if (i - start_training -1 ) % train_interval == 0 and i > start_training:
	if (i - start_training -1 ) % train_interval == 0:
	# if True:
		# adaptive_clbf_mpc.model.update()
		adaptive_mpc.model.train()
		adaptive_mpc.model_trained = True
	update_time_log_3.append((time.time() - start)*1000)
	prediction_error_3[i] = adaptive_mpc.predict_error
	prediction_error_true_3[i] = adaptive_mpc.true_predict_error
	prediction_var_3[:,i:i+1] = np.clip(adaptive_mpc.predict_var,0,params["qp_max_var"])
	# trGssGP_mpc[i] = adaptive_clbf_mpc.qpsolve.trGssGP

	start = time.time()
	u_4[:,i+1] = adaptive_clbf_trigger.get_control(z_4[:,i:i+1],z_d[:,i+1:i+2],z_d_dot,dt=dt,use_model=True,add_data=add_data,use_qp=True,ref_traj=z_d[:,i:])
	control_time_log_4.append((time.time() - start)*1000)
	start = time.time()
	if (i - 1 == train_interval) or adaptive_clbf_trigger.qpsolve.triggered:
	# if True:
	# if (i - 1 == train_interval):
		# adaptive_clbf_trigger.model.update()
		trigger_iter_log_4.append(i)
		adaptive_clbf_trigger.model.train()
		adaptive_clbf_trigger.model_trained = True
	update_time_log_4.append((time.time() - start)*1000)
	constraint_value_log_4.append(adaptive_clbf_trigger.qpsolve.constraint_value)
	prediction_error_4[i] = adaptive_clbf_trigger.predict_error
	prediction_error_true_4[i] = adaptive_clbf_trigger.true_predict_error
	prediction_var_4[:,i:i+1] = np.clip(adaptive_clbf_trigger.predict_var,0,params["qp_max_var"])

	start = time.time()
	u_ad[:,i+1] = adaptive_ad.get_control(z_ad[:,i:i+1],z_d[:,i+1:i+2],z_d_dot,dt=dt,use_model=True,add_data=add_data,use_qp=False)
	control_time_log_ad.append((time.time() - start)*1000)
	start = time.time()
	if (i - start_training - 1) % train_interval == 0 and i > start_training:
	# if i - 1 == train_interval:
	# if True:
		adaptive_ad.model.train()
		adaptive_ad.model_trained = True
	update_time_log_ad.append((time.time() - start)*1000)
	prediction_error_ad[i] = adaptive_ad.predict_error
	prediction_error_true_ad[i] = adaptive_ad.true_predict_error
	prediction_var_ad[:,i:i+1] = np.clip(adaptive_ad.predict_var,0,params["qp_max_var"])
    '''
	# dt = np.random.uniform(0.05,0.15)
	c_1 = copy.copy(u_1[:,i+1:i+2])
	c_2 = copy.copy(u_2[:,i+1:i+2])
	c_3 = copy.copy(u_3[:,i+1:i+2])
	c_4 = copy.copy(u_4[:,i+1:i+2])
	c_ad = copy.copy(u_ad[:,i+1:i+2])
	
	z_1[:,i+1:i+2] = true_dyn.step(z_1[:,i:i+1],c_1,dt)
	z_2[:,i+1:i+2] = true_dyn.step(z_2[:,i:i+1],c_2,dt)
	z_3[:,i+1:i+2] = true_dyn.step(z_3[:,i:i+1],c_3,dt)
	z_4[:,i+1:i+2] = true_dyn.step(z_4[:,i:i+1],c_4,dt)
	z_ad[:,i+1:i+2] = true_dyn.step(z_ad[:,i:i+1],c_ad,dt)

	x_1[:,i+1:i+2] = true_dyn.convert_z_to_x(z_1[:,i+1:i+2])
	x_2[:,i+1:i+2] = true_dyn.convert_z_to_x(z_2[:,i+1:i+2])
	x_3[:,i+1:i+2] = true_dyn.convert_z_to_x(z_3[:,i+1:i+2])
	x_4[:,i+1:i+2] = true_dyn.convert_z_to_x(z_4[:,i+1:i+2])
	x_ad[:,i+1:i+2] = true_dyn.convert_z_to_x(z_ad[:,i+1:i+2])
	
	print('Iteration ', i, ', Time elapsed (ms): ', (time.time() - start0)*1000)
	# print('control', u_1[:,i+1].T)
	# print('x', x_1[:,i+1].T)


path_to_save = '/home/wuzhixuan/pro/test2_ws/data/'

'''
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
plt.semilogy(t[:-1],prediction_var_1[0,:-2],'g-',alpha=0.9)
plt.semilogy(t[:-1],prediction_var_1[1,:-2],'g:',alpha=0.9)
plt.semilogy(t[:-1],prediction_var_1[2,:-2],'g--',alpha=0.9)
# plt.semilogy(t[:-1],prediction_var_3[0,:-2],'b--',alpha=.9)
# plt.semilogy(t[:-1],prediction_var_3[1,:-2],'b:',alpha=.9)
# plt.semilogy(t[:-1],prediction_var_4[0,:-2],'m--',alpha=.9)
# plt.semilogy(t[:-1],prediction_var_4[1,:-2],'m:',alpha=.9)
# plt.semilogy(t[:-1],prediction_var_ad[0,:-2],'m--',alpha=0.9)
# plt.semilogy(t[:-1],prediction_var_ad[1,:-2],'m:',alpha=0.9)
plt.xlabel("Time(s)")
plt.ylabel(r"$\sigma_{\bar{\Delta}}(x,\mu)$")
# plt.legend(['all[1]','all[2]','without_qp[1]','without_qp[2]','without_mpc[1]','without_mpc[2]'],bbox_to_anchor=(0,1.2,1,0.2), loc="upper center", ncol=2)
plt.legend(['all[1]','all[2]','all[3]'],bbox_to_anchor=(0,1.2,1,0.2), loc="upper center", ncol=2)
plt.plot([t[0],t[-1]],[params["measurement_noise"],params["measurement_noise"]],'r--')
plt.savefig(path_to_save+'1.1.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'1.1.png', dpi=600, format='png',bbox_inches='tight')
# plt.subplot(312)
# plt.plot(t[:-1],trGssGP[:-2],'g--',alpha=0.9)
# plt.ylabel("trace(GssGP)")
# plt.xlabel("Time(s)")
plt.figure()
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams.update({'font.size': 12})
# plt.plot(t[:-1],prediction_error_1[:-2],'g-',alpha=0.9, label='all')
plt.plot(t[:-1],prediction_error_true_1[:-2],'g-',alpha=0.9, label='clbfet')
# plt.plot(t[:-1],prediction_error_3[:-2],'b--',alpha=0.9)
# plt.plot(t[:-1],prediction_error_true_3[:-2],'b:',alpha=0.9)
# plt.plot(t[:-1],prediction_error_4[:-2],'m--',alpha=0.9)
# plt.plot(t[:-1],prediction_error_true_4[:-2],'m:',alpha=0.9)
# plt.plot(t[:-1],prediction_error_ad[:-2],'m--',alpha=0.9)
# plt.plot(t[:-1],prediction_error_true_ad[:-2],'m:',alpha=0.9)
# plt.ylim([0,1.0])
plt.ylabel("Prediction error")
plt.xlabel("Time(s)")
# plt.legend(['all[1]','all[2]','without_qp[1]','without_qp[2]','without_mpc[1]','without_mpc[2]'],bbox_to_anchor=(0,1.2,1,0.2), loc="upper center", ncol=2)
plt.legend(bbox_to_anchor=(0,1.2,1,0.2), loc="upper center", ncol=1)
plt.savefig(path_to_save+'1.2.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'1.2.png', dpi=600, format='png',bbox_inches='tight')
'''
np.savetxt(path_to_save+'sim_predict_error_log_1.txt', prediction_error_1)
np.savetxt(path_to_save+'sim_predict_var_log_1.txt', prediction_var_1)
np.savetxt(path_to_save+'sim_predict_error_log_2.txt', prediction_error_2)
np.savetxt(path_to_save+'sim_predict_var_log_2.txt', prediction_var_2)
np.savetxt(path_to_save+'sim_predict_error_log_3.txt', prediction_error_3)
np.savetxt(path_to_save+'sim_predict_var_log_3.txt', prediction_var_3)
np.savetxt(path_to_save+'sim_predict_error_log_4.txt', prediction_error_4)
np.savetxt(path_to_save+'sim_predict_var_log_4.txt', prediction_var_4)


fig = plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False

ax = plt.axes(projection='3d')
ax.plot3D(x_1[0,:], x_1[1,:], x_1[2,:], 'g-',alpha=0.9,label='clbfet')
# ax.plot3D(x_2[0,:], x_2[1,:], x_2[2,:], 'c-',alpha=0.9,label='FBLC-QP-MPC')
# ax.plot3D(x_3[0,:], x_3[1,:], x_3[2,:], 'b-',alpha=0.9,label='LB-FBLC-MPC')
# ax.plot3D(x_4[0,:], x_4[1,:], x_4[2,:], 'm-',alpha=0.9,label='LB-FBLC-QP')
ax.plot3D(x_d[0,:], x_d[1,:], x_d[2,:], 'k-',label='ref')
ax.scatter3D(barrier_x[:], barrier_y[:], barrier_z[:], c='r', s=2000)

# to = np.linspace(0, np.pi * 2, 100)
# so = np.linspace(0, np.pi, 100)
# to, so = np.meshgrid(to, so)
# xo = np.cos(to) * np.sin(so)
# yo = np.sin(to) * np.sin(so)
# zo = np.cos(so)
# ax.plot_surface(xo, yo, zo, rstride=1, cstride=1, cmap='r')

# plt.plot(x_1[0,:],x_1[1,:],'g-',alpha=0.9,label='LB-FBLC-QP-MPC')
# plt.plot(x_2[0,:],x_2[1,:],'c-',alpha=0.9,label='FBLC-QP-MPC')
# plt.plot(x_3[0,:],x_3[1,:],'b-',alpha=0.9,label='LB-FBLC-MPC')
# plt.plot(x_4[0,:],x_4[1,:],'m-',alpha=0.9,label='LB-FBLC-QP')
# # plt.plot(x_ad[0,:],x_ad[1,:],'m--',alpha=0.9,label='ad')
# plt.plot(x_d[0,:],x_d[1,:],'k-',label='ref')
plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower center", ncol=3)
ax = fig.gca()
# for i in range(barrier_x.size):
# 	circle = plt.Circle((barrier_x[i],barrier_y[i]), params["barrier_radius"], color='r')
# 	ax.add_artist(circle)
# plt.xlabel('X Position')
# plt.ylabel('Y Position')
plt.savefig(path_to_save+'2.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'2.png', dpi=600, format='png',bbox_inches='tight')

np.savetxt(path_to_save+'sim_x_log_1.txt', x_1)
np.savetxt(path_to_save+'sim_x_log_2.txt', x_2)
np.savetxt(path_to_save+'sim_x_log_3.txt', x_3)
np.savetxt(path_to_save+'sim_x_log_4.txt', x_4)
np.savetxt(path_to_save+'sim_x_log_ref.txt', x_d)

'''
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
# plt.subplot(211)
plt.plot(t,u_1[0,:-1],'g-',alpha=0.9)
plt.plot(t,u_2[0,:-1],'c-',alpha=0.9)
plt.plot(t,u_3[0,:-1],'b-',alpha=0.9)
plt.plot(t,u_4[0,:-1],'m-',alpha=0.9)
# plt.plot(t,u_ad[0,:-1],'m--',alpha=0.9)
plt.legend(['clbfet','FBLC-QP-MPC','LB-FBLC-MPC','LB-FBLC-QP'],bbox_to_anchor=(0,1.1,1,0.2), loc="upper center", ncol=4)
plt.plot([t[0],t[-1]],[params["a_lim"],params["a_lim"]],'r--')
plt.plot([t[0],t[-1]],[-params["a_lim"],-params["a_lim"]],'r--')
plt.ylabel('acc_x')
plt.xlabel('Time (s)')
plt.savefig(path_to_save+'3.1.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'3.1.png', dpi=600, format='png',bbox_inches='tight')
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
plt.plot(t,u_1[1,:-1],'g-',alpha=0.9)
plt.plot(t,u_2[1,:-1],'c-',alpha=0.9)
plt.plot(t,u_3[1,:-1],'b-',alpha=0.9)
plt.plot(t,u_4[1,:-1],'m-',alpha=0.9)
# plt.plot(t,u_ad[1,:-1],'m--',alpha=0.9)
plt.legend(['clbfet','FBLC-QP-MPC','LB-FBLC-MPC','LB-FBLC-QP'],bbox_to_anchor=(0,1.1,1,0.2), loc="upper center", ncol=4)
plt.plot([t[0],t[-1]],[params["a_lim"],params["a_lim"]],'r--')
plt.plot([t[0],t[-1]],[-params["a_lim"],-params["a_lim"]],'r--')
plt.ylabel('acc_y (m/s^2)')
plt.xlabel('Time (s)')
plt.savefig(path_to_save+'3.2.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'3.2.png', dpi=600, format='png',bbox_inches='tight')
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
plt.plot(t,u_1[2,:-1],'g-',alpha=0.9)
plt.plot(t,u_2[2,:-1],'c-',alpha=0.9)
plt.plot(t,u_3[2,:-1],'b-',alpha=0.9)
plt.plot(t,u_4[2,:-1],'m-',alpha=0.9)
# plt.plot(t,u_ad[1,:-1],'m--',alpha=0.9)
plt.legend(['clbfet','FBLC-QP-MPC','LB-FBLC-MPC','LB-FBLC-QP'],bbox_to_anchor=(0,1.1,1,0.2), loc="upper center", ncol=4)
plt.plot([t[0],t[-1]],[params["a_lim"],params["a_lim"]],'r--')
plt.plot([t[0],t[-1]],[-params["a_lim"],-params["a_lim"]],'r--')
plt.ylabel('acc_z (m/s^2)')
plt.xlabel('Time (s)')
plt.savefig(path_to_save+'3.3.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'3.3.png', dpi=600, format='png',bbox_inches='tight')
'''
np.savetxt(path_to_save+'sim_acc_log_1.txt', u_1)
np.savetxt(path_to_save+'sim_acc_log_2.txt', u_2)
np.savetxt(path_to_save+'sim_acc_log_3.txt', u_3)
np.savetxt(path_to_save+'sim_acc_log_4.txt', u_4)


barrier_dist_1 = np.min(np.stack([np.sqrt((barrier_x[i]-x_1[0,:])**2 + (barrier_y[i]-x_1[1,:])**2 + (barrier_z[i]-x_1[2,:])**2) for i in range(len(barrier_x))]),axis=0)
barrier_dist_2 = np.min(np.stack([np.sqrt((barrier_x[i]-x_2[0,:])**2 + (barrier_y[i]-x_2[1,:])**2 + (barrier_z[i]-x_2[2,:])**2) for i in range(len(barrier_x))]),axis=0)
barrier_dist_3 = np.min(np.stack([np.sqrt((barrier_x[i]-x_3[0,:])**2 + (barrier_y[i]-x_3[1,:])**2 + (barrier_z[i]-x_3[2,:])**2) for i in range(len(barrier_x))]),axis=0)
barrier_dist_4 = np.min(np.stack([np.sqrt((barrier_x[i]-x_4[0,:])**2 + (barrier_y[i]-x_4[1,:])**2 + (barrier_z[i]-x_4[2,:])**2) for i in range(len(barrier_x))]),axis=0)
barrier_dist_ad = np.min(np.stack([np.sqrt((barrier_x[i]-x_ad[0,:])**2 + (barrier_y[i]-x_ad[1,:])**2 + (barrier_z[i]-x_ad[2,:])**2) for i in range(len(barrier_x))]),axis=0)
np.savetxt(path_to_save+'sim_barrier_dist_log_1.txt', barrier_dist_1)
np.savetxt(path_to_save+'sim_barrier_dist_log_2.txt', barrier_dist_2)
np.savetxt(path_to_save+'sim_barrier_dist_log_3.txt', barrier_dist_3)
np.savetxt(path_to_save+'sim_barrier_dist_log_4.txt', barrier_dist_4)
'''
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
plt.plot(t,np.sqrt((x_d[0,:]-x_1[0,:])**2 + (x_d[1,:]-x_1[1,:])**2 + (x_d[2,:]-x_1[2,:])**2),'g-',alpha=0.9)
plt.plot(t,np.sqrt((x_d[0,:]-x_2[0,:])**2 + (x_d[1,:]-x_2[1,:])**2 + (x_d[2,:]-x_2[2,:])**2),'c-',alpha=0.9)
plt.plot(t,np.sqrt((x_d[0,:]-x_3[0,:])**2 + (x_d[1,:]-x_3[1,:])**2 + (x_d[2,:]-x_3[2,:])**2),'b-',alpha=0.9)
plt.plot(t,np.sqrt((x_d[0,:]-x_4[0,:])**2 + (x_d[1,:]-x_4[1,:])**2 + (x_d[2,:]-x_4[2,:])**2),'m-',alpha=0.9)
# plt.plot(t,np.sqrt((x_d[0,:]-x_ad[0,:])**2 + (x_d[1,:]-x_ad[1,:])**2),'m--',alpha=0.9)
plt.plot([0],[0],'r--')
plt.ylabel("Tracking error (m)")
plt.xlabel("Time(s)")
# plt.legend(['LB-FBLC-QP-MPC','FBLC-QP-MPC','LB-FBLC-MPC','LB-FBLC-QP'],bbox_to_anchor=(0,1.2,1,0.2), loc="upper right", ncol=2)
plt.legend(['clbfet','FBLC-QP-MPC','LB-FBLC-MPC','LB-FBLC-QP'], loc="upper right", ncol=2)
# plt.ylim([0,1.0])
plt.savefig(path_to_save+'4.1.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'4.1.png', dpi=600, format='png',bbox_inches='tight')
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
plt.plot(t,x_d[3,:],'k-')
plt.plot(t,x_1[3,:],'g-',alpha=0.9)
plt.plot(t,x_2[3,:],'c-',alpha=0.9)
plt.plot(t,x_3[3,:],'b-',alpha=0.9)
plt.plot(t,x_4[3,:],'m-',alpha=0.9)
# plt.plot(t,x_ad[3,:],'m--',alpha=0.9)
plt.ylabel('Vel (m/s)')
plt.plot([t[0],t[-1]],[params["max_velocity"],params["max_velocity"]],'r--')
plt.plot([t[0],t[-1]],[params["min_velocity"],params["min_velocity"]],'r--')
plt.legend(['ref','clbfet','FBLC-QP-MPC','LB-FBLC-MPC','LB-FBLC-QP','barrier'],bbox_to_anchor=(0,1.2,1,0.2), loc="upper center", ncol=3)
plt.xlabel('Time (s)')
plt.savefig(path_to_save+'4.2.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'4.2.png', dpi=600, format='png',bbox_inches='tight')
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
plt.plot(t,barrier_dist_1-params["barrier_radius"],'g-',alpha=0.9)
plt.plot(t,barrier_dist_2-params["barrier_radius"],'c-',alpha=0.9)
plt.plot(t,barrier_dist_3-params["barrier_radius"],'b-',alpha=0.9)
plt.plot(t,barrier_dist_4-params["barrier_radius"],'m-',alpha=0.9)
# plt.plot(t,barrier_dist_ad-params["barrier_radius"],'m--',alpha=0.9)
plt.plot([t[0],t[-1]],[0,0],'r--')
plt.ylabel("Distance to obstacles (m)")
plt.xlabel("Time(s)")
# plt.legend(['LB-FBLC-QP-MPC','FBLC-QP-MPC','LB-FBLC-MPC','LB-FBLC-QP','barrier'],bbox_to_anchor=(0,1.2,1,0.2), loc="center right", ncol=2)
plt.legend(['clbfet','FBLC-QP-MPC','LB-FBLC-MPC','LB-FBLC-QP'], loc="upper right", ncol=2)
plt.savefig(path_to_save+'4.3.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'4.3.png', dpi=600, format='png',bbox_inches='tight')
'''

np.savetxt(path_to_save+'sim_constraint_value_log_1.txt', constraint_value_log_1)
'''
constraint_value_log_1 = np.clip(constraint_value_log_1, -1e10, 1e-4)
# constraint_value_log_2 = np.clip(constraint_value_log_2, -1e10, 1e-4)
# constraint_value_log_4 = np.clip(constraint_value_log_4, -1e10, 1e-4)
plt.figure()
plt.rcParams.update({'font.size': 12})
plt.rcParams['axes.unicode_minus'] = False
plt.plot(t[40:-1],constraint_value_log_1[40:],'g-',alpha=.9)
plt.plot([t[0],t[-1]],[-5e-4, -5e-4],'r--',alpha=.9)
plt.ylabel(r'$\lambda$')
plt.xlabel("Time(s)")
plt.savefig(path_to_save+'5.1.eps', dpi=600, format='eps',bbox_inches='tight')
plt.savefig(path_to_save+'5.1.png', dpi=600, format='png',bbox_inches='tight')
# plt.figure()
# plt.rcParams.update({'font.size': 12})
# plt.rcParams['axes.unicode_minus'] = False
# plt.plot(t[40:-1],constraint_value_log_2[40:],'b-',alpha=.9)
# plt.plot([t[0],t[-1]],[-5e-4, -5e-4],'r--',alpha=.9)
# plt.ylabel(r'$\lambda$')
# plt.xlabel("Time(s)")
# plt.savefig('5.2.eps', dpi=600, format='eps',bbox_inches='tight')
# plt.savefig('5.2.png', dpi=600, format='png',bbox_inches='tight')
# plt.figure()
# plt.rcParams.update({'font.size': 12})
# plt.rcParams['axes.unicode_minus'] = False
# plt.plot(t[40:-1],constraint_value_log_4[40:],'m-',alpha=.9)
# plt.plot([t[0],t[-1]],[-5e-4, -5e-4],'r--',alpha=.9)
# plt.ylabel(r'$\lambda$')
# plt.xlabel("Time(s)")
# plt.savefig('5.3.eps', dpi=600, format='eps',bbox_inches='tight')
# plt.savefig('5.3.png', dpi=600, format='png',bbox_inches='tight')
'''

output = 'clbfet\n'
output += 'mean control time: ' + str(sum(control_time_log_1) / len(control_time_log_1)) + '\n'
output += 'mean update  time: ' + str(sum(update_time_log_1) / len(update_time_log_1)) + '\n'
output += 'mean position err: ' + str(np.mean(np.sqrt((x_d[0,:]-x_1[0,:])**2 + (x_d[1,:]-x_1[1,:])**2 + (x_d[2,:]-x_1[2,:])**2))) + '\n'
output += 'average obsta dis: ' + str(np.sum(barrier_dist_1) / barrier_dist_1.shape[0]) + '\n'
output += 'minimum obsta dis: ' + str(np.min(barrier_dist_1)) + '\n'
output += 'trigger    counts: ' + str(len(trigger_iter_log_1)) + '\n'

with open(path_to_save+'output1.txt', 'w') as f:
    f.write(output)
print(output)

'''
print('1')
print('control time: ', sum(control_time_log_1))
print('update time: ', sum(update_time_log_1))
print('mean control time: ', sum(control_time_log_1) / len(control_time_log_1))
print('mean update  time: ', sum(update_time_log_1) / len(update_time_log_1))
print('average obsta dis: ', np.sum(barrier_dist_1) / barrier_dist_1.shape[0])
print('minimum obsta dis: ', np.min(barrier_dist_1))
print("mean prediction error: ", np.mean(prediction_error_1[start_training+2:]))
print("mean true error: ", np.mean(prediction_error_true_1[start_training+2:]))
print("mean position error: ", np.mean(np.sqrt((x_d[0,:]-x_1[0,:])**2 + (x_d[1,:]-x_1[1,:])**2)))
print("trigger counts: ", len(trigger_iter_log_1))
# for i in trigger_iter_log_1:
# 	print i,' '
print('')

print('2')
print('control time: ', sum(control_time_log_2))
print('mean control time: ', sum(control_time_log_2) / len(control_time_log_2))
print('update time: ', sum(update_time_log_2))
print('mean update time: ', sum(update_time_log_2) / len(update_time_log_2))
print("mean prediction error: ", np.mean(prediction_error_2[start_training+2:]))
print("mean true error: ", np.mean(prediction_error_true_2[start_training+2:]))
print("mean position error: ", np.mean(np.sqrt((x_d[0,:]-x_2[0,:])**2 + (x_d[1,:]-x_2[1,:])**2)))
print('')

print('3')
print('control time: ', sum(control_time_log_3))
print('mean control time: ', sum(control_time_log_3) / len(control_time_log_3))
print('update time: ', sum(update_time_log_3))
print('mean update time: ', sum(update_time_log_3) / len(update_time_log_3))
print("mean prediction error: ", np.mean(prediction_error_3[start_training+2:]))
print("mean true error: ", np.mean(prediction_error_true_3[start_training+2:]))
print("mean position error: ", np.mean(np.sqrt((x_d[0,:]-x_3[0,:])**2 + (x_d[1,:]-x_3[1,:])**2)))
print('')

print('4')
print('control time: ', sum(control_time_log_4))
print('mean control time: ', sum(control_time_log_4) / len(control_time_log_4))
print('update time: ', sum(update_time_log_4))
print('mean update time: ', sum(update_time_log_4) / len(update_time_log_4))
print("mean prediction error: ", np.mean(prediction_error_4[start_training+2:]))
print("mean true error: ", np.mean(prediction_error_true_4[start_training+2:]))
print("mean position error: ", np.mean(np.sqrt((x_d[0,:]-x_4[0,:])**2 + (x_d[1,:]-x_4[1,:])**2)))
print("trigger counts: ", len(trigger_iter_log_4))
# for i in trigger_iter_log_4:
# 	print i,' '
print('')

print('ad')
print('control time: ', sum(control_time_log_ad))
print('mean control time: ', sum(control_time_log_ad) / len(control_time_log_ad))
print('update time: ', sum(update_time_log_ad))
print('mean update time: ', sum(update_time_log_ad) / len(update_time_log_ad))
print("mean prediction error: ", np.mean(prediction_error_ad[start_training+2:]))
print("mean true error: ", np.mean(prediction_error_true_ad[start_training+2:]))
print("mean position error: ", np.mean(np.sqrt((x_d[0,:]-x_ad[0,:])**2 + (x_d[1,:]-x_ad[1,:])**2)))
'''
# plt.show()
