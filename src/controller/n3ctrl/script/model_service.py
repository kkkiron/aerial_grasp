#! /usr/bin/env python3
import os

# don't use gpu
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

import rospy
import actionlib
import numpy as np

from dynamic_reconfigure.client import Client as DynamicReconfigureClient

import n3ctrl.msg
from n3ctrl.srv import *
from std_msgs.msg import String

import numpy as np
# import matplotlib.pyplot as plt
# from scaledgp import ScaledGP
# from scipy import signal
# from progress.bar import Bar
import random

# from sklearn.gaussian_process import GaussianProcessRegressor
# from sklearn.gaussian_process.kernels import ConstantKernel, RBF
import time

from IGPR import IGPR

BASE_PATH = os.path.expanduser('~/Documents')

class ModelService(object):
	_train_result = n3ctrl.msg.TrainModelResult()

	def __init__(self, xdim, use_service = True):

		self.xdim=xdim
		self.verbose = True

		if use_service:
			# train action server
			self._action_service = actionlib.SimpleActionServer('train_model_service', n3ctrl.msg.TrainModelAction, execute_cb=self.train, auto_start = False)
			self._action_service.start()
			# add data service
			self._add_data_srv = rospy.Service('add_data_2_model', AddData2Model, self.add_data)
			# predict service
			self._predict_srv = rospy.Service('predict_model', PredictModel, self.predict)

			# self.pub = rospy.Publisher('model_train', String, queue_size = 10)

			# Create a dynamic reconfigure client
			# self.dyn_reconfig_client = DynamicReconfigureClient('controller_adaptiveclbf_reconfig', timeout=30, config_callback=self.reconfigure_cb)

			# self.N_data = rospy.get_param('/controller_adaptiveclbf/N_data',200)
			# self.verbose = rospy.get_param('/controller_adaptiveclbf/learning_verbose',False)


	def reconfigure_cb(self, config):
		self.N_data = config["N_data"]
		self.verbose = config["learning_verbose"]
		self.N_updates = config["N_updates"]

	def predict(self,req):
		# overload
		return None

	def train(self,goal):
		# overload
		return None

	def add_data(self,req):
		# overload
		return None


	def scale_data(self,x,xmean,xstd):
		if (xstd == 0).any():
			return (x-xmean)
		else:
			return (x - xmean) / xstd

	def unscale_data(self,x,xmean,xstd):
		if (xstd == 0).any():
			return x + xmean
		else:
			return x * xstd + xmean

'''
class ModelGPService(ModelService):
	def __init__(self,xdim,use_service=True):
		ModelService.__init__(self,xdim,use_service)
		model_xdim=self.xdim
		model_ydim=self.xdim//2

		self.m = ScaledGP(xdim=model_xdim,ydim=model_ydim)
		self.y = np.zeros((0,model_ydim))
		self.Z = np.zeros((0,model_xdim))
		self.N_data = 20

	def predict(self,req):
		# rospy.loginfo('predicting')
		if not hasattr(self, 'm'):
			resp = PredictModelResponse()
			resp.result = False
			return resp
		x = np.expand_dims(req.x, axis=0).T

		# format the input and use the model to make a prediction.
		y, var = self.m.predict(x.T)
		y_out = y.T

		resp = PredictModelResponse()
		resp.y_out = y_out.flatten()
		resp.var = var.T.flatten()
		resp.result = True
		return resp

	def train(self, goal=None):
		success = True
		
		if goal is not None:
			# goal was cancelled
			if self._action_service.is_preempt_requested():
				print("Prempt training request")
				self._action_service.set_preempted()
				success = False

		# train model.  this gets called by the training thread on timer_cb() in adaptive_clbf_node.
		if success and self.Z.shape[0] > 0 and self.Z.shape[0] == self.y.shape[0]:
			# self.m.optimize(self.Z,self.y)
			# self.pub.publish('1')
			if goal is not None:
				self._train_result.model_trained = True
				self._action_service.set_succeeded(self._train_result)
		else:
			if goal is not None:
				# self.pub.publish('0')
				self._train_result.model_trained = False
				self._action_service.set_succeeded(self._train_result)

	def add_data(self,req):
		# rospy.loginfo('adding data')
		if not hasattr(self, 'y'):
			return AddData2ModelResponse(False)

		x = np.expand_dims(req.x, axis=0).T

		# add a sample to the history of data
		ynew = np.expand_dims(req.ynew, axis=0).T
		self.Z = np.concatenate((self.Z,x.T))
		self.y = np.concatenate((self.y,ynew.T))

		# throw away old samples if too many samples collected.
		if self.y.shape[0] > self.N_data:
			self.y = self.y[-self.N_data:,:]
			self.Z = self.Z[-self.N_data:,:]
			# self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
			# self.Z = np.delete(self.Z,random.randint(0,self.N_data-1),axis=0)

		if self.verbose:
			print("ynew",ynew.T)
			# print("Znew",Znew)
			# print('x', x)
			print("n data:", self.y.shape[0])
			# print("prediction error:", self.predict_error)
			# print("predict var:", self.predict_var)

		return AddData2ModelResponse(True)
'''
class ModelIGPRService(ModelService):
	def __init__(self,xdim,use_service=True):
		ModelService.__init__(self,xdim,use_service)
		model_xdim=self.xdim
		model_ydim=self.xdim//2
		self.ydim = model_ydim
		self.max_k_matrix_size = 20

		self.m = []
		for i in range(self.ydim):
			self.m.append(IGPR(max_k_matrix_size=self.max_k_matrix_size))
		self.y = np.zeros((0,self.ydim))
		self.Z = np.zeros((0,self.xdim))
		self.N_data = self.max_k_matrix_size

	def predict(self,req):
		if not hasattr(self, 'm'):
			resp = PredictModelResponse()
			resp.result = False
			return resp
		x = np.expand_dims(req.x, axis=0).T

		# format the input and use the model to make a prediction.
		y = np.zeros((self.ydim, 1))
		var = np.zeros((self.ydim, 1))
		for i in range(self.ydim):
			yt, vart = self.m[i].predict(x.reshape((1,self.xdim)))
			y[i,0] = yt[0]
			var[i,0] = vart# np.sqrt(vart)
		# var = np.hstack((var, var, var))

		resp = PredictModelResponse()
		resp.y_out = y.flatten()
		resp.var = var.T.flatten()
		resp.result = True

		return resp

	def train(self, goal=None):
		success = True

		if goal is not None:
			# goal was cancelled
			if self._action_service.is_preempt_requested():
				print("Preempt training request")
				self._action_service.set_preempted()
				success = False

		# train model.  this gets called by the training thread on timer_cb() in adaptive_clbf_node.
		if success and self.Z.shape[0] > 0 and self.Z.shape[0] == self.y.shape[0]:
			# for i in range(self.ydim):
			# 	self.m[i].hyperparam_optimization()
			# self.pub.publish('1')
			if goal is not None:
				self._train_result.model_trained = True
				self._action_service.set_succeeded(self._train_result)
		else:
			if goal is not None:
				# self.pub.publish('0')
				self._train_result.model_trained = False
				self._action_service.set_succeeded(self._train_result)
	def add_data(self,req):
		if not hasattr(self, 'y'):
			return AddData2ModelResponse(False)

		x = np.expand_dims(req.x, axis=0).T

		# add a sample to the history of data
		ynew = np.expand_dims(req.ynew, axis=0).T
		self.Z = np.concatenate((self.Z,x.T))
		self.y = np.concatenate((self.y,ynew.T))
		for i in range(self.ydim):
			self.m[i].learn(x.reshape((self.xdim)), ynew[i,0])

		# throw away old samples if too many samples collected.
		if self.y.shape[0] > self.N_data:
			self.y = self.y[-self.N_data:,:]
			self.Z = self.Z[-self.N_data:,:]
			# self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
			# self.Z = np.delete(self.Z,random.randint(0,self.N_data-1),axis=0)

		if self.verbose:
			print("ynew",ynew)
			print("n data:", self.y.shape[0])
			# print("prediction error:", self.predict_error)
			# print("predict var:", self.predict_var)

		return AddData2ModelResponse(True)

class ModelIGPR2Service(ModelService):
	# update when train() instead of add_data()
	def __init__(self,xdim,use_service=True):
		ModelService.__init__(self,xdim,use_service)
		model_xdim=self.xdim
		model_ydim=self.xdim//2
		self.ydim = model_ydim
		self.max_k_matrix_size = 20
		self.data_counts_need_update = 0

		self.m = []
		for i in range(self.ydim):
			self.m.append(IGPR(max_k_matrix_size=self.max_k_matrix_size))
		self.y = np.zeros((0,self.ydim))
		self.Z = np.zeros((0,self.xdim))
		self.N_data = self.max_k_matrix_size

	def predict(self,req):
		if not hasattr(self, 'm'):
			resp = PredictModelResponse()
			resp.result = False
			return resp
		x = np.expand_dims(req.x, axis=0).T

		# format the input and use the model to make a prediction.
		y = np.zeros((self.ydim, 1))
		var = np.zeros((self.ydim, 1))
		for i in range(self.ydim):
			yt, vart = self.m[i].predict(x.reshape((1,self.xdim)))
			y[i,0] = yt[0]
			var[i,0] = vart# np.sqrt(vart)
		# var = np.hstack((var, var, var))

		resp = PredictModelResponse()
		resp.y_out = y.flatten()
		resp.var = var.T.flatten()
		resp.result = True

		return resp

	def train(self, goal=None):
		success = True

		if goal is not None:
			# goal was cancelled
			if self._action_service.is_preempt_requested():
				print("Preempt training request")
				self._action_service.set_preempted()
				success = False

		# train model.  this gets called by the training thread on timer_cb() in adaptive_clbf_node.
		if success and self.Z.shape[0] > 0 and self.Z.shape[0] == self.y.shape[0]:
			self.data_counts_need_update = min(self.data_counts_need_update, self.max_k_matrix_size)
			for i in range(self.y.shape[0] - self.data_counts_need_update, self.y.shape[0]):
				for j in range(self.ydim):
					self.m[j].learn(self.Z[i].reshape((self.xdim)), self.y[i,j])
			self.data_counts_need_update = 0

			# for i in range(self.ydim):
			# 	self.m[i].hyperparam_optimization()
			# self.pub.publish('1')
			if goal is not None:
				self._train_result.model_trained = True
				self._action_service.set_succeeded(self._train_result)
		else:
			if goal is not None:
				# self.pub.publish('0')
				self._train_result.model_trained = False
				self._action_service.set_succeeded(self._train_result)
	def add_data(self,req):
		if not hasattr(self, 'y'):
			return AddData2ModelResponse(False)

		x = np.expand_dims(req.x, axis=0).T

		# add a sample to the history of data
		ynew = np.expand_dims(req.ynew, axis=0).T
		self.Z = np.concatenate((self.Z,x.T))
		self.y = np.concatenate((self.y,ynew.T))

		self.data_counts_need_update += 1
		if self.y.shape[0] == 1:
			self.train()

		# for i in range(self.ydim):
		# 	self.m[i].learn(x.reshape((self.xdim)), ynew[i,0])

		# throw away old samples if too many samples collected.
		if self.y.shape[0] > self.N_data:
			self.y = self.y[-self.N_data:,:]
			self.Z = self.Z[-self.N_data:,:]
			# self.y = np.delete(self.y,random.randint(0,self.N_data-1),axis=0)
			# self.Z = np.delete(self.Z,random.randint(0,self.N_data-1),axis=0)

		if self.verbose:
			print("ynew",ynew)
			print("n data:", self.y.shape[0])
			# print("prediction error:", self.predict_error)
			# print("predict var:", self.predict_var)

		return AddData2ModelResponse(True)

if __name__ == '__main__':
    rospy.init_node('model_service')
    # server = ModelGPService(6, use_service = True) # TODO: put this in yaml or somewhere else
    server = ModelIGPRService(6, use_service = True) # TODO: put this in yaml or somewhere else
    # server = ModelIGPR2Service(6, use_service = True) # TODO: put this in yaml or somewhere else
    rospy.spin()
