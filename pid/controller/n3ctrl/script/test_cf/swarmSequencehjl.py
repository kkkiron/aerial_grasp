#!/usr/bin/env python3

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017-2018 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

import math
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie import HighLevelCommander
from NatNetClient import NatNetClient
import threading

pos_real = {}
agent_1_id = 1
# agent_2_id = 2

# agent_3_id = 3
# agent_4_id = 4

# agent_5_id = 5
# agent_6_id = 6

# agent_7_id = 7
# agent_8_id = 8

# agent_9_id = 9
# agent_10_id = 10

# Change uris and sequences according to your setup
# URI1 = 'radio://0/10/2M/E7E7E7E140'
URI1 = 'radio://0/60/2M/E7E7E7E060'
URI2 = 'radio://0/10/2M/E7E7E7E030'
URI3 = 'radio://0/60/2M/E7E7E7E060'
URI4 = 'radio://0/60/2M/E7E7E7E090'



#URI4 = 'radio://1/120/2M/E7E7E7E740'

#URI5 = 'radio://1/120/2M/E7E7E7E750'
#URI6 = 'radio://1/120/2M/E7E7E7E760'

#URI7 = 'radio://2/100/2M/E7E7E7E770'
#URI8 = 'radio://2/100/2M/E7E7E7E780'

#URI9 =  'radio://3/100/2M/E7E7E7E790'
#URI10 = 'radio://3/100/2M/E7E7E7E70A'




class OptiTrack:
    def __init__(self):
        print('optitrack')
        # optitrack stuff
        self.l_odom = list()
        self.l_index = -1
        self.sampleInterval = 5
        self.s1 = threading.Semaphore(1)
        self.agent_1_id = agent_1_id
        # self.agent_2_id = agent_2_id
        # self.agent_3_id = agent_3_id
        # self.agent_4_id = agent_4_id
        #self.agent_5_id = agent_5_id
        #self.agent_6_id = agent_6_id
        #self.agent_7_id = agent_7_id
        #self.agent_8_id = agent_8_id
        #self.agent_9_id = agent_9_id
        #self.agent_10_id = agent_10_id
        self.position = {}

        self.streamingClient = NatNetClient("172.16.5.205")  # Net
        self.streamingClient.rigidBodyListListener = self.rigidBodyListListener
        self.streamingClient.run()

    def rigidBodyListListener(self, rigidBodyList, timestamp ):
        # print('rigidBodyList = ',rigidBodyList)
        for rigidBody in rigidBodyList:
            id = rigidBody[0]
            if id == self.agent_1_id:
                pos_real[self.agent_1_id] = rigidBody[1]
            # elif id == self.agent_2_id:
            # 	pos_real[self.agent_2_id] = rigidBody[1]
            # elif id == self.agent_3_id:
            # 	pos_real[self.agent_3_id] = rigidBody[1]
            # elif id == self.agent_4_id:
            #     pos_real[self.agent_4_id] = rigidBody[1]
            # elif id == self.agent_5_id:
            #     pos_real[self.agent_5_id] = rigidBody[1]
            # elif id == self.agent_6_id:
            #     pos_real[self.agent_6_id] = rigidBody[1]
            # elif id == self.agent_7_id:
            #     pos_real[self.agent_7_id] = rigidBody[1]
            # elif id == self.agent_8_id:
            #     pos_real[self.agent_8_id] = rigidBody[1]
            # elif id == self.agent_9_id:
            #     pos_real[self.agent_9_id] = rigidBody[1]
            # elif id == self.agent_10_id:
            #     pos_real[self.agent_10_id] = rigidBody[1]

            # print('position = ', rigidBody[1])



optitrack = OptiTrack()
yaw_pref_in_deg = 90
step_in_path = 0.1
time.sleep(2)




# 
all_agent_path = [list() for _ in range(10)]
# all_agent_id = [agent_1_id, agent_2_id, agent_3_id, agent_4_id, agent_5_id, \
#                 agent_6_id, agent_7_id, agent_8_id, agent_9_id, agent_10_id]
all_agent_id = [agent_1_id]
# all_agent_id = [agent_1_id, agent_2_id, agent_3_id, agent_4_id]
all_sequence = [list() for _ in range(10)]
print('all_agent_path = ', all_agent_path)
with open('data_orca417.txt',"r") as f:
    pass
    line = f.readline()
    # agent_num = int(len(line.split())/4)
    agent_num = 1
    print('agent_num = ',agent_num)
    count = 0
    st = 3
    for line in f:
        if count % st ==0:
            line_data = line.split()
            for i in range(agent_num):
                agent_xyzyaw = line_data[4*i:4*i+4]
                x = float(agent_xyzyaw[0])
                y = float(agent_xyzyaw[1])
                z = float(agent_xyzyaw[2])
                yaw = float(agent_xyzyaw[3])
                point = (x,y,z,yaw)
                all_agent_path[i].append(point)
                # print('agent_xyzyaw = ',(x,y,z,yaw))
        count = count + 1
    for i in range(agent_num):
        agent_id = all_agent_id[i]
        sequence = [agent_id]
        for point in all_agent_path[i]:
            x = point[0]
            y = point[1]
            z = point[2]
            step = step_in_path
            data = (x, y, z, step)
            sequence.append(data)
        # print('sequence = ', sequence)
        print('sequence length = ', len(sequence))
        # print('\n')
        all_sequence[i].append(sequence)


#    x   y   z  time
step = 3
delta_x = 0.0
delta_y = -0.5
sequence1 = all_sequence[1-1][0]
# sequence2 = all_sequence[2-1][0]
# sequence3 = all_sequence[3-1][0]
# sequence4 = all_sequence[4-1][0]
# sequence5 = all_sequence[5-1][0]
# sequence6 = all_sequence[6-1][0]
# sequence7 = all_sequence[7-1][0]
# sequence8 = all_sequence[8-1][0]
# sequence9 = all_sequence[9-1][0]
# sequence10 = all_sequence[10-1][0]


seq_args = {
    URI1: [sequence1],
    # URI2: [sequence2],
    # URI3: [sequence3],
    # URI4: [sequence4],
    #URI5: [sequence5],
    #URI6: [sequence6],
    #URI7: [sequence7],
    #URI8: [sequence8],
    #URI9: [sequence9],
    #URI10: [sequence10],
}

# List of URIs, comment the one you do not want to fly
uris = {
    URI1,
    # URI2,
    # URI3,
    # URI4,
    #URI5,
    #URI6,
    #URI7,
    #URI8,
    #URI9,
    #URI10
}


def wait_for_position_estimator(scf, sequence):
    # print('sequence = ',sequence)
    print('Waiting for estimator to find position...')
    print('scf = ',scf)
    agent_id = sequence[0]
    print('agent_id = ', agent_id)

    cf = scf.cf

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')
    log_config.add_variable('kalman.stateX', 'float')
    log_config.add_variable('kalman.stateY', 'float')
    log_config.add_variable('kalman.stateZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    # threshold = 0.001
    threshold = 0.005

    with SyncLogger(cf, log_config) as logger:
        for log_entry in logger:
            [x_real, y_real, z_real] = pos_real[agent_id]
            print('pos_real = ', pos_real[agent_id])
            cf.extpos.send_extpos(x_real, y_real, z_real)

            data = log_entry[1]

            print("pos_mcu: ", data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ'])
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def set_initial_position(scf, sequence):
    pass
    print('scf = ',scf)
    print('set_initial_position')
    # print('sequence in set_initial_position = ', sequence)
    
    agent_id = sequence[0]
    print('agent_id = ', agent_id)

    (x_real, y_real, z_real) = pos_real[agent_id]
    yaw_deg = yaw_pref_in_deg
    print('pos_real[agent_id] = ', pos_real[agent_id])

    print('x_real = ', x_real)
    scf.cf.param.set_value('kalman.initialX', x_real)
    print('y_real = ', y_real)
    scf.cf.param.set_value('kalman.initialY', y_real)
    print('z_real = ', z_real)
    scf.cf.param.set_value('kalman.initialZ', z_real)

    print('set value for [kalman.initialYaw]')
    yaw_radians = math.radians(yaw_deg)
    print('yaw_radians = ',yaw_radians)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

def reset_estimator(scf, sequence):
    print('reset_estimator')
    # print('sequence in reset_estimator = ', sequence)
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(scf, sequence)



def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)

def position_callback(timestamp, data, logconf):
    print('position_callback')
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    print('scf in start_position_printing ', scf )
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(vz)

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def land(cf, position):
    landing_time = 1.5
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time

    print(vz)

    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def run_sequence(scf, sequence):
    # print('sequence in run_sequence = ',sequence)
    try:
        cf = scf.cf
        yaw_deg = yaw_pref_in_deg
        stop_flag = False

        optitrack_boundary = {}
        optitrack_boundary['x'] = [-2.49,2.17]
        optitrack_boundary['y'] = [-3.32,3.51]

        agent_id = sequence[0]
        print('agent_id = ', agent_id)
        [x_real, y_real, z_real] = pos_real[agent_id]
        print('pos_real = ', pos_real[agent_id])
        
        # takeoff
        take_off_time = 1.5 # s
        take_off_height = 0.5 # m
        take_off_x = x_real
        take_off_y = y_real
        end_time = time.time() + take_off_time
        while time.time() < end_time:
            [x_real, y_real, z_real] = pos_real[agent_id]
            print('pos_real = ', pos_real[agent_id])
            cf.extpos.send_extpos(x_real, y_real, z_real)
            cf.commander.send_position_setpoint(take_off_x, take_off_y, take_off_height, yaw_deg)
            time.sleep(0.1)

        # end_time2 = time.time() + 3
        # while time.time() < end_time2:
        #     [x_real, y_real, z_real] = pos_real[agent_id]
        #     print('pos_real = ', pos_real[agent_id])
        #     cf.extpos.send_extpos(x_real, y_real, z_real)
        #     # cf.commander.send_position_setpoint(-.5, -.5, take_off_height, yaw_deg)
        #     cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
        #     time.sleep(0.1)

        # move
        # targets = sequence[1:len(sequence)]
        # # print('targets = ',targets)\
        # for position in targets:
        #     print('Setting position {}'.format(position))
        #     [x_real, y_real, z_real] = pos_real[agent_id]
        #     print('pos_real = ', pos_real[agent_id])
        #     cf.extpos.send_extpos(x_real, y_real, z_real)
        #     cf.commander.send_position_setpoint(position[0], position[1], take_off_height, yaw_deg)
        #     time.sleep(0.08)
        #     # time.sleep(position[3])
        #     if  x_real<optitrack_boundary['x'][0] or x_real>optitrack_boundary['y'][1] or \
        #         y_real<optitrack_boundary['y'][0] or y_real>optitrack_boundary['y'][1] or \
        #         z_real<0.5*take_off_height:
        #         stop_flag = True
        #         break

        # land
        print('landing... ')
        # [x_real, y_real, z_real] = pos_real[agent_id]
        # print('pos_real = ', pos_real[agent_id])
        # land_height = take_off_height # m
        # land_x = x_real
        # land_y = y_real
        # while ~stop_flag and land_height > -0.3:
        #     land_height = land_height-0.015
        #     [x_real, y_real, z_real] = pos_real[agent_id]
        #     print('pos_real = ', pos_real[agent_id])
        #     cf.extpos.send_extpos(x_real, y_real, z_real)
        #     cf.commander.send_position_setpoint(land_x, land_y, land_height, yaw_deg)
        #     # cf.commander.send_velocity_world_setpoint(0, 0, - 0.1, 0)
        #     time.sleep(0.08)
        #     if  x_real<optitrack_boundary['x'][0] or x_real>optitrack_boundary['y'][1] or \
        #         y_real<optitrack_boundary['y'][0] or y_real>optitrack_boundary['y'][1]:
        #         break;

        landing_time = 1.5
        sleep_time = 0.1
        steps = int(landing_time / sleep_time)
        vz = -take_off_height/ landing_time
        for _ in range(steps):
            [x_real, y_real, z_real] = pos_real[agent_id]
            cf.extpos.send_extpos(x_real, y_real, z_real)
            cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
            time.sleep(sleep_time)
        # cf.high_level_commander.land(0.0, 2.0)
        cf.commander.send_stop_setpoint()

    except Exception as e:
        print(e)






if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    
    
    time.sleep(1)

    factory = CachedCfFactory(rw_cache='./cache')
    # with Swarm(uris, factory=factory) as swarm:
    #     pass
    #     # If the copters are started in their correct positions this is
    #     # probably not needed. The Kalman filter will have time to converge
    #     # any way since it takes a while to start them all up and connect. We
    #     # keep the code here to illustrate how to do it.
    #     # swarm.parallel(reset_estimator)

    #     # The current values of all parameters are downloaded as a part of the
    #     # connections sequence. Since we have 10 copters this is clogging up
    #     # communication and we have to wait for it to finish before we start
    #     # flying.

    #     print('Waiting for parameters to be downloaded...')
    #     swarm.parallel(wait_for_param_download, args_dict=seq_args)

    #     print('set_initial_position')
    #     swarm.parallel(set_initial_position, args_dict=seq_args)

    #     print('reset_estimator')
    #     swarm.parallel(reset_estimator, args_dict=seq_args)

    #     # print('start_position_printing')
    #     # swarm.parallel(start_position_printing)
        
    #     print('run_sequence')
    #     swarm.parallel(run_sequence, args_dict=seq_args)

    #     # time.sleep(5)
    
    swarm = Swarm(uris, factory=factory)
    swarm.open_links()
    time.sleep(5)
    print('Waiting for parameters to be downloaded...')
    swarm.parallel(wait_for_param_download, args_dict=seq_args)

    print('set_initial_position')
    swarm.parallel(set_initial_position, args_dict=seq_args)

    print('reset_estimator')
    swarm.parallel(reset_estimator, args_dict=seq_args)

    # print('start_position_printing')
    # swarm.parallel(start_position_printing)
    
    print('run_sequence')
    swarm.parallel(run_sequence, args_dict=seq_args)

    swarm.close_links()

    print('end')




