Enable = True

import sys
import json
import math
import time
import numpy as np                                                
from PanoLib.PyPanoData.PyPanoDataIO import PyPanoDataIO 
from PanoLib.PyPanoObject.PyPanoWarningOutput import PyPanoWarningOutput

warning_dict={'EBW':101,'UFCW':102,'FOW':103,'PCW':104,'ICW':105,'RCW':106,'FCW':107,'VRUCW':108,'BSW':109,'LCW':110,
    'DNPW':111,'CLW':112,'TJW':113,'DOW':114,'LDW':115,'FDW':116,'SDW':117,'HMW':118,'SORW':119,'CVW':120,
    'EVW':121,'STBSD':122,'RLVW':123,'HLW':124,'SLW':125,'CSWS':126,'AVW':127,'LTA':128
}
#veh_name 主车的名字
veh_name = "Veh_1"

def initialize():
    pass

#Veh_1
def start():
    print('------------agent starting-----------------')
    global g_db, g_grid, cache_bsm, vehicle_cache, sim_time,host_id
    host_id = get_veh_id(veh_name)
    #host_id = '65ff4a6f-e423-428f-b9f1-c5e17da2fbba'
    g_db = PyPanoDataIO()
    g_grid = g_db.get_grid()
    sim_time = 0.0

    cache_bsm = g_grid.GetOrCreateCache[str, str]('PanoDataModel.Sensor.OBU.BSM.Output')
    vehicle_cache = g_grid.GetOrCreateCache[str, str]("PanoDataModel.VehicleStatus")
    print('------------agent started-----------------')

def output(time):
    global sim_time,cache_bsm
    sim_time = time
    print(time, " ===========================")
    for sensor in sensorList:
        if sensor.available():
            bsm = cache_bsm.Get(sensor.ID)
            gps_thread(host_id)
            can_thread(host_id)
            v2x_thread(json.loads(bsm),host_id)
            #print(sensor.Owner +':'+ str(json_obj[0]))

    

def terminate():
    pass


DIRECTION_THRESHOLD = 15
ego_latitude = 0.0
ego_longitude = 0.0
ego_heading = 0.0
ego_turn_light = 0
def angle_diff(a1, a2):
    diff = a1 - a2
    if diff > 180:
        diff -= 360 
    if diff < -180:
        diff += 360
    return diff

def calculate_relative_position(latitude, longitude, heading):
    diff = angle_diff(heading, ego_heading)
    if abs(diff) < DIRECTION_THRESHOLD:
        direction = 1
    elif abs(diff - 90) < DIRECTION_THRESHOLD:
        direction = 2
    elif abs(diff) > 180 - DIRECTION_THRESHOLD:
        direction = -1
    elif abs(diff + 90) < DIRECTION_THRESHOLD:
        direction = -2
    else:
        direction = 0
    pos_ego = np.asarray([ego_latitude, ego_longitude])
    pos_rv = np.asarray([latitude, longitude])
    pos_ego_next = np.asarray([pos_ego[0] + 10 * math.sin(ego_heading * 3.14159 / 180), pos_ego[1] + 10 * math.cos(ego_heading * 3.14159 / 180)])
    #pos_ego_next = np.asarray([pos_ego[0] + 10 * math.sin(ego_heading), pos_ego[1] + 10 * math.cos(ego_heading)])
    lane_distance = np.cross(pos_ego_next - pos_ego, pos_ego - pos_rv) / np.linalg.norm(pos_ego_next - pos_ego)
    if abs(lane_distance) < 2:
        lane_no = 0
    elif abs(lane_distance) < 5:
        lane_no = 1
    else:
        lane_no = 2
    angle = math.atan2(pos_rv[0] - pos_ego[0], pos_rv[1] - pos_ego[1]) * 180 / 3.14159
    position = 1 if abs(angle_diff(angle, ego_heading)) < 90 else -1
    approach = 1 if abs(angle_diff(angle + 180, heading)) < 90 else -1
    # print('pos_rv: ')
    # print(pos_rv)
    # print(' pos_ego: ')
    # print(pos_ego)

    distance = np.linalg.norm(pos_rv - pos_ego)
    return direction, lane_no * np.sign(lane_distance), position, distance, approach

def v2x_thread(bsm_messages,sensor_owner):
    for bsm_message in bsm_messages:
        latitude = bsm_message['pos.lat']
        longitude = bsm_message['pos.long']
        heading = bsm_message['heading']*180/3.14159
        ebw = False
        avw = False
        clw = False
        evw = False
        if bsm_message['brakes.traction'] != 0:
            ebw = True
        elif bsm_message['brakes.abs'] != 0:
            avw = True
        elif bsm_message['brakes.scs'] != 0:
            clw = True
        elif bsm_message['vehicleClass.classification'] != 10:
            evw = True
        direction, lane, position, distance, approach = calculate_relative_position(latitude, longitude,heading)
        print("soucrceID:", bsm_message['SourceID'], "direction:", direction, "lane:", lane, "position:", position, "distance:", distance,"approach:", approach)
        if ebw and position == 1 and distance < 50:
            send_hmi_warning(sensor_owner,'EBW',sim_time)
        elif avw and position == 1 and distance < 50:
            send_hmi_warning(sensor_owner,'AVW',sim_time)
        elif clw and position == 1 and distance < 50:
            send_hmi_warning(sensor_owner,'CLW',sim_time)
        elif evw and distance < 50:
            send_hmi_warning(sensor_owner,'EVW',sim_time)
        elif direction == 1 and lane == 0 and position == 1 and distance < 30:
            if distance < 10:
                priority = 2
            elif distance < 20:
                priority = 1
            else:
                priority = 0
            send_hmi_warning(sensor_owner,'FCW',sim_time)
        elif abs(direction) == 2 and position == 1 and distance < 50 and approach == 1:
            send_hmi_warning(sensor_owner,'ICW',sim_time)
        elif ego_turn_light == -1 and direction == -1 and lane == 2 and position == 1 and distance < 50:
            send_hmi_warning(sensor_owner,'LTA',sim_time)
        elif ego_turn_light == -1 and direction == 1 and lane == -1 and position == -1 and distance < 50:
            send_hmi_warning(sensor_owner,'LCW',sim_time)
        elif ego_turn_light == 1 and direction == 1 and lane == 1 and position == -1 and distance < 20:
            send_hmi_warning(sensor_owner,'LCW',sim_time)
        elif ego_turn_light == -1 and direction == -1 and lane == 1 and position == 1 and distance < 30:
            send_hmi_warning(sensor_owner,'DNPW',sim_time)
        elif ego_turn_light == 0 and direction == 1 and abs(lane) == 1 and position == -1 and distance < 20:
            send_hmi_warning(sensor_owner,'BSW',sim_time)


def send_hmi_warning(sensor_owner,hmi_type,timestamp, duration=0.00,isAlwaysShow = False, info = 'warning info'):
    try:
        PyPanoWarningOutput.create_value(sensor_owner,hmi_type,warning_dict[hmi_type])
        print('*******************************************')
        print(hmi_type)
        print('*******************************************')
    except Exception as ex:
        print(str(ex))
    finally:
        pass

#ego_id应该为车辆id
def gps_thread(veh_key):
    global ego_latitude, ego_longitude, ego_heading, ego_speed
    vehicle_dynamic = g['VehicleDynamic'][veh_key]
    ego_latitude = vehicle_dynamic.Y
    ego_longitude = vehicle_dynamic.X
    ego_heading = vehicle_dynamic.Yaw*180/3.14159
    ego_speed = vehicle_dynamic.Speed
    #print('id:'+veh_key+' lat:'+str(ego_latitude)+' longti:'+str(ego_longitude)+' ego_heading:'+str(ego_heading)+' ego_speed:'+str(ego_speed))
       
def can_thread(veh_key):
    global ego_turn_light
    try:
        can_message = vehicle_cache.Get(veh_key)
        #解析json
        data = json.loads(can_message)
        left_light = data['left_light']
        right_light = data['right_light']
            
        light = calc_turn_light(left_light,right_light,sim_time)

        if light == '4':
            ego_turn_light = -1
        elif light == '8':
            ego_turn_light = 1
        else:
            ego_turn_light = 0
    except Exception as e:
        print('vehicle cache get failed ,log info :'+repr(e))
    finally:
        pass
        #time.sleep(1)

def calc_turn_light(left_light,right_light,cur_sim_time):
    light = ''
    left_changedtime = left_light.split(',')
    right_changedtime = right_light.split(',')
    left_size = 0
    right_size = 0
    if len(left_changedtime) == 1 and left_changedtime == '':
        pass
    else:
        left_size = len(left_changedtime)

    if len(right_changedtime) == 1 and right_changedtime[0] == right_light:
        pass
    else:
        right_size = len(right_changedtime)
    

 
    exteriorlightstring = 0

    j = 0
    k = 0
    while j<left_size  and cur_sim_time > float(left_changedtime[j]):
        j+=1 

    if j%2 ==0:
        exteriorlightstring = exteriorlightstring &~(1<<2)
    elif j%2 ==1:
        exteriorlightstring = exteriorlightstring | (1<<2)
    else:
        pass

    while  k<right_size and cur_sim_time > float(left_changedtime[k]) :
        k+=1
  
    if k%2 ==0:
        exteriorlightstring = exteriorlightstring &~(1<<3)
    elif k%2 ==1:
        exteriorlightstring = exteriorlightstring | (1<<3)
    else:
        pass

    if (4&exteriorlightstring) == 4 and (8&exteriorlightstring) ==8:
        light = 12+48
    elif (4&exteriorlightstring)==4:
        light = 4+48
    elif (8 & exteriorlightstring) == 8:
        light = 8 + 48
    else:
        light = 48
    return chr(light)

def get_veh_id(target_obsName):
    vehicles = g["Vehicles"]
    target_veh_key = ''
    for key in vehicles:    
        if vehicles[key].ObservableName == target_obsName:
            target_veh_key = key
    return target_veh_key
        
      