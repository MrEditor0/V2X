Enable = True
remote_tcp_server_ip = "192.168.0.99"
remote_tcp_server_port = 9999

import sys
import datetime
import math
import numpy as np   
from PanoLib.PyPanoObject.PyPanoWarningOutput import PyPanoWarningOutput
import json

warning_dict={'EBW':101,'UFCW':102,'FOW':103,'PCW':104,'ICW':105,'RCW':106,'FCW':107,'VRUCW':108,'BSW':109,'LCW':110,
    'DNPW':111,'CLW':112,'TJW':113,'DOW':114,'LDW':115,'FDW':116,'SDW':117,'HMW':118,'SORW':119,'CVW':120,
    'EVW':121,'STBSD':122,'RLVW':123,'HLW':124,'SLW':125,'CSWS':126,'AVW':127,'LTA':128,'':0,'GLOSA':402,'IVS':403
}
prio_dict={'EBW':20,'UFCW':25,'FOW':26,'PCW':27,'ICW':60,'RCW':80,'FCW':120,'VRUCW':130,'BSW':140,'LCW':145,
    'DNPW':150,'CLW':170,'TJW':180,'DOW':200,'LDW':410,'FDW':420,'SDW':421,'HMW':425,'SORW':430,'CVW':449,
    'EVW':450,'STBSD':460,'RLVW':470,'HLW':480,'SLW':481,'CSWS':482,'AVW':451,'LTA':128,'GLOSA':402,'IVS':403,'RLVW':470
}
veh_name = "Veh_1"

def initialize():
    pass


def start():
    global host_id
    print("in function start")
    host_id = get_veh_id(veh_name)
    #processer.start()
    print("finish function start")


def output(time):
    print(time, " 开始*********************************************************************************")
    for sensor in sensorList:
        if sensor.available() and sensor.Type=='Sensor.V2XModel' and veh_name+'/' in sensor.Name:
            sensor_data = sensor.getData()
            gps_thread(host_id)
            if(sensor_data!=None):
                v2x_thread(sensor_data,host_id,time)
                # for appk,appv in sensor_data[1].items():
                #     for mk,mv in appv.items():
                #         print("转换为json")
                #         #print(json.dumps(mv))
                #         v2x_thread(mv,host_id,time)
                #         print("转换为json结束")
    print(time, " 结束*********************************************************************************")


def terminate():
    #global remote_handler
    #processer.terminate()
    pass


DIRECTION_THRESHOLD = 15
ego_latitude = 0.0
ego_longitude = 0.0
ego_heading = 0.0
ego_turn_light = 0
ego_speed = 0
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

    distance = np.linalg.norm(pos_rv - pos_ego)
    return direction, lane_no * np.sign(lane_distance), position, distance, approach

spat_id = 0
spat_light = 0
spat_time = 0
spat_ts = 0
def v2x_thread(sensor_data,sensor_owner,sim_time):
    hmi_mess = []
    global spat_id, spat_light, spat_time, spat_ts
    def check_in_road(last_position, current_position):
        road_angle = math.atan2(current_position[0] - last_position[0], current_position[1] - last_position[1]) * 180 / 3.14159
        if abs(angle_diff(road_angle, ego_heading)) < DIRECTION_THRESHOLD  or abs(angle_diff(road_angle, ego_heading-180)) < DIRECTION_THRESHOLD:
            pos_ego = np.asarray([ego_latitude, ego_longitude])
            distance = abs(np.cross(current_position - last_position, last_position - pos_ego) / np.linalg.norm(current_position - last_position))
            if distance < 10:
                angle_last = math.atan2(pos_ego[0] - last_position[0], pos_ego[1] - last_position[1]) * 180 / 3.14159
                angle_current = math.atan2(current_position[0] - pos_ego[0], current_position[1] - pos_ego[1]) * 180 / 3.14159
                if (abs(angle_diff(angle_last, ego_heading-180)) < 90 or abs(angle_diff(angle_last, ego_heading)) < 90) and abs(angle_diff(angle_current, ego_heading)) < 90:
                    return True
        return False
    for appk,appv in sensor_data[1].items():    
        for mk,message in appv.items():
            if message['Type'] == 'BSM':
                pass
            elif message['Type'] == 'RSM':
                pass
            elif message['Type'] == 'RSI':
                pass
            elif message['Type'] == 'MAP':
                last_position = None
                for point in message['message']['nodes'][1]['inLinks'][0]['lanes'][0]['points']:
                        latitude = point['posOffset']['offsetLL'][1]['lat']
                        longitude = point['posOffset']['offsetLL'][1]['lon']
                        current_position = np.asarray([latitude, longitude])
                        if last_position is not None:
                            if check_in_road(last_position, current_position):
                                # if ego_speed >message['message']['nodes'][1]['inLinks'][0]['speedLimits'][0]['speed']:
                                #     send_hmi_warning(HMI_WARING_TYPE.index("SLW"), 1)
                                #if message['message']['nodes'][1]['inLinks'][0]['movements'][0]['phaseId'] == spat_id and datetime.datetime.now().timestamp() - spat_ts < 2:
                                phaseid = message['message']['nodes'][1]['inLinks'][0]['lanes'][0]['connectsTo'][0]['phaseId']
                                if message['message']['nodes'][1]['inLinks'][0]['lanes'][0]['connectsTo'][0]['phaseId'] == spat_id and sim_time - spat_ts < 2:
                                    if spat_light == 3:
                                        hmi_mess.append({'TYPE':'RLVW','LEVEL':2})
                                    elif spat_light == 5:
                                        pos_ego = np.asarray([ego_latitude, ego_longitude])
                                        distance = np.linalg.norm(current_position - pos_ego)
                                        hmi_mess.append({'TYPE':'GLOSA-'+str((distance / spat_time)*3.6),'LEVEL':1})
                                break
                        last_position = current_position
                    
            elif message['Type'] == 'SPAT':
                #stop-And-Remain (3), permissive-Movement-Allowed (5),
                spat_id = message['message']['intersections'][0]['phases'][0]['id']
                spat_light =  message['message']['intersections'][0]['phases'][0]['phaseStates'][0]['light']
                spat_time =  message['message']['intersections'][0]['phases'][0]['phaseStates'][0]['timing']['likelyEndTime'] / 1000
                #spat_ts = datetime.datetime.now().timestamp()
                spat_ts = sim_time

    urgent_hmi = get_urgent_hmi(hmi_mess)
    if urgent_hmi:
        send_hmi_warning(sensor_owner,urgent_hmi['TYPE'],hmi_level = urgent_hmi['LEVEL'],time_stamp = sim_time)
    else:
        send_hmi_warning(sensor_owner,'',hmi_level = 0,time_stamp = sim_time)

#ego_id应该为车辆id
def gps_thread(veh_key):
    global ego_latitude, ego_longitude, ego_heading, ego_speed
    vehicle_dynamic = g['VehicleDynamic'][veh_key]
    ego_latitude = vehicle_dynamic.Y
    ego_longitude = vehicle_dynamic.X
    ego_heading = vehicle_dynamic.Yaw*180/3.14159
    ego_speed = vehicle_dynamic.Speed
    #print('id:'+veh_key+' lat:'+str(ego_latitude)+' longti:'+str(ego_longitude)+' ego_heading:'+str(ego_heading)+' ego_speed:'+str(ego_speed))

def get_veh_id(target_obsName):
    vehicles = g["Vehicles"]
    target_veh_key = ''
    for key in vehicles:    
        if vehicles[key].ObservableName == target_obsName:
            target_veh_key = key
    return target_veh_key

#msg_type= None, message_type_id=None,warning_level=None, time_stamp=None
def send_hmi_warning(sensor_owner,hmi_type,hmi_level=None,time_stamp=None):
    try:
        PyPanoWarningOutput.create_value(sensor_owner,hmi_type,warning_dict[hmi_type.split("-", 1)[0]],warning_level=hmi_level)
        print('*******************************************')
        print('*******告警类型 ：'+hmi_type+'  告警级别 ：'+str(hmi_level)+' *******')
        print('*******************************************')
    except Exception as ex:
        print(str(ex))
    finally:
        pass

def get_urgent_hmi(hmi_mess):
    urgent_hmi = ''
    count = len(hmi_mess)
    if count > 0:
        key = hmi_mess[0]
        urgent_hmi = key
        for i in range(1,count-1):
            if prio_dict[key['TYPE'].split("-", 1)[0]] < prio_dict[hmi_mess[i]['TYPE'].split("-", 1)[0]]:
                urgent_hmi = hmi_mess[i]
    return urgent_hmi