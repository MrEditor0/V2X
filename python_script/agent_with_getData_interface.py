Enable = True

import sys
import datetime
import math
import numpy as np   
from PanoLib.PyPanoData.PyPanoDataIO import PyPanoDataIO 
from PanoLib.PyPanoObject.PyPanoWarningOutput import PyPanoWarningOutput
import json

warning_dict={'EBW':101,'UFCW':102,'FOW':103,'PCW':104,'ICW':105,'RCW':106,'FCW':107,'VRUCW':108,'BSW':109,'LCW':110,
    'DNPW':111,'CLW':112,'TJW':113,'DOW':114,'LDW':115,'FDW':116,'SDW':117,'HMW':118,'SORW':119,'CVW':120,
    'EVW':121,'STBSD':122,'RLVW':123,'HLW':124,'SLW':125,'CSWS':126,'AVW':127,'LTA':128,'':0,'GLOSA':402,'IVS':403
}
#'Jc_8':201,'Bangsha_1':229,'Base_LeftR':202
prio_dict={'EBW':20,'UFCW':25,'FOW':26,'PCW':27,'ICW':60,'RCW':80,'FCW':120,'VRUCW':130,'BSW':140,'LCW':145,
    'DNPW':150,'CLW':170,'TJW':180,'DOW':200,'LDW':410,'FDW':420,'SDW':421,'HMW':425,'SORW':430,'CVW':449,
    'EVW':450,'STBSD':460,'RLVW':470,'HLW':480,'SLW':481,'CSWS':482,'AVW':451,'LTA':128,'GLOSA':402,'IVS':454,'RLVW':470,
}
#'Jc_8':454,'Bangsha_1':454,'Base_LeftR':454
veh_name = "Veh_1"

def initialize():
    pass


def start():
    global g_db, g_grid, vehicle_cache, sim_time,host_id
    #host_id = '65ff4a6f-e423-428f-b9f1-c5e17da2fbba'
    g_db = PyPanoDataIO()
    g_grid = g_db.get_grid()
    sim_time = 0.0
    vehicle_cache = g_grid.GetOrCreateCache[str, str]("PanoDataModel.VehicleStatus")
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
            can_thread(host_id)
            if(sensor_data!=None):
                #print(sensor_data)
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
        #print('signboard 的 前后两点的角度{0}，远车的角度{1},angle_diff :{2}'.format(str(road_angle),str(ego_heading),str(angle_diff(road_angle, ego_heading))))
        if abs(angle_diff(road_angle, ego_heading)) < DIRECTION_THRESHOLD  or abs(angle_diff(road_angle, ego_heading-180)) < DIRECTION_THRESHOLD:
            pos_ego = np.asarray([ego_latitude, ego_longitude])
            distance = abs(np.cross(current_position - last_position, last_position - pos_ego) / np.linalg.norm(current_position - last_position))
            #print('主车和车道的距离{0}'.format(distance))
            if distance < 18.75:
                angle_last = math.atan2(pos_ego[0] - last_position[0], pos_ego[1] - last_position[1]) * 180 / 3.14159
                angle_current = math.atan2(current_position[0] - pos_ego[0], current_position[1] - pos_ego[1]) * 180 / 3.14159
                if (abs(angle_diff(angle_last, ego_heading-180)) < 90 or abs(angle_diff(angle_last, ego_heading)) < 90) and abs(angle_diff(angle_current, ego_heading)) < 90:
                    return True
        return False
    for appk,appv in sensor_data[1].items():    
        for mk,message in appv.items():
            if message['Type'] == 'BSM':
                latitude = message['message']['pos']['lat']
                longitude = message['message']['pos']['long']
                heading = message['message']['heading']*180/3.14159
                speed = message['message']['speed']
                ebw = False
                avw = False
                clw = False
                evw = False
                if message['message']['brakes']['traction'] != 0:
                    ebw = True
                elif message['message']['brakes']['abs'] != 0:
                    avw = True
                elif message['message']['brakes']['scs'] != 0:
                    clw = True
                elif message['message']['vehicleClass']['classification'] != 10:
                    evw = True
                direction, lane, position, distance, approach = calculate_relative_position(latitude, longitude,heading)
                print("soucrceID:", message['SourceID'], "direction:", direction, "lane:", lane, "position:", position, "distance:", distance,"approach:", approach)
                if ebw and position == 1 and distance < 50:
                    priority = calc_priority(distance,10,20)
                    hmi_mess.append({'TYPE':'EBW','LEVEL':priority})
                elif avw and position == 1 and distance < 50:
                    priority = calc_priority(distance,20,30)
                    hmi_mess.append({'TYPE':'AVW','LEVEL':priority})
                elif clw and position == 1 and distance < 50:
                    priority = calc_priority(distance,20,30)
                    hmi_mess.append({'TYPE':'CLW','LEVEL':priority})
                elif evw and distance < 50:
                    priority = calc_priority(distance,20,30)
                    hmi_mess.append({'TYPE':'EVW','LEVEL':priority})
                elif direction == 1 and lane == 0 and position == 1 and distance < 30:
                    priority = calc_priority(distance,10,20)
                    hmi_mess.append({'TYPE':'FCW','LEVEL':priority})
                elif abs(direction) == 2 and position == 1 and distance < 50 and approach == 1:
                    priority = calc_priority(distance,10,20)
                    hmi_mess.append({'TYPE':'ICW','LEVEL':priority})
                elif ego_turn_light == -1 and direction == -1 and lane == 2 and position == 1 and distance < 50:
                    priority = calc_priority(distance,30,40)
                    hmi_mess.append({'TYPE':'LTA','LEVEL':priority})
                elif ego_turn_light == -1 and direction == 1 and lane == 1 and position == -1 and distance < 50:
                    priority = calc_priority(distance,30,40)
                    hmi_mess.append({'TYPE':'LCW','LEVEL':priority})
                elif ego_turn_light == 1 and direction == 1 and lane == -1 and position == -1 and distance < 20:
                    priority = calc_priority(distance,10,20)
                    hmi_mess.append({'TYPE':'LCW','LEVEL':priority})
                elif ego_turn_light == -1 and direction == -1 and lane == 1 and position == 1 and distance < 30:
                    priority = calc_priority(distance,20,30)
                    hmi_mess.append({'TYPE':'DNPW','LEVEL':priority})
                elif ego_turn_light == 0 and direction == 1 and abs(lane) == 1 and position == -1 and distance < 20:
                    priority = calc_priority(distance,10,20)
                    hmi_mess.append({'TYPE':'BSW','LEVEL':priority})
            elif message['Type'] == 'RSM':
                #print(message)
                for participant in message['message']['participants']:
                    pass
                latitude = message['message']['participants'][0]['posOffset']['offsetLL'][1]['lat']
                longitude = message['message']['participants'][0]['posOffset']['offsetLL'][1]['lon']
                heading = message['message']['participants'][0]['heading'] *180/3.14159
                direction, lane, position, distance, approach = calculate_relative_position(latitude, longitude, heading)
                print("id:", message['message']['id'], "direction:", direction, "lane:", lane, "position:", position, "distance:", distance, "light:", ego_turn_light, "approach:", approach)
                if position == 1 and distance < 50 and message['message']['participants'][0]['ptcType'] == 1:
                    priority = calc_priority(distance,30,40)
                    hmi_mess.append({'TYPE':'VRUCW','LEVEL':priority})
            elif message['Type'] == 'RSI':
                last_position = None
                for point in message['message']['alertPath']:
                    latitude = point['offsetLL'][1]['lat']
                    longitude = point['offsetLL'][1]['lon']
                    current_position = np.asarray([latitude, longitude])
                    if last_position is not None:
                        if check_in_road(last_position, current_position):
                            ref_latitude = message['message']['refPos']['lat']
                            ref_longitude = message['message']['refPos']['long']
                            pos_sign = np.asarray([ref_latitude, ref_longitude])
                            pos_ego = np.asarray([ego_latitude, ego_longitude])
                            distance = np.linalg.norm(pos_sign - pos_ego)
                            if distance < message['message']['alertRadius']:
                                angle = math.atan2(pos_sign[0] - pos_ego[0], pos_sign[1] - pos_ego[1]) * 180 / 3.14159
                                if message['message']['alertType'] == 1 :
                                     priority = calc_priority(distance,30,50)
                                     hmi_mess.append({'TYPE':'TJW','LEVEL':priority})
                                # if message['message']['alertType'] == 1 and abs(angle_diff(angle,ego_heading)) < DIRECTION_THRESHOLD:
                                #     priority = calc_priority(distance,30,50)
                                #     hmi_mess.append({'TYPE':'TJW','LEVEL':priority})
                                description = message['message']['description']
                                des_list = description.split('/')
                                hmi_mess.append({'TYPE':des_list[len(des_list)-1],'LEVEL':2})
                            #print('车和交通标志牌{0}的距离是{1}'.format(message['message']['description'],str(distance)))
                            break
                    last_position = current_position
            elif message['Type'] == 'MAP':
                last_position = None
                for point in message['message']['nodes'][1]['inLinks'][0]['lanes'][0]['points']:
                    latitude = point['posOffset']['offsetLL'][1]['lat']
                    longitude = point['posOffset']['offsetLL'][1]['lon']
                    current_position = np.asarray([latitude, longitude])
                    if last_position is not None:
                        print('主车速度为：{0}，限速为：{1}'.format(ego_speed,message['message']['nodes'][1]['inLinks'][0]['speedLimits'][0]['speed']))
                        if check_in_road(last_position, current_position):
                            if ego_speed >message['message']['nodes'][1]['inLinks'][0]['speedLimits'][0]['speed']:
                                hmi_mess.append({'TYPE':'SLW','LEVEL':2})
                            elif message['message']['nodes'][1]['inLinks'][0]['lanes'][0]['connectsTo'][0]['phaseId'] == spat_id and sim_time - spat_ts < 2:
                                if spat_light == 3:
                                    hmi_mess.append({'TYPE':'RLVW','LEVEL':2})
                                elif spat_light == 5:
                                    pos_ego = np.asarray([ego_latitude, ego_longitude])
                                    distance = np.linalg.norm(current_position - pos_ego)
                                    hmi_mess.append({'TYPE':'GLOSA-'+str((distance / spat_time)*3.6),'LEVEL':2})
                            break
                    last_position = current_position
            elif message['Type'] == 'SPAT':
                #stop-And-Remain (3), permissive-Movement-Allowed (5),
                spat_id = message['message']['intersections'][0]['phases'][0]['id']
                spat_light =  message['message']['intersections'][0]['phases'][0]['phaseStates'][0]['light']
                spat_time =  message['message']['intersections'][0]['phases'][0]['phaseStates'][0]['timing']['likelyEndTime'] / 1000
                #spat_ts = datetime.datetime.now().timestamp()
                spat_ts = sim_time
    print(hmi_mess)
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


def calc_priority(distance,threshold_one,threshold_two):
    priority = 0
    if distance < threshold_one:
        priority = 2
    elif distance < threshold_two:
        priority = 1
    else:
        priority = 0
    return priority

       
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
        print('exception try catch info :'+repr(e))
    finally:
        pass
        #time.sleep(1)

def calc_turn_light(left_light,right_light,cur_sim_time):
    light = ''
    left_changedtime = left_light.split(',')
    right_changedtime = right_light.split(',')
    left_size = 0
    right_size = 0
    if left_light.strip()=='':
        pass
    else:
        left_size = len(left_changedtime)

    if right_light.strip() =='':
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

    while  k<right_size and cur_sim_time > float(right_changedtime[k]):
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

#msg_type= None, message_type_id=None,warning_level=None, time_stamp=None
def send_hmi_warning(sensor_owner,hmi_type,hmi_level=None,time_stamp=None):
    try:
        code = 403 if hmi_type.split("-", 1)[0] not in warning_dict.keys() else warning_dict[hmi_type.split("-", 1)[0]]
        PyPanoWarningOutput.create_value(sensor_owner,hmi_type, code,warning_level=hmi_level)
        print('*******************************************')
        print('*******告警类型 ：'+hmi_type+'  告警级别 ：'+str(hmi_level)+' *******')
        print('*******************************************')
    except Exception as ex:
        print(str(ex))
    finally:
        pass

def get_urgent_hmi_old(hmi_mess):
    urgent_hmi = ''
    count = len(hmi_mess)
    if count > 0:
        key = hmi_mess[0]
        urgent_hmi = key
        for i in range(1,count):
            if key['TYPE'].split("-", 1)[0] in prio_dict.keys() and hmi_mess[i]['TYPE'].split("-", 1)[0] in prio_dict.keys():
                if prio_dict[key['TYPE'].split("-", 1)[0]] < prio_dict[hmi_mess[i]['TYPE'].split("-", 1)[0]]:
                    urgent_hmi = hmi_mess[i]
    return urgent_hmi

def get_urgent_hmi(hmi_mess):
    urgent_hmi = ''
    count = len(hmi_mess)
    if count > 0:
        key = hmi_mess[0]
        urgent_hmi = key
        for i in range(0,count):
            key_prio = prio_dict['IVS'] if key['TYPE'].split("-", 1)[0] not in prio_dict.keys() else prio_dict[key['TYPE'].split("-", 1)[0]]
            temp_prio = prio_dict['IVS'] if hmi_mess[i]['TYPE'].split("-", 1)[0] not in prio_dict.keys() else prio_dict[hmi_mess[i]['TYPE'].split("-", 1)[0]]
            urgent_hmi = hmi_mess[i] if key_prio < temp_prio else urgent_hmi
    return urgent_hmi