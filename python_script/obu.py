import os
import socket
import threading
import asn1tools
import pynmea2
import math
import datetime
import json
import numpy as np

HMI_WARING_TYPE = ["", "FCW", "ICW", "LTA", "LCW", "DNPW", "EBW", "AVW", "CLW", "EVW", "BSW", "RLVW", "RSA", "SLW", "GLOSA"]
ASN_DIR = '../codec/asn/'
IP_ADDRESS = '127.0.0.1'
V2X_PORT = 2498
GPS_PORT = 2497
CAN_PORT = 7112
HMI_PORT = 7130
BUFFER_SIZE = 2048
DIRECTION_THRESHOLD = 15

warning_sock = None
ego_latitude = 0.0
ego_longitude = 0.0
ego_heading = 0.0
ego_speed = 0.0
ego_turn_light = 0

def get_pos(latitude, longitude):
    y = (latitude - 37.788204) * 3.14159 * 6371004 / 180
    x = (longitude + 122.399498) * 3.14159 * 6371004 * math.cos(latitude * 3.14159 / 180) / 180
    return np.asarray([x, y])

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
    pos_ego = get_pos(ego_latitude, ego_longitude)
    pos_rv = get_pos(latitude, longitude)
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
    print('pos_rv:'+str(pos.rv)+'--- pos_ego:'+str(pos_ego))
    distance = np.linalg.norm(pos_rv - pos_ego)
    return direction, lane_no * np.sign(lane_distance), position, distance, approach

hmi_warning_ts = 0.0
hmi_warning_priority = -1
def send_hmi_warning(type, priority, subtype = 0, extra = ""):
    global hmi_warning_ts, hmi_warning_priority
    if type == 0 or priority >= hmi_warning_priority:
        hmi_warning_ts = datetime.datetime.now().timestamp()
        hmi_warning_priority = priority
        if warning_sock:
            if type == HMI_WARING_TYPE.index("GLOSA"):
                warning_sock.send(('{"TA_TM":{"Type":0,"Priority":-1,"Subtype":0,"Data":""}}').encode())
                warning_sock.send(('{"DVIN_SSM":{"Speed":%.1f}}' % (extra)).encode())
            else:
                warning_sock.send(('{"TA_TM":{"Type":%d,"Priority":%d,"Subtype":%d,"Data":"%s"}}' % (type, priority, subtype, extra)).encode())

spat_id = 0
spat_light = 0
spat_time = 0
spat_ts = 0
def v2x_thread():
    global spat_id, spat_light, spat_time, spat_ts
    def check_in_road(last_position, current_position):
        road_angle = math.atan2(current_position[0] - last_position[0], current_position[1] - last_position[1]) * 180 / 3.14159
        if abs(angle_diff(road_angle, ego_heading)) < DIRECTION_THRESHOLD:
            pos_ego = get_pos(ego_latitude, ego_longitude)
            distance = abs(np.cross(current_position - last_position, last_position - pos_ego) / np.linalg.norm(current_position - last_position))
            if distance < 10:
                angle_last = math.atan2(pos_ego[0] - last_position[0], pos_ego[1] - last_position[1]) * 180 / 3.14159
                angle_current = math.atan2(current_position[0] - pos_ego[0], current_position[1] - pos_ego[1]) * 180 / 3.14159
                if abs(angle_diff(angle_last, ego_heading)) < 90 and abs(angle_diff(angle_current, ego_heading)) < 90:
                    return True
        return False
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP_ADDRESS, V2X_PORT))
    asn = asn1tools.compile_files([ASN_DIR + s for s in os.listdir(ASN_DIR)], 'uper')
    while True:
        data, _ = sock.recvfrom(BUFFER_SIZE)
        try:
            msg = asn.decode('MessageFrame', data[5:])
        except:
            continue
        if msg[0] == 'bsmFrame':
            latitude = msg[1]['pos']['lat'] / 10000000.0
            longitude = msg[1]['pos']['long'] / 10000000.0
            heading = msg[1]['heading'] / 80.0
            ebw = False
            avw = False
            clw = False
            evw = False
            if msg[1]['safetyExt']['events'][0][0] == 0x01:
                ebw = True
            elif msg[1]['safetyExt']['events'][0][0] == 0x80:
                avw = True
            elif msg[1]['safetyExt']['events'][0][0] != 0x00:
                clw = True
            elif msg[1]['vehicleClass']['classification'] != 10:
                evw = True
            direction, lane, position, distance, approach = calculate_relative_position(latitude, longitude, heading)
            print("id:", msg[1]['id'], "direction:", direction, "lane:", lane, "position:", position, "distance:", distance, "light:", ego_turn_light, "approach:", approach)
            if ebw and position == 1 and distance < 50:
                send_hmi_warning(HMI_WARING_TYPE.index("EBW"), 1)
            elif avw and position == 1 and distance < 50:
                send_hmi_warning(HMI_WARING_TYPE.index("AVW"), 1)
            elif clw and position == 1 and distance < 50:
                send_hmi_warning(HMI_WARING_TYPE.index("CLW"), 1)
            elif evw and distance < 50:
                send_hmi_warning(HMI_WARING_TYPE.index("EVW"), 1)
            elif direction == 1 and lane == 0 and position == 1 and distance < 30:
                if distance < 10:
                    priority = 2
                elif distance < 20:
                    priority = 1
                else:
                    priority = 0
                send_hmi_warning(HMI_WARING_TYPE.index("FCW"), priority)
            elif abs(direction) == 2 and position == 1 and distance < 50 and approach == 1:
                send_hmi_warning(HMI_WARING_TYPE.index("ICW"), 2)
            elif ego_turn_light == -1 and direction == -1 and lane == -2 and position == 1 and distance < 50:
                send_hmi_warning(HMI_WARING_TYPE.index("LTA"), 1)
            elif ego_turn_light == -1 and direction == 1 and lane == -1 and position == -1 and distance < 50:
                send_hmi_warning(HMI_WARING_TYPE.index("LCW"), 1)
            elif ego_turn_light == 1 and direction == 1 and lane == 1 and position == -1 and distance < 20:
                send_hmi_warning(HMI_WARING_TYPE.index("LCW"), 1)
            elif ego_turn_light == -1 and direction == -1 and lane == -1 and position == 1 and distance < 30:
                send_hmi_warning(HMI_WARING_TYPE.index("DNPW"), 1)
            elif ego_turn_light == 0 and direction == 1 and abs(lane) == 1 and position == -1 and distance < 20:
                send_hmi_warning(HMI_WARING_TYPE.index("BSW"), 1)
        elif msg[0] == 'rsmFrame':
            latitude = msg[1]['participants'][0]['pos']['offsetLL'][1]['lat'] / 10000000.0
            longitude = msg[1]['participants'][0]['pos']['offsetLL'][1]['lon'] / 10000000.0
            heading = msg[1]['participants'][0]['heading'] / 80.0
            direction, lane, position, distance, approach = calculate_relative_position(latitude, longitude, heading)
            print("id:", msg[1]['id'], "direction:", direction, "lane:", lane, "position:", position, "distance:", distance, "light:", ego_turn_light, "approach:", approach)
            if position == 1 and distance < 50:
                send_hmi_warning(HMI_WARING_TYPE.index("RSA"), 1, 10)
        elif msg[0] == 'rsiFrame':
            last_position = None
            for point in msg[1]['alertPath']:
                latitude = point['offsetLL'][1]['lat'] / 10000000.0
                longitude = point['offsetLL'][1]['lon'] / 10000000.0
                current_position = get_pos(latitude, longitude)
                if last_position is not None:
                    if check_in_road(last_position, current_position):
                        send_hmi_warning(HMI_WARING_TYPE.index("RSA"), 1, msg[1]['alertType'], bytearray.fromhex(msg[1]['description']).decode())
                        break
                last_position = current_position
        elif msg[0] == 'mapFrame':
            last_position = None
            for point in msg[1]['nodes'][1]['inLinks'][0]['lanes'][0]['points']:
                latitude = point['posOffset']['offsetLL'][1]['lat'] / 10000000.0
                longitude = point['posOffset']['offsetLL'][1]['lon'] / 10000000.0
                current_position = get_pos(latitude, longitude)
                if last_position is not None:
                    if check_in_road(last_position, current_position):
                        if ego_speed > msg[1]['nodes'][1]['inLinks'][0]['speedLimits'][0]['speed'] * 3600.0 / 50 / 1000:
                            send_hmi_warning(HMI_WARING_TYPE.index("SLW"), 1)
                        elif msg[1]['nodes'][1]['inLinks'][0]['movements'][0]['phaseId'] == spat_id and datetime.datetime.now().timestamp() - spat_ts < 2:
                            if spat_light == 'stop-And-Remain':
                                send_hmi_warning(HMI_WARING_TYPE.index("RLVW"), 2)
                            elif spat_light == 'permissive-Movement-Allowed':
                                pos_ego = get_pos(ego_latitude, ego_longitude)
                                distance = np.linalg.norm(current_position - pos_ego)
                                send_hmi_warning(HMI_WARING_TYPE.index("GLOSA"), 0, extra = distance / spat_time)
                        break
                last_position = current_position
        elif msg[0] == 'spatFrame':
            spat_id = msg[1]['intersections'][0]['phases'][0]['id']
            spat_light = msg[1]['intersections'][0]['phases'][0]['phaseStates'][0]['light']
            spat_time = msg[1]['intersections'][0]['phases'][0]['phaseStates'][0]['timing']['likelyEndTime'] / 10
            spat_ts = datetime.datetime.now().timestamp()

def gps_thread():
    def process_gps_degree(degree):
        ff = float(degree)
        dd = int(ff / 100)
        mm = ff - dd * 100
        return dd + mm / 60
    global ego_latitude, ego_longitude, ego_heading, ego_speed
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP_ADDRESS, GPS_PORT))
    while True:
        data, _ = sock.recvfrom(BUFFER_SIZE)
        msg = pynmea2.parse(data.decode())
        ego_latitude = process_gps_degree(msg.lat)
        ego_longitude = - process_gps_degree(msg.lon)
        ego_heading = float(msg.true_course)
        ego_speed = msg.spd_over_grnd * 1.852
        if datetime.datetime.now().timestamp() - hmi_warning_ts > 1:
            send_hmi_warning(0, -1)

def can_thread():
    global ego_turn_light
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((IP_ADDRESS, CAN_PORT))
    sock.listen()
    while True:
        client_sock, _ = sock.accept()
        while True:
            try:
                msg = client_sock.recv(BUFFER_SIZE)
                if not msg:
                    break
                data = json.loads(msg[:-1])
                if data["VIS_HVSM"]["Lights"] == 4:
                    ego_turn_light = -1
                elif data["VIS_HVSM"]["Lights"] == 8:
                    ego_turn_light = 1
                else:
                    ego_turn_light = 0
            except:
                break
        client_sock.close()

def hmi_thread():
    global warning_sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((IP_ADDRESS, HMI_PORT))
    sock.listen()
    while True:
        client_sock, _ = sock.accept()
        warning_sock = client_sock
        while True:
            try:
                msg = client_sock.recv(BUFFER_SIZE)
                if not msg:
                    break
            except:
                break
        client_sock.close()

if __name__ == '__main__':
    threading.Thread(target=hmi_thread).start()
    threading.Thread(target=v2x_thread).start()
    threading.Thread(target=gps_thread).start()
    threading.Thread(target=can_thread).start()
    
