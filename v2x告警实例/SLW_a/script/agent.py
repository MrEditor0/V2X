Enable = True
remote_tcp_server_ip = "192.168.0.99"
remote_tcp_server_port = 9999

import sys

from PythonHandler import TcpRemoteHandler
from PythonHandler.Processer import MessageProcesser
from PythonHandler.Filter import SimpleFilter
from PanoLib.MessageTypeCode import MessageTypeCode

import json

def initialize():
    #global processer
    #processer = MessageProcesser()
    #filter_1 = SimpleFilter(message_type=MessageTypeCode.MMWRadar, message_name=None)
    #hanlder_1 = TcpRemoteHandler(ip=remote_tcp_server_ip, port=remote_tcp_server_port, rx=remote_rx_handler)
    #processer.config_filter_action(filter=filter_1, handler=hanlder_1)
    pass


def start():
    print("in function start")
    #processer.start()
    print("finish function start")


def output(time):
    print(time, " ===========================")
    for sensor in sensorList:
        if sensor.available() and sensor.Type=='Sensor.V2XModel':
            sensor_data = sensor.getData()
            if(sensor_data!=None):
                print(sensor.Type)
                print(sensor_data)
                # for m in sensor_data[1].values():
                    # print("message", m['MessageID'], "type", m['Type'], "from", m['SourceID'], " ===========================")
                    # print(json.dumps(m))
                    # print(" ==================================")
            #processer.send(message_type=sensor.Type, message_name=sensor.Name,
            #              message_data=sensor_data)


def terminate():
    #global remote_handler
    #processer.terminate()
    pass


def local_handler(type, id, data):
    if type == "Radar":
        pass
    elif type == "Camera":
        pass


def remote_rx_handler(message):
    print(command)