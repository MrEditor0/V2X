Enable = True
from PanoLib.PyPanoData.PyPanoDataIO import PyPanoDataIO
import json

#CLW
def initialize():
    pass


def start():
    print('manager.py is start()')
    set_vehicle_status_demo()
  
  
def output(time):
    pass


def terminate():
    pass

'''
left_light:
right_light:
emergency_brake:
high_beam:远光灯
dipped_headlight：近光灯
trouble_light:故障灯
foglight：雾灯
automatic_lamp:自动灯
parking_lamp:停车灯
day_driving_lights：日间行车灯
ABS:
TCS:
ESP:
OTHER:
fire-engine:消防车
ambulance：救护车
police_wagon:警车
'''


def set_vehicle_status_demo():
    vehicles = g["Vehicles"]
    target_veh_key = ''
      
    target_veh_name = 'Veh_1_1'  #vehicle observableName in UI 
    #BSMData['brakes']['scs'] = vehExpParams['ESP'] #'unavailable' optioinal
    #vehicle status data
    dic = {
    'left_light':'',
    'right_light':'',
    'emergency_brake':0,
    'high_beam':0,
    'dipped_headlight':0,
    'trouble_light':0,
    'foglight':0,
    'automatic_lamp':0,
    'parking_lamp':0,
    'day_driving_lights':0,
    'ABS':0,
    'TCS':0,
    'ESP':1,
    'OTHER':0,
    'fire-engine':0,
    'ambulance':0,
    'police_wagon':0
    }

    for key in vehicles:    
        if vehicles[key].ObservableName == target_veh_name:
            target_veh_key = key
            
    vehicle_cache = PyPanoDataIO().get_grid().GetOrCreateCache[str, str]("PanoDataModel.VehicleStatus")
    vehicle_cache.Put(target_veh_key,json.dumps(dic))
    print('set success : %s' % json.dumps(dic))



    

