Enable = True


# from PanoSimOpenAPI.InterferingVehicle import InterferingVehicle
# from PanoSimOpenAPI.TestVehicle import TestVehicle
# from PanoLib.PyPanoObject.PyPanoDisturbanceData import PyPanoDisturbanceData
from PanoSimOpenAPI.Experiment import Experiment


def initialize():
    exp = Experiment()
    # for speed in (1,2,3):
        # print('第{0}次启动'.format(speed))
        # exp.run(50)
    '''
    #c# 干扰交通模型
    value = PanoDataModel.PanoDisturbanceData()
    value.CutInAtSpeedValue = 4.5
    print(value.CutInAtSpeedValue) 
    '''
   

    # import clr
    # from clr import  PanoDataModel
    # inter_vehicle_name = 'veh_2'
    # inter_ferr_vehicle = InterferingVehicle(inter_vehicle_name)
    # print(dir(PyPanoDisturbanceData))
    # value = PanoDataModel.PanoDisturbanceData()
    # value.CutInAtSpeedValue = 4.5
    # print(value.CutInAtSpeedValue)
    # print(dir(PanoDataModel))
   

def start():
    pass

def output(time):
    pass


def terminate():
    pass

