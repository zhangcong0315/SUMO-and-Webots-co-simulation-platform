from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import time
import math
import traci
import random
import numpy as np
import optparse

from sumolib import checkBinary
from controlmodel import controlModel
from controlmodel import Car
import traci.constants as tc
import matplotlib.pyplot as plt

import pandas as pd
from random import expovariate




def runsumo():
    try:
        sys.path.append("D:\\Webots\\update1\\update\\controllers\\sumo_supervisor")

    except ImportError:
        sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

    if_show_gui= True
    if not if_show_gui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    sumocfgfile="D:\\Webots\\update1\\update\\controllers\\sumo_supervisor\\sumo.sumocfg"
    traci.start([sumoBinary,"-c", sumocfgfile])

def cleanfile():
    file1 = 'basicmessage_output.txt'
    file2 = 'position_output.txt'
    file3 = 'detector_output.txt'
    file_handle1 = open(file1, mode='a')
    file_handle1.seek(0)
    file_handle1.truncate()
    file_handle2 = open(file2, mode='a')
    file_handle2.seek(0)
    file_handle2.truncate()
    file_handle3 = open(file3, mode='a')
    file_handle3.seek(0)
    file_handle3.truncate()


def runcar():
    global TOTlist
    vehlist=[]
    showlist=[]
    cardict={}
    cleanfile()  # clean important output and position output and rewrite them
    TOTtime=1000
    for clock in np.arange(0.0, TOTtime, 0.1):
        t1=time.time()
        traci.simulationStep()  # step length=0.1s, go one step length=0.1
        vehlist = traci.vehicle.getIDList()
        #print("identifying....................................................................................")
        file1 = 'basicmessage_output.txt'
        file2 = 'position_output.txt'

        for carID in vehlist:
            if carID not in showlist:
                c=Car(carID)
                cardict[carID]=c
                showlist.append(carID)  #a car list which have built a class
            print("carID",carID)

            cardict[carID].detectors2()
        #     controlModel(carID).outmessage(file1, file2)

        print("current clock", traci.simulation.getTime() + 0.1, "............................................")

        #txtTodf()


N = 3000 # generated vehicle number
TOTtime =15000
#TOTtime =30.0*60.0*10.0   # clock, 30 min*60s*10 step/1s
pAggCar = 0.05  # cav penetration
pCar = 1-pAggCar  # ov
i = 0
k = 8

departtime = 0
lmbda = 0.5 # 3600*0.4/ 3600 veh/s density!!!!!!!!!!
count = int(N)
intervals = [expovariate(lmbda) for i in range(count)]
timestamps = [0.0]
timestamp = 0.0
for t in intervals:
    timestamp += t
    timestamps.append(timestamp)  # timestamps restore the departure time


def random():
    pr = ''
    ram = np.random.uniform(0, 1)
    print(ram)
    if ram <= 1/12:
        pr = "routeNS" # NS
    elif ram <= 2/12:
        pr = "routeNE"
    elif ram <= 3/12:
        pr = "routeNW"
    elif ram <= 4 / 12:
        pr = "routeEW"
    elif ram <= 5/12:
        pr = "routeEN"
    elif ram <= 6/12:
        pr = "routeES"
    elif ram <= 7 / 12:
        pr = "routeSE"
    elif ram <= 8/12:
        pr = "routeSN"
    elif ram <= 9/12:
        pr = "routeSW"
    elif ram <= 10/12:
        pr = "routeWS"
    elif ram <= 11/12:
        pr = "routeNE"
    else:
        pr = "routeWN"

    return ram,pr

def speed_random():
    speed = np.random.normal(15, 0.2)
    return str(speed)
    # return speed
def cleanf():
    file1 = 'collision_result.txt'
    file_handle1 = open(file1, mode='a')
    file_handle1.seek(0)
    file_handle1.truncate()

def clean():
    file2 = 'sumo.rou.xml'
    file_handle2 = open(file2, mode='a')
    file_handle2.seek(0)
    file_handle2.truncate()
#\t<vType accel="2.5" decel="4.6" id="AggrCar" length="5.0" maxSpeed="15.0" sigma="0" guiShape="truck" speedFactor="1.4" impatience="1.0" minGapLat="0.25" lcStrategic="1" lcCooperative="1" lcSpeedGain="2" lcKeepRight="0" lcOvertakeRight="1" lcAssertive="5" lcPushy="1" lcImpatience="1" lcTimeToImpatience="0" lcCooperativeRoundabout="1"/>
#\t<vType accel="2.5" decel="4.6" id="Car" length="5.0" minGap="2.0" carFollowModel = "IDM" maxSpeed="15.0" sigma="0" lcCooperative="0" lcSpeedGain="2" guiShape="truck"/>""")

def departLane(pr):
    if pr=='routeWS'or pr=='routeWE'or pr=='routeEN' or pr=='routeSW':
        lane=1

    else:
        lane= np.random.uniform(0, 1)
        if lane<=0.5:
            lane=0
        else:
            lane=1

    return str(lane)
def writeroute():
    with open("sumo.rou.xml", "w") as routes:

        routes.write("""<routes>\n""")
        # routes.write('\n') lcSpeedGain = "1"
        routes.write("""<vTypeDistribution id = "mixed" >""")
        routes.write("""
\t<vType accel="2.5" decel="4.6" id="AggrCar" length="3.0" maxSpeed="15.0" sigma="0" guiShape="truck" speedFactor="1.4" impatience="1.0" lcAssertive='2.0' lcImpatience="1"/>
\t<vType accel="2.5" decel="4.6" id="Car" length="3.0" minGap="2.0" carFollowModel = "IDM" maxSpeed="15.0" sigma="0" impatience="-1.0" guiShape="truck"/>""")
        routes.write('\n')
        routes.write('</vTypeDistribution >')
        routes.write("""
\t<route id="routeWS" edges="442406711 392862156_1 392862156_2 392862156_3 392862156_4 392862156_5 481006164 376034504 E0 376034499_1 376034499_2 376034498_1 376034498_2 376034498_3"/>
\t<route id="routeWE" edges="442406711 392862156_1 392862156_2 392862156_3 392862156_4 392862156_5 481006164 376034504 E0 225946551_5 376034496 216974214_1 -E5 216974214_3 442406712_1 442406712_2 442534392 405036644_1 405036644_4 405036644_3 405036644_2 442534378_1"/> 
\t<route id="routeWN" edges="442406711 392862156_1 392862156_2 392862156_3 392862156_4 392862156_5 481006164 376034504 E0 225946551_5 E1 225946551_7 4691810_1 4691810_2 4691810_3 442406701 400517186 216972159 412456386 442406698 401784524"/>

\t<route id="routeEN" edges="-442406712_1 -216974214_3 E5 -216974214_1 376034497_1 376034497_2 225946551_7 4691810_1 4691810_2 4691810_3 442406701 400517186 216972159 412456386 442406698"/>
\t<route id="routeEW" edges="-442406712_1 -216974214_3 E5 -216974214_1 376034497_1 376034497_2 225946551_7 225946551_2 376034509 481006163  -481006164 -392862156_5 -392862156_4 -392862156_3 -392862156_2 -392862156_1 -442406711 -442406713_4 -442406713_2 -442406713_3 -442406713_1"/>
\t<route id="routeES" edges="-442406712_1 -216974214_3 E5 -216974214_1 376034497_1 376034497_2 225946551_7 225946551_2 225946551_3 E0 376034499_1 376034499_2 376034498_1 376034498_2 376034498_3"/>

\t<route id="routeNW" edges="442406702_1 442406702_2 123456_1 E7 123456_2 E8 123456_3 E4 481006163 -481006164 -392862156_5 -392862156_4  -392862156_3 -392862156_2 -392862156_1 -442406711 -442406713_4 -442406713_2 -442406713_3 -442406713_1"/>
\t<route id="routeNS" edges="442406702_1 442406702_2 442406702_3 222822932 225946551_3 E0 376034499_1 376034499_2 376034498_1 376034498_2 376034498_3"/>
\t<route id="routeNE" edges="442406702_1 442406702_2 442406702_3 222822932 225946551_3 E0 225946551_5 376034496 216974214_1 -E5 216974214_3 442406712_1 442406712_2 442534392 405036644_1 405036644_4 405036644_3 405036644_2 442534378_1"/>

\t<route id="routeSE" edges="-376034498_3 -376034498_2 -376034498_1 376034500_1 376034500_2 376034496 216974214_1 -E5 216974214_3 442406712_1 442406712_2 442534392 405036644_1 405036644_4 405036644_3 405036644_2 442534378_1"/>
\t<route id="routeSN" edges="-376034498_3 -376034498_2 -376034498_1 376034500_1 376034500_2 E1 225946551_7 4691810_1 4691810_2 4691810_3 442406701 400517186 216972159 412456386 442406698 401784524"/>
\t<route id="routeSW" edges="-376034498_3 -376034498_2 -376034498_1 376034500_1 376034500_2 E1 225946551_7 225946551_2 376034509 481006163 -481006164 -392862156_5 -392862156_4 -392862156_3 -392862156_2 -392862156_1 -442406711 -442406713_4 -442406713_2 -442406713_3 -442406713_1"/>


""")



        routes.write('\n')
        # tau= a drivers desired minimum time headway
        # carFollowModel="IDM"
        veh_car_0, veh_car_1 = 0, 0
        k = 0
        j = 0
        for i in range(0, N):
            departtime = timestamps[i]
            [ram,pr] = random()
            if ram <= pAggCar:  # cav
                # routes.write("""<vehicle id="car_0_""" + str(k) + """\"""" + """ type="cav" route="route1" color="255,0,0"  depart=\"""" + str(departtime) + """\"""" + """ departLane=\"""" + lane_random() + """\"""" + """ departSpeed=\"""" + speed_random() + """\"""" + """> </vehicle>""")
                routes.write("""<vehicle id=\""""+str(pr[-2:]) +"Agg_car_""" + str(k)+ """\"""" + """ type="AggrCar" route=\"""" + pr + """\"""" + """ color="225,0,0" departLane="random" depart=\"""" + str(departtime) + """"/>""")
                routes.write('\n')
                veh_car_0 += 1

                k += 1
            else:  # ov
                # routes.write("""<vehicle id="car_1_"""+ str(j) + """\"""" + """ type="ov"  route="route1" color="225,225,0"  depart=\"""" + str(departtime) + """\"""" + """ departLane=\"""" + lane_random() + """\"""" + """ departSpeed=\"""" + speed_random() +  """\"""" + """> </vehicle>""")
                routes.write("""<vehicle id=\""""+str(pr[-2:]) +"car_""" + str(j)+ """\"""" + """ type="Car"  route=\"""" + pr + """\"""" + """ color="0,225,225" departLane="random" depart=\"""" + str(departtime) + """"/>""")
                routes.write('\n')
                veh_car_1 += 1

                j += 1
        routes.write("""</routes>""")
        print("route file is ready....")




if __name__ == "__main__":
    TOTlist=[]
    #clean()
    writeroute()
    cleanf()
    clock = 0
    #runsumo()  # run sumo
    #runcar()
    #traci.close()
    sys.stdout.flush()