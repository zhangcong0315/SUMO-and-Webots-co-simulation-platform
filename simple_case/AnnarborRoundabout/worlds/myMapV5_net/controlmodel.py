import traci
import time
import numpy as np
import random
import pandas as pd
import math
import matplotlib.pyplot as plt
import traci.constants as tc
import decimal
decimal.getcontext().prec = 100
from decimal import Decimal, localcontext
import re


class controlModel(object):

    def __init__(self, carID):
        self.carID = carID
        self.vehicle_type = traci.vehicle.getTypeID(carID)#the type of car cav or ov
        self.speed = traci.vehicle.getSpeed(self.carID)
        self.acc = traci.vehicle.getAcceleration(self.carID)
        self.lane = traci.vehicle.getLaneID(self.carID)#return vehicle's lane ID
        [self.color1,self.color2,self.color3,self.color4]= traci.vehicle.getColor(self.carID)  # return route
        [self.x, self.y] = traci.vehicle.getPosition(self.carID)
        self.time = traci.simulation.getTime()
        self.height=traci.vehicle.getHeight(self.carID)  # did not be identified

    def outmessage(self, filename1, filename2):

        file_handle1 = open(filename1, mode='a')
        file_handle2 = open(filename2, mode='a')
        list1 = [str(self.time), self.carID, self.vehicle_type[0:3], str(self.color1), str(self.color2), str(self.color3),
                 str(self.speed),str(self.acc), self.lane, str(self.x), str(self.height)]
        list2 = [self.carID, str(self.time), self.lane, str(self.x), str(self.height)]

        s1 = ' '.join(list1)
        file_handle1.write(s1)
        file_handle1.write('\n\n')
        file_handle1.close()

        s2 = ' '.join(list2)
        file_handle2.write(s2)
        file_handle2.write('\n\n')
        file_handle2.close()




class Car(object):
    def __init__(self, carID):

        self.carID = carID
        self.steplength=0.1 #s
        self.length = traci.vehicle.getLength(self.carID)
        self.height = traci.vehicle.getHeight(self.carID)
        [self.a, self.b] = traci.vehicle.getPosition(self.carID)
        self.vehicle_type = traci.vehicle.getTypeID(self.carID)
        self.lane = traci.vehicle.getLaneID(self.carID)  # return vehicle's lane ID
        self.time=traci.simulation.getTime()
        [self.color1, self.color2, self.color3, self.color4] = traci.vehicle.getColor(self.carID)  # return route

        self.speed = traci.vehicle.getSpeed(self.carID)


    def detectors2(self):
        df = pd.read_csv('colli_ps.csv')

        df.rename(index={0: 'EN', 1: 'EW', 2: 'ES', 3: 'NE', 4: 'NW', 5: 'NS', 6: 'SE', 7: 'SN', 8: 'SW', 9: 'WN', 10: 'WE',
                   11: 'WS'}, inplace=True)

        df2=pd.read_table('location.txt',sep=" ", header=None)
        df2.rename(columns={0: 'D1', 1: 'D2', 2: '00', 3: '01', 4: '10', 5: '11'}, inplace=True)
        # df2 = pd.read_csv('colli_location.csv')
        # df2.rename(
        #     index={0: 'EN', 1: 'EW', 2: 'ES', 3: 'NE', 4: 'NW', 5: 'NS', 6: 'SE', 7: 'SN', 8: 'SW', 9: 'WN', 10: 'WE',
        #            11: 'WS'}, inplace=True)
        # df = df.loc[:, ~df.columns.str.contains('Unnamed')]
        # df2 = df2.loc[:, ~df2.columns.str.contains('Unnamed')]
        #print(df)
        direc1=self.carID[0:2]
        L=df2.loc[0,'00']
        L = re.findall(r'\d',L)  # 返回list
        traci.vehicle.subscribeContext(self.carID, tc.CMD_GET_VEHICLE_VARIABLE, 100.0, [tc.VAR_POSITION])
        self.diction = traci.vehicle.getContextSubscriptionResults(str(self.carID))
        [carIDx, carIDy] = traci.vehicle.getPosition(self.carID)
        print(carIDx,carIDy)
        checklist=[]
        for ID in self.diction.keys():
            if ID in traci.vehicle.getIDList() and ID!=self.carID:
                lane1 = traci.vehicle.getLaneID(self.carID)[-1] #str
                lane2 = traci.vehicle.getLaneID(ID)[-1]
                direc2 = ID[0:2]
                if direc2==df2.loc[0,1] and direc1==df2.loc[0,0]:
                    lane1=traci.vehicle.getlaneID(self.carID)
                    lane2=traci.vehicle.getlaneID(ID)
                    # if lane1==0 and lane2==0:
                    #     L=df2.loc[0,3]
                    # if lane1==0 and lane2==1:
                    #     L=df2.loc[0,5]
                    # if lane1 == 1 and lane2 == 0:
                    #     L = df2.loc[0, 3]
                    # if lane1 == 1 and lane2 == 1:
                    #     L = df2.loc[0, 5]
                    #     L_list.append
                print('carID,ID',self.carID,ID)
                print("direc1, direc2",direc1, direc2)

                p = df.at[direc1, direc2]

                #L=df2.at[direc1, direc2]

                [IDx, IDy] = traci.vehicle.getPosition(ID)
                D=math.sqrt((carIDx-IDx)**2+(carIDy-IDy)**2)
                if p==1 and ID not in checklist: #have already compare ID and self.carID these pair
                    checklist.append(ID)


                    print(self.carID,ID,'may having collsion')
                    print("L",L)
                    if L<10:
                        if L==1: # collision location is 1
                            x0=889.7210712153357
                            y0=791.8295988002185
                            #L1_1=882.7584059395308 798.2051870667234
                            # L1_2=871.0217575039254 798.3419355774116
                        if L==2: # collision location is 2
                            x0=866.0888405038332
                            y0 = 794.8494921537883
                        if L==3: # collision location is 3
                            x0=863.3838294530843
                            y0 =765.5419544155916
                        if L==4: # collision location is 4
                            x0=892.6110583195211
                            y0 =764.4289098190995
                        dis=math.sqrt((carIDx-x0)**2+(carIDy-y0)**2)
                    if L>10:
                        L_list=[]
                        dis_list=[]
                        L2 = L % 10
                        L1 = (L - L2) / 10 # ten
                        L_list.append(L2)
                        L_list.append(L1)
                        for L in L_list:
                            if L == 1:  # collision location is 1
                                x0 = 889.7210712153357
                                y0 = 791.8295988002185
                                # L1_1=882.7584059395308 798.2051870667234
                                # L1_2=871.0217575039254 798.3419355774116
                            if L == 2:  # collision location is 2
                                x0 = 866.0888405038332
                                y0 = 794.8494921537883
                            if L == 3:  # collision location is 3
                                x0 = 863.3838294530843
                                y0 = 765.5419544155916
                            if L == 4:  # collision location is 4
                                x0 = 892.6110583195211
                                y0 = 764.4289098190995
                            D=math.sqrt((carIDx-x0)**2+(carIDy-y0)**2)
                            dis_list.append(D)
                        dis=max(dis_list)


                    PET=1
                    TTC=2
                    speed=traci.vehicle.getSpeed(self.carID)
                    TTCi=dis/(speed+0.01)
                    if TTCi<=TTC:
                        print("found collision!!!")
                        s=TTC-TTCi
                        self.outmessage2(self.carID,ID,TTCi,s)





    def outmessage2(self,car1,car2,TTCi,s):
        filename1 = 'collision_result.txt'
        file_handle1 = open(filename1, mode='a')

        list1 = [str(car1),str(car2),str(TTCi),str(s)]

        s1 = ' '.join(list1)
        file_handle1.write(s1)
        file_handle1.write('\n\n')
        file_handle1.close()






