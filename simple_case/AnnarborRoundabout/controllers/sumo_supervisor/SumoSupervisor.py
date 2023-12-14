# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SumoSupervisor class inheriting from Supervisor."""

from controller import Supervisor, Node
from Objects import Vehicle, TrafficLight
from WebotsVehicle import WebotsVehicle
import pandas as pd
import os
import sys
import math
import re
import csv
import numpy as np
import traci.constants as tc
hiddenPosition = 10000
import pandas
# import sklearn
# from sklearn import linear_model


def rotation_from_yaw_pitch_roll(yaw, pitch, roll):
    """Compute the axis-angle rotation from the yaw pitch roll angles"""
    rotation = [0, 0, 1, 0]
    # construct rotation matrix
    # a b c
    # d e f
    # g h i
    a = math.cos(roll) * math.cos(yaw)
    b = -math.sin(roll)
    c = math.cos(roll) * math.sin(yaw)
    d = math.sin(roll) * math.cos(yaw) * math.cos(pitch) + math.sin(yaw) * math.sin(pitch)
    e = math.cos(roll) * math.cos(pitch)
    f = math.sin(roll) * math.sin(yaw) * math.cos(pitch) - math.cos(yaw) * math.sin(pitch)
    g = math.sin(roll) * math.cos(yaw) * math.sin(pitch) - math.sin(yaw) * math.cos(pitch)
    h = math.cos(roll) * math.sin(pitch)
    i = math.sin(roll) * math.sin(yaw) * math.sin(pitch) + math.cos(yaw) * math.cos(pitch)
    # convert it to rotation vector
    cosAngle = 0.5 * (a + e + i - 1.0)
    if math.fabs(cosAngle) > 1:
        return rotation
    else:
        rotation[0] = b - d
        rotation[1] = f - h
        rotation[2] = g - c
        rotation[3] = math.acos(cosAngle)
        # normalize vector
        length = math.sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1] + rotation[2] * rotation[2])
        if length != 0:
            rotation[0] = rotation[0] / length
            rotation[1] = rotation[1] / length
            rotation[2] = rotation[2] / length
        if rotation[0] == 0 and rotation[1] == 0 and rotation[2] == 0:
            return [0, 0, 1, 0]
        else:
            return rotation


class SumoSupervisor (Supervisor):
    """This is the main class that implements the actual interface."""

    def get_viewpoint_position_field(self):
        """Look for the 'position' field of the Viewpoint node."""
        children = self.getRoot().getField("children")
        number = children.getCount()
        for i in range(0, number):
            node = children.getMFNode(i)
            if node.getType() == Node.VIEWPOINT:
                return node.getField("position")
        return None

    def get_initial_vehicles(self):
        """Get all the vehicles (both controlled by SUMO and Webots) already present in the world."""
        for i in range(0, self.vehiclesLimit):
            defName = "SUMO_VEHICLE%d" % self.vehicleNumber
            node = self.getFromDef(defName)
            if node:
                self.vehicles[i] = Vehicle(node)
                self.vehicles[i].name.setSFString("SUMO vehicle %i" % self.vehicleNumber)
                self.vehicleNumber += 1
            else:
                break
        for i in range(0, self.vehiclesLimit):
            defName = "WEBOTS_VEHICLE%d" % self.webotsVehicleNumber
            web=defName
            print("webots name", defName)
            node = self.getFromDef(defName)
            if node:
                self.webotsVehicles[i] = WebotsVehicle(node, self.webotsVehicleNumber)
                print("self.webots",self.webotsVehicles)
                self.webotsVehicleNumber += 1
            else:
                break
        return web



    def generate_new_vehicle(self, vehicleClass):
        """Generate and import a new vehicle that will be controlled by SUMO."""
        # load the new vehicle

        vehicleString, defName = Vehicle.generate_vehicle_string(self.vehicleNumber, vehicleClass)
        self.rootChildren.importMFNodeFromString(-1, vehicleString)
        self.vehicles[self.vehicleNumber] = Vehicle(self.getFromDef(defName))
        self.vehicleNumber += 1

    def get_vehicle_index(self, id, generateIfneeded=True):
        """Look for the vehicle index corresponding to this id (and optionnaly create it if required)."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].currentID == id:
                # the vehicle was already here at last step
                return i
        if not generateIfneeded:
            return -1
        # the vehicle was not present last step
        # check if a corresponding vehicle is already in the simulation
        node = self.getFromDef(id)
        if node and (node.getTypeName() in Vehicle.get_car_models_list() or
                     node.getTypeName() in Vehicle.get_bus_models_list() or
                     node.getTypeName() in Vehicle.get_truck_models_list() or
                     node.getTypeName() in Vehicle.get_motorcycle_models_list()):
            self.vehicles[self.vehicleNumber] = Vehicle(node)
            self.vehicles[self.vehicleNumber].currentID = id
            self.vehicleNumber += 1
            return self.vehicleNumber - 1
        # check if a vehicle is available
        vehicleClass = self.get_vehicle_class(id)
        for i in range(0, self.vehicleNumber):
            if not self.vehicles[i].inUse and self.vehicles[i].vehicleClass == vehicleClass:
                # if a vehicle is available assign it to this id
                self.vehicles[i].currentID = id
                self.vehicles[i].name.setSFString(id)
                return i
        # no vehicle available => generate a new one if limit is not reached
        if self.vehicleNumber < self.vehiclesLimit:
            vehicleClass = self.get_vehicle_class(id)
            self.generate_new_vehicle(vehicleClass)
            return self.vehicleNumber - 1
        return -1

    def get_vehicle_class(self, id):
        """Get the class of the vehicle associated to this id."""
        if id in self.vehiclesClass:
            return self.vehiclesClass[id]
        vehicleClass = Vehicle.get_corresponding_vehicle_class(self.traci.vehicle.getVehicleClass(id))
        self.vehiclesClass[id] = vehicleClass
        return vehicleClass

    def disable_unused_vehicles(self, IdList):
        """Check for all the vehicles currently used if they need to be disabled."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].inUse and self.vehicles[i].currentID not in IdList:
                self.vehicles[i].inUse = False
                self.vehicles[i].name.setSFString("SUMO vehicle %i" % i)
                self.vehicles[i].currentLane = None
                self.vehicles[i].currentRoad = None
                self.vehicles[i].laneChangeStartTime = None
                self.vehicles[i].laneChangeDistance = 0

    def hide_unused_vehicles(self):
        """Hide all the newly unused vehicles."""
        for i in range(0, self.vehicleNumber):
            if not self.vehicles[i].inUse:
                if self.vehicles[i].targetPos[0] != hiddenPosition:
                    self.vehicles[i].targetPos = [hiddenPosition, i * 10, 0.5]
                    self.vehicles[i].currentPos = [hiddenPosition, i * 10, 0.5]
                    self.vehicles[i].currentRot = [0, 0, 1, 0]
                    self.vehicles[i].targetRot = [0, 0, 1, 0]
                    self.vehicles[i].currentAngles = [0, 0, 0]
                    self.vehicles[i].targetAngles = [0, 0, 0]
                    self.vehicles[i].translation.setSFVec3f([hiddenPosition, i * 10, 0.5])
                    self.vehicles[i].node.setVelocity([0, 0, 0, 0, 0, 0])
                    for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                        wheelAngularVelocity.setSFVec3f([0, 0, 0])

    def stop_all_vehicles(self):
        """Stop all the vehicles (to be called when controller exits)."""
        for i in range(0, self.vehicleNumber):
            self.vehicles[i].node.setVelocity([0, 0, 0, 0, 0, 0])
            for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                wheelAngularVelocity.setSFVec3f([0, 0, 0])

    def get_vehicles_position(self, id, subscriptionResult, step, xOffset, yOffset,
                              maximumLateralSpeed, maximumAngularSpeed, laneChangeDelay):
        """Compute the new desired position and orientation for all the vehicles controlled by SUMO."""
        if not subscriptionResult:
            return
        height = 0.4
        roll = 0.0
        pitch = 0.0
        sumoPos = subscriptionResult[self.traci.constants.VAR_POSITION]
        sumoAngle = subscriptionResult[self.traci.constants.VAR_ANGLE]
        pos = [sumoPos[0] + xOffset, sumoPos[1] + yOffset, height]
        angle = -math.pi * sumoAngle / 180
        dx = math.cos(angle)
        dz = -math.sin(angle)
        yaw = -math.atan2(dx, dz)
        # correct position (origin of the car is not the same in Webots / sumo)
        vehicleLength = subscriptionResult[self.traci.constants.VAR_LENGTH]
        pos[0] += 0.5 * vehicleLength * math.sin(angle)
        pos[1] -= 0.5 * vehicleLength * math.cos(angle)
        # if needed check the vehicle is in the visibility radius
        if self.radius > 0:
            viewpointPosition = self.viewpointPosition.getSFVec3f()
            xDiff = viewpointPosition[0] - pos[0]
            yDiff = viewpointPosition[1] - pos[1]
            zDiff = viewpointPosition[2]
            distance = math.sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff)
            if distance > self.radius:
                index = self.get_vehicle_index(id, generateIfneeded=False)
                if index >= 0:
                    self.vehicles[index].inUse = False
                    self.vehicles[index].currentID = ""
                    self.vehicles[index].name.setSFString("SUMO vehicle %i" % index)
                return
        index = self.get_vehicle_index(id)
        if index >= 0:
            vehicle = self.vehicles[index]
            height = vehicle.wheelRadius
            if self.enableHeight:
                roadID = subscriptionResult[self.traci.constants.VAR_ROAD_ID]

                roadPos = subscriptionResult[self.traci.constants.VAR_LANEPOSITION]
                if roadID.startswith(":"):
                    # this is a lane change it does not contains edge information
                    # in that case, use previous height, roll and pitch
                    height = vehicle.currentPos[2]
                    roll = vehicle.roll
                    pitch = vehicle.pitch
                else:
                    tags = roadID.split('_')
                    del tags[0]  # remove the first one which is the 'id' of the road
                    for tag in tags:
                        if tag.startswith('height'):
                            height = height + float(tag.split('height', 1)[1])
                        elif tag.startswith('roll'):
                            roll = float(tag.split('roll', 1)[1])
                        elif tag.startswith('pitch'):
                            pitch = float(tag.split('pitch', 1)[1])
                    vehicle.pitch = pitch
                    vehicle.roll = roll
                    # ajust height according to the pitch
                    if pitch != 0:
                        height += (roadPos - 0.5 * vehicleLength) * math.sin(pitch)
                    # ajust height according to the roll and lateral position of the vehicle
                    if roll != 0.0:
                        laneIndex = subscriptionResult[self.traci.constants.VAR_LANE_INDEX]
                        laneID = subscriptionResult[self.traci.constants.VAR_LANE_ID]
                        laneWidth = self.traci.lane.getWidth(laneID)
                        edge = self.net.getEdge(roadID)
                        numberOfLane = edge.getLaneNumber()
                        # compute lateral distance from the center of the lane
                        distance = math.fabs((laneIndex - numberOfLane / 2) + 0.5) * laneWidth
                        if laneIndex >= (numberOfLane / 2):
                            height = height - distance * math.sin(roll)
                        else:
                            height = height + distance * math.sin(roll)
            pos[2] = height
            if vehicle.inUse:
                # TODO: once the lane change model of SUMO has been improved
                #       (sub-lane model currently in development phase) we will be able to remove this corrections

                # compute longitudinal (x) and lateral (y) displacement
                diffX = pos[0] - vehicle.targetPos[0]
                diffY = pos[1] - vehicle.targetPos[1]
                x1 = math.cos(-angle) * diffX - math.sin(-angle) * diffY
                y1 = math.sin(-angle) * diffX + math.cos(-angle) * diffY
                # check for lane change

                if (vehicle.currentRoad is not None and
                        vehicle.currentRoad == subscriptionResult[self.traci.constants.VAR_ROAD_ID] and
                        vehicle.currentLane is not None and
                        vehicle.currentLane != subscriptionResult[self.traci.constants.VAR_LANE_INDEX]):
                    vehicle.laneChangeStartTime = self.getTime()
                    vehicle.laneChangeDistance = x1
                x2 = x1
                # artificially add an angle depending on the lateral speed
                artificialAngle = 0
                if y1 > 0.0001:  # don't add the angle if speed is very small as atan2(0.0, 0.0) is unstable
                    # the '0.15' factor was found empirically and should not depend on the simulation
                    artificialAngle = 0.15 * math.atan2(x1, y1)
                if (vehicle.laneChangeStartTime is not None and
                        vehicle.laneChangeStartTime > self.getTime() - laneChangeDelay):  # lane change case
                    ratio = (self.getTime() - vehicle.laneChangeStartTime) / laneChangeDelay
                    ratio = (0.5 + 0.5 * math.sin((ratio - 0.5) * math.pi))
                    p = vehicle.laneChangeDistance * ratio
                    x2 = x1 - (vehicle.laneChangeDistance - p)
                    artificialAngle = math.atan2(-x2, y1)
                # limit lateral speed
                threshold = 0.001 * step * maximumLateralSpeed
                x2 = min(max(x2, -threshold), threshold)
                x3 = math.cos(angle) * x2 - math.sin(angle) * y1
                y3 = math.sin(angle) * x2 + math.cos(angle) * y1
                pos = [x3 + vehicle.targetPos[0], y3 + vehicle.targetPos[1], pos[2]]
                diffYaw = yaw - vehicle.targetAngles[2] - artificialAngle
                # limit angular speed
                diffYaw = (diffYaw + 2*math.pi) % (2*math.pi)
                if (diffYaw > math.pi):
                    diffYaw -= 2*math.pi
                threshold = 0.001 * step * maximumAngularSpeed
                diffYaw = min(max(diffYaw, -threshold), threshold)
                yaw = diffYaw + vehicle.targetAngles[2]
                # tilt motorcycle depending on the angluar speed
                if vehicle.type in Vehicle.get_motorcycle_models_list():
                    threshold = 0.001 * step * maximumLateralSpeed
                    roll -= min(max(diffYaw / (0.001 * step), -0.2), 0.2)
            rot = rotation_from_yaw_pitch_roll(yaw, pitch, roll)
            if not vehicle.inUse:
                # this vehicle was previously not used, move it directly to the correct initial location
                vehicle.inUse = True
                vehicle.currentPos = pos
                vehicle.currentRot = rot
                vehicle.currentAngles = [roll, pitch, yaw]
            else:
                vehicle.currentPos = vehicle.targetPos
                vehicle.currentRot = vehicle.targetRot
                vehicle.currentAngles = vehicle.targetAngles
            # update target and wheels speed
            vehicle.targetPos = pos
            vehicle.targetRot = rot
            vehicle.targetAngles = [roll, pitch, yaw]
            if self.traci.constants.VAR_SPEED in subscriptionResult:
                vehicle.speed = subscriptionResult[self.traci.constants.VAR_SPEED]
            vehicle.currentRoad = subscriptionResult[self.traci.constants.VAR_ROAD_ID]
            vehicle.currentLane = subscriptionResult[self.traci.constants.VAR_LANE_INDEX]
            return vehicle.currentRoad

    def update_vehicles_position_and_velocity(self, step, rotateWheels):
        """Update the actual position (using angular and linear velocities) of all the vehicles in Webots."""
        for i in range(0, self.vehicleNumber):
            if self.vehicles[i].inUse:
                self.vehicles[i].translation.setSFVec3f(self.vehicles[i].currentPos)
                self.vehicles[i].rotation.setSFRotation(self.vehicles[i].currentRot)
                velocity = []
                velocity.append(self.vehicles[i].targetPos[0] - self.vehicles[i].currentPos[0])
                velocity.append(self.vehicles[i].targetPos[1] - self.vehicles[i].currentPos[1])
                velocity.append(self.vehicles[i].targetPos[2] - self.vehicles[i].currentPos[2])
                for j in range(0, 3):
                    diffAngle = self.vehicles[i].currentAngles[j] - self.vehicles[i].targetAngles[j]
                    diffAngle = (diffAngle + 2*math.pi) % (2*math.pi)
                    if (diffAngle > math.pi):
                        diffAngle -= 2*math.pi
                    velocity.append(diffAngle)
                velocity[:] = [1000 * x / step for x in velocity]
                self.vehicles[i].node.setVelocity(velocity)
                if rotateWheels:
                    angularVelocity = [0, self.vehicles[i].speed / self.vehicles[i].wheelRadius, 0]
                    for wheelAngularVelocity in self.vehicles[i].wheelsAngularVelocity:
                        wheelAngularVelocity.setSFVec3f(angularVelocity)

    def update_webots_vehicles(self, xOffset, yOffset):
        """Update the position of all the vehicles controlled by Webots in SUMO."""
        for i in range(0, self.webotsVehicleNumber):
            if self.webotsVehicles[i].is_on_road(xOffset, yOffset, self.maxWebotsVehicleDistanceToLane, self.net):
                self.webotsVehicles[i].update_position(self.getTime(), self.net, self.traci, self.sumolib, xOffset, yOffset)
            else:
                # the controlled vehicle is not on any road
                # => we remove it from sumo network
                if self.webotsVehicles[i].name in self.traci.vehicle.getIDList():
                    self.traci.vehicle.remove(self.webotsVehicles[i].name)

    def get_traffic_light(self, IDlist):
        """Get the state of all the traffic lights controlled by SUMO."""
        self.trafficLightNumber = len(IDlist)
        self.trafficLights = {}
        LEDNames = []
        for i in range(0, self.getNumberOfDevices()):
            device = self.getDeviceByIndex(i)
            if device.getNodeType() == Node.LED:
                LEDNames.append(device.getName())
        for i in range(0, self.trafficLightNumber):
            id = IDlist[i]
            self.trafficLights[id] = TrafficLight()
            self.trafficLights[id].lightNumber = len(self.traci.trafficlight.getRedYellowGreenState(id))
            for j in range(0, self.trafficLights[id].lightNumber):
                trafficLightNode = self.getFromDef("TLS_" + id + "_" + str(j))
                if trafficLightNode is not None:
                    self.trafficLights[id].trafficLightRecognitionColors[j] = trafficLightNode.getField("recognitionColors")
                ledName = id + "_" + str(j) + "_"
                if ledName + "r" in LEDNames:
                    self.trafficLights[id].LED[3 * j + 0] = self.getDevice(ledName + "r")
                else:
                    self.trafficLights[id].LED[3 * j + 0] = None
                if ledName + "y" in LEDNames:
                    self.trafficLights[id].LED[3 * j + 1] = self.getDevice(ledName + "y")
                else:
                    self.trafficLights[id].LED[3 * j + 1] = None
                if ledName + "g" in LEDNames:
                    self.trafficLights[id].LED[3 * j + 2] = self.getDevice(ledName + "g")
                else:
                    self.trafficLights[id].LED[3 * j + 2] = None

    def update_traffic_light_state(self, id, states):
        """Update the traffic lights state in Webots."""
        # update light LED state if traffic light state has changed
        currentState = states[self.traci.constants.TL_RED_YELLOW_GREEN_STATE]
        if self.trafficLights[id].previousState != currentState:
            self.trafficLights[id].previousState = currentState
            for j in range(0, self.trafficLights[id].lightNumber):
                # Update red LED if it exists
                if self.trafficLights[id].LED[3 * j + 0]:
                    if currentState[j] == 'r' or currentState[j] == 'R':
                        self.trafficLights[id].LED[3 * j + 0].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [1, 0, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 0].set(0)
                # Update yellow LED if it exists
                if self.trafficLights[id].LED[3 * j + 1]:
                    if currentState[j] == 'y' or currentState[j] == 'Y':
                        self.trafficLights[id].LED[3 * j + 1].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [1, 0.5, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 1].set(0)
                # Update green LED if it exists
                if self.trafficLights[id].LED[3 * j + 2]:
                    if currentState[j] == 'g' or currentState[j] == 'G':
                        self.trafficLights[id].LED[3 * j + 2].set(1)
                        # update recognition colors
                        if j in self.trafficLights[id].trafficLightRecognitionColors:
                            self.trafficLights[id].trafficLightRecognitionColors[j].setMFColor(1, [0, 1, 0])
                    else:
                        self.trafficLights[id].LED[3 * j + 2].set(0)

    # def cleanfile(self,file1):
    #     file_handle1 = open(file1, mode='a')
    #     file_handle1.seek(0)
    #     file_handle1.truncate()
    #
    #
    # def outmessage(self,filename1, T, car, id, TTCi,speed1,speed2,AnotherTrail,WeDirc,AgreesiveLevel,AfterMerging,x_we,y_we,x_sumo,y_sumo,inroundabout,passyeildlane,dij,v1,v2,heading1,heading2,theta0,theta1,theta2):
    #     # speed1_webot  speed2:sumo
    #     file_handle1 = open(filename1, mode='a')
    #     # init: inroundabout=0;passyeildlane=0; otherwise:inroundabout=1;passyeildlane=1
    #     list1 = [str(T), str(car), str(id), str(TTCi),str(speed1), str(speed2),str(AnotherTrail),str(WeDirc),str(AgreesiveLevel),str(AfterMerging),str(x_we),str(y_we),str(x_sumo),str(y_sumo),str(inroundabout),str(passyeildlane),str(dij),str(v1),str(v2),str(heading1),str(heading2),str(theta0),str(theta1),str(theta2)]
    #     if filename1=="total_data.txt":
    #         print(list1)
    #
    #     s1 = ' '.join(list1)
    #     file_handle1.write(s1)
    #     file_handle1.write('\n')
    #     file_handle1.close()
    #
    # def savewarninginfo(self,filename,warning):
    #     if os.path.isfile(filename):
    #         # print("Not exist, creat!")
    #         with open (filename,"w") as f:
    #             f.write(str(warning))
    #     if not os.path.isfile(filename):
    #         # print("Not exist, creat!")
    #         with open (filename,"w") as f:
    #             f.write(str(warning))
    # #
    # # def PredictAggVehSpeed(self,x, y, R_webot, speed1, x_sumo, y_sumo, R_sumo,Distance,heading1,heading2):
    # #     TTC=2
    # #     yj = y
    # #     xj = x
    # #     rj = R_webot
    # #
    # #     yi = y_sumo
    # #     xi = x_sumo
    # #     ri = R_sumo
    # #     dij = math.sqrt((x - x_sumo) ** 2 + (y - y_sumo) ** 2) - ri - rj
    # #     ar = math.radians(abs(xj - xi) / Distance)
    # #     theta1 = math.degrees(math.acos(ar)) + 90 - heading1
    # #
    # #     v1 = math.cos(math.radians(theta1)) * speed1
    # #     theta2 = 90 - math.degrees(math.acos(ar)) + heading2
    # #     print("dij,TTC,v1",dij,TTC,v1)
    # #     v2 =abs(dij/TTC)-v1
    # #     #print("v2",v2)
    # #     speed2 = v2/math.cos(math.radians(theta2))
    # #
    # #     return speed2
    #
    #
    # def check2(self,x, y, R_webot, speed1, x_sumo, y_sumo, R_sumo, speed2,Distance,heading1,heading2):
    #
    #     yj = y
    #     xj = x
    #     rj = R_webot
    #
    #     yi = y_sumo
    #     xi = x_sumo
    #     ri = R_sumo
    #
    #     dij = math.sqrt((x - x_sumo) ** 2 + (y - y_sumo) ** 2) - ri - rj
    #     # print("dij, ri,rj", dij, ri, rj)
    #     # print("heading1,heading2",heading1,heading2)
    #     theta0 = math.degrees(math.acos(abs(xj - xi) / Distance))
    #     if heading1>0 and heading1<90:
    #         theta1=heading1
    #
    #         # print("theta0",theta0)
    #         v1 = math.cos(math.radians(90 - theta1 - theta0)) * speed1
    #     if heading1>270 and heading1<360:
    #         theta1=360-heading1
    #         v1= math.cos(math.radians(theta1+90 - theta0)) * speed1
    #     if heading1>90 and heading1<180:
    #         theta1=heading1-90
    #         v1 = math.cos(math.radians(180-theta1-theta0)) * speed1
    #     if heading1>180 and heading1<270:
    #         theta1=heading1-180
    #         v1 = math.cos(math.radians(90-theta1-theta0)) * speed1
    #
    #     if heading2>270 and heading2<=360:
    #         theta2=heading2-270
    #         v2 = math.cos(math.radians(theta2+ theta0)) * speed2
    #         if v1+v2!=0:
    #             TTC = abs(dij / (v1 + v2))
    #         else:
    #             TTC = abs(dij / (v1 + v2+0.01))
    #     elif heading2<270 and heading2>=180:
    #         theta2=270-heading2
    #         v2 = math.cos( math.radians(theta0-theta2))* speed2
    #         if v1+v2!=0:
    #             TTC = abs(dij / (v1 + v2))
    #         else:
    #             TTC = abs(dij / (v1 + v2+0.01))
    #     elif heading2<180 and heading2>=90:
    #         theta2=180-heading2
    #         v2 = math.cos( math.radians(90-theta0+theta2))* speed2
    #         if v1+v2!=0:
    #             TTC = abs(dij / (v1 + v2))
    #         else:
    #             TTC = abs(dij / (v1 + v2+0.01))
    #     elif heading2<90 and heading2>=0:
    #         theta2=heading2
    #         v2 = math.cos( math.radians(90-theta0-theta2))* speed2
    #         if v1+v2!=0:
    #             TTC = abs(dij / (v1 + v2))
    #         else:
    #             TTC = abs(dij / (v1 + v2+0.01))
    #     else:
    #         print("ERROR!!!!!!!!!!!!!!!!!!!!!!!!")
    #     return TTC,dij,v1,v2,heading1,heading2,theta0,theta1,theta2
    #
    # def choose_Route(self,choice,Lane):
    #     r1='376034504'
    #     r2 = 'E0'
    #     if Lane == str(r1+ '_0') or Lane == str(r1 + '_1'):
    #         if choice==1:
    #             RID = "0"
    #     if Lane == str(r2+ '_0') or Lane == str(r2 + '_1'):
    #         if choice==1:
    #             RID = "routeSN"
    #     else:
    #         RID = "routeSN"
    #
    #     return RID
    #     #return car,RID
    #
    #
    #
    # def GetAggSpeed2(self,speed_we,D1,D2):
    #     speed_su=min(15,speed_we* (D2/D1))
    #     return speed_su
    #
    #
    def run(self, port, disableTrafficLight, directory, step, rotateWheels,
            maxVehicles, radius, enableHeight, useDisplay, displayRefreshRate,
            displayZoom, displayFitSize, maximumLateralSpeed, maximumAngularSpeed,
            laneChangeDelay, traci, sumolib):
        """Main loop function."""
        try:

            print('AConnect to SUMO... This operation may take a few seconds.')
            self.step(step)
            traci.init(port, numRetries=20)

        except:
            sys.exit('Unable to connect to SUMO, please make sure any previous instance of SUMO is closed.\n You can try'
                     ' changing SUMO port using the "--port" argument.')


    #
    #     def Remain_Dis_new(WeDirc,x_we,y_we,x_sumo,y_sumo):#carID is sumo aggressive vehicle, ID is webot vehcile
    #
    #         speed_sumo_v1 =trans_speed( traci.vehicle.getSpeed("Agg"))
    #         if WeDirc=="WE":
    #             warning_distance =speed_sumo_v1 * 2.0# forward 1s issue warning, 6 meter is the distance from the yeild point to the collision point
    #         if WeDirc=="WN":
    #             warning_distance =speed_sumo_v1 * 2.0# forward 1s issue warning
    #         warning = 0
    #         if WeDirc=="WE":
    #             if x_sumo>840.65: #every far away
    #                 D2 = max(0.1, 0+13+24.15+16.35 + 8.1 + 18.3 - math.sqrt((x_sumo - 864.2) ** 2 + (y_sumo - 1141.2) ** 2))
    #             elif x_sumo>823: # 1 area
    #                 D2 = max(0.1, 0+13+16.35 + 8.1 + 18.3 - math.sqrt((x_sumo - 839.65) ** 2 + (y_sumo - 1135.04) ** 2))
    #             elif x_sumo<=823 and x_sumo>=814: # 2 area
    #                 D2=max(0.1,0+13+6.35+8.1-math.sqrt((x_sumo - 822.7) ** 2 + (y_sumo - 1128.29) ** 2))
    #                 if warning_distance>=D2:
    #                     D2 = -2
    #                     warning=1
    #             elif x_sumo<814 and x_sumo>803:#797:#795: #3 area
    #                 D2=max(0.1,0+13+16.35-math.sqrt((x_sumo - 813.2) ** 2 + (y_sumo - 1124.4) ** 2))
    #                 #D2=max(0.1,16.35-math.sqrt((x_sumo - 814.68) ** 2 + (y_sumo - 1127.20) ** 2))
    #                 if warning_distance>=D2:
    #                     D2=-2
    #                     warning = 1
    #             elif x_sumo<803 and x_sumo>785:
    #                 D2=-2
    #                 warning = 1
    #             else:
    #                 D2=-1
    #
    #
    #             if y_we>1050 and y_we<1077:
    #                 D1=max(0.1, 28.56+36.8+22.3-math.sqrt((x_we - 781) ** 2 + (y_we - 1049.5) ** 2))
    #             elif y_we<1100 and y_we>1077:
    #                 D1=max(0.1, 36.8+22.3-math.sqrt((x_we - 783) ** 2 + (y_we - 1078) ** 2))
    #             elif y_we<1113.29 and y_we>1100:
    #                 D1=max(0.1,34.8-math.sqrt((x_we - 794.54) ** 2 + (y_we - 1097.67) ** 2))
    #             elif y_we>=1113.29 and y_we<=1130:
    #                 D1 = max(0.1,34.8 -17.6- math.sqrt((x_we - 802.05) ** 2 + (y_we - 1113.29) ** 2))
    #             else:
    #                 D1=-1
    #
    #
    #         if WeDirc=="WN":
    #             if y_sumo>1155.28 and y_sumo<1178:
    #                 D2 = max(0.1,10+ 22.18+7.84+16.2 - math.sqrt((x_sumo - 779.5) ** 2 + (y_sumo - 1178.63) ** 2))
    #             elif y_sumo>1139 and y_sumo<1155.28:
    #                 D2 = max(0.1, 10+16.2 + 7.84  - math.sqrt((x_sumo - 779.8) ** 2 + (y_sumo - 1155.63) ** 2))
    #                 if warning_distance>=D2:
    #                     D2=-2
    #                     warning = 1
    #             elif y_sumo<=1139 and y_sumo>=1124:
    #                 D2=-2
    #                 warning = 1 # true
    #             else:
    #                 D2=-1
    #
    #             if x_we<794.63:
    #                 D1=max(0.1, 21.98-math.sqrt((x_we - 794.63) ** 2 + (y_we - 1128.38) ** 2))
    #             elif y_we>1050 and y_we<1077:
    #                 D1=max(0.1, 21.98+28.56+36.8+22.3-math.sqrt((x_we - 781) ** 2 + (y_we - 1049.5) ** 2))
    #             elif y_we<1100 and y_we>1077:
    #                 D1=max(0.1,21.98+36.8+22.3-math.sqrt((x_we - 783) ** 2 + (y_we - 1078) ** 2))
    #             elif y_we<1113.29 and y_we>1100:
    #                 D1=max(0.1,21.98+34.8-math.sqrt((x_we - 794.54) ** 2 + (y_we - 1097.67) ** 2))
    #             elif y_we>=1113.29 and y_we<=1130:
    #                 D1 = max(0.1,21.98+34.8 -17.6- math.sqrt((x_we - 802.05) ** 2 + (y_we - 1113.29) ** 2))
    #             else:
    #                 D1=-1
    #
    #
    #         # print("D1,D2,speed_v1",D1,D2,warning,speed_sumo_v1)
    #         return D1,D2,warning
    #
    #     # def Remain_Dis(Chosen_Tobe_Aggveh,we_trigerDirec,ID,x_we,y_we,carID,x_sumo,y_sumo,speed1,AgreesiveLevel):#carID is sumo aggressive vehicle, ID is webot vehcile
    #         #     D1=0 # non sence
    #         #     D2=0 # non sence
    #         #     area='area'
    #         #     if we_trigerDirec=="WE":
    #         #         if x_sumo>823:
    #         #             D2 = max(0.1, 16.35 + 8.1 + 18.3 - math.sqrt((x_sumo - 839.65) ** 2 + (y_sumo - 1135.04) ** 2))
    #         #             area='area1'
    #         #         elif x_sumo<=823 and x_sumo>=814:
    #         #             D2=max(0.1,16.35+8.1-math.sqrt((x_sumo - 822.7) ** 2 + (y_sumo - 1128.29) ** 2))
    #         #             area='area2'
    #         #         elif x_sumo<804 and x_sumo>780: # the first waiting location
    #         #             D2=max(0.1,16.35-math.sqrt((x_sumo - 814) ** 2 + (y_sumo - 1125) ** 2))
    #         #             # D2=-1
    #         #
    #         #
    #         #         if y_we<1113.29 and y_we>1100:
    #         #             D1=max(0.1, 34.8-math.sqrt((x_we - 794.54) ** 2 + (y_we - 1097.67) ** 2))
    #         #         elif y_we>=1113.29 and y_we<=1130:
    #         #             D1 = max(0.1, 34.8 -17.6- math.sqrt((x_we - 802.05) ** 2 + (y_we - 1113.29) ** 2))
    #         #         elif y_we<=1100:
    #         #             D1=0
    #         #         elif y_we>=1130 and  y_we<1200:
    #         #             D1=0.5
    #         #
    #         #
    #         #
    #         #     elif we_trigerDirec=="WN":
    #         #         if y_sumo>1155.63 and y_sumo<1212.82:
    #         #             D2 = max(0.1, 16.2 + 7.84 +56.64 - math.sqrt((x_sumo - 780.13) ** 2 + (y_sumo - 1155.63) ** 2))
    #         #             area='area1'
    #         #         elif y_sumo<=1155.63 and y_sumo>=1147.8:
    #         #             D2=max(0.1,16.2+7.84-math.sqrt((x_sumo - 779.61) ** 2 + (y_sumo - 1147.81) ** 2))
    #         #             area='area2'
    #         #         elif y_sumo<1147.8 and y_sumo>1139:#795:
    #         #             D2=max(0.1,16.2-math.sqrt((x_sumo - 777.61) ** 2 + (y_sumo - 1139.81) ** 2))
    #         #             #D2=-1
    #         #
    #         #
    #         #         if y_we>=1098 and y_we<1130 and x_we>792:
    #         #             D1=max(0.1, 21.98+34.8-math.sqrt((x_we - 794.54) ** 2 + (y_we - 1097.67) ** 2))
    #         #         if x_we<=792 and x_we>=772 and y_we>=1123:
    #         #             D1 = max(0.1,21.98- math.sqrt((x_we - 776.87) ** 2 + (y_we - 1131.86) ** 2))
    #         #         if x_we<772 and x_we>=760:
    #         #             D1=-1
    #         #
    #         #
    #         #     elif we_trigerDirec=="SN":
    #         #         if y_sumo>1155.63 and y_sumo<1212.82:
    #         #             D2 = max(0.1, 16.2 + 7.84 +56.64 - math.sqrt((x_sumo - 780.13) ** 2 + (y_sumo - 1155.63) ** 2))
    #         #             area='area1'
    #         #         elif y_sumo<=1155.63 and y_sumo>=1147.8:
    #         #             D2=max(0.1,16.2+7.84-math.sqrt((x_sumo - 779.61) ** 2 + (y_sumo - 1147.81) ** 2))
    #         #             area='area2'
    #         #         elif y_sumo<1147.8 and y_sumo>1132:#795:
    #         #             D2=-1
    #         #
    #         #
    #         #         if x_we>=794.66 and x_we<802 and y_we>1123:
    #         #             D1=max(0.1, 21.98+7.78-math.sqrt((x_we - 802) ** 2 + (y_we - 1126.28) ** 2))
    #         #         if x_we<794.66 and x_we>=766 and y_we>=1119:
    #         #             D1 = max(0.1,21.98- math.sqrt((x_we -794.66) ** 2 + (y_we - 1128.86) ** 2))
    #         #
    #         #
    #         #
    #         #     elif we_trigerDirec=="SW":
    #         #         if x_sumo<736.24: #area1
    #         #             D2 = max(0.1, 23.4+17.06+8.54+15.36+11.34 - math.sqrt((x_sumo -696.28) ** 2 + (y_sumo - 1103.66) ** 2))
    #         #             area='area1'
    #         #         elif x_sumo>=736.24 and x_sumo<=744.78:#area2
    #         #             D2=max(0.1,8.54+15.36+11.34 -math.sqrt((x_sumo - 736.24) ** 2 + (y_sumo - 1110.07) ** 2))
    #         #             area='area2'
    #         #         elif x_sumo>744.78 and x_sumo<=769.49: #area3
    #         #             D2=-1
    #         #
    #         #
    #         #         if y_we<=1128.37 and y_we>1115.56 and x_we<=775:
    #         #             D1=max(0.1, 10.65+5.8+15.2-math.sqrt((x_we - 770.71) ** 2 + (y_we - 1128.37) ** 2))
    #         #         elif y_we<=1115.56 and y_we>1109.76 and x_we<=775:
    #         #             D1 = max(0.1, 10.65+5.8-math.sqrt((x_we - 764.31) ** 2 + (y_we - 1115.56) ** 2))
    #         #         elif y_we<=1109.76:
    #         #             D1= max(0.1, 10.65-math.sqrt((x_we - 764.61) ** 2 + (y_we - 1109.76) ** 2))
    #         #
    #         #
    #         #     elif we_trigerDirec=="EW":
    #         #         if x_sumo<736.24: #area1
    #         #             D2 = max(0.1, 23.4+17.06+8.54+15.36+11.34 - math.sqrt((x_sumo -696.28) ** 2 + (y_sumo - 1103.66) ** 2))
    #         #             area='area1'
    #         #         elif x_sumo>=736.24 and x_sumo<=744.78:#area2
    #         #             D2=max(0.1,8.54+15.36+11.34 -math.sqrt((x_sumo - 736.24) ** 2 + (y_sumo - 1110.07) ** 2))
    #         #             area='area2'
    #         #         elif x_sumo>744.78 and x_sumo<=769.49: #area3
    #         #             D2=-1
    #         #
    #         #
    #         #         if y_we<=1128.37 and y_we>1115.56:
    #         #             D1=max(0.1, 10.65+5.8+15.2-math.sqrt((x_we - 770.71) ** 2 + (y_we - 1128.37) ** 2))
    #         #         elif y_we<=1115.56 and y_we>1109.76:
    #         #             D1 = max(0.1, 10.65+5.8-math.sqrt((x_we - 764.31) ** 2 + (y_we - 1115.56) ** 2))
    #         #         elif y_we<=1109.76:
    #         #             D1= max(0.1, 10.65-math.sqrt((x_we - 764.61) ** 2 + (y_we - 1109.76) ** 2))
    #         #
    #         #
    #         #     elif we_trigerDirec=="ES":
    #         #         if y_sumo<1061.51: #area1
    #         #             D2 = max(0.1, 27.5+11.1+9.94 - math.sqrt((x_sumo -781.04) ** 2 + (y_sumo - 1050.54) ** 2))
    #         #             area='area1'
    #         #         elif y_sumo<=1077.9:#area2
    #         #             D2=max(0.1,16.45+11.1+9.94 -math.sqrt((x_sumo - 782.18) ** 2 + (y_sumo - 1061.51) ** 2))
    #         #             area='area2'
    #         #         elif y_sumo<=1097.6: #area3
    #         #             D2=max(0.1,11.1+9.94 -math.sqrt((x_sumo - 783.6) ** 2 + (y_sumo - 1077.9) ** 2))
    #         #
    #         #
    #         #         if y_we<=1128.37 and y_we>1109.15 and x_we<772.68: #area1
    #         #             D1=max(0.1, 9.2+12.6+6.42+13.82+15.07-math.sqrt((x_we - 773.46) ** 2 + (y_we - 1128) ** 2))
    #         #         elif y_we<=1109.15 and x_we<775.83:#area2
    #         #             D1 = max(0.1, 13.82+15.07-math.sqrt((x_we - 765.22) ** 2 + (y_we - 1109.15) ** 2))
    #         #         elif x_we>=775.83 and x_we<=791: #area3
    #         #             D1= max(0.1, 15.07-math.sqrt((x_we - 775.83) ** 2 + (y_we - 1097.45) ** 2))
    #         #
    #         #     elif we_trigerDirec=="NS":
    #         #         if y_sumo<1061.51: #area1
    #         #             D2 = max(0.1, 27.5+11.1+9.94 - math.sqrt((x_sumo -781.04) ** 2 + (y_sumo - 1050.54) ** 2))
    #         #             area='area1'
    #         #         elif y_sumo<=1077.9:#area2
    #         #             D2=max(0.1,16.45+11.1+9.94 -math.sqrt((x_sumo - 782.18) ** 2 + (y_sumo - 1061.51) ** 2))
    #         #             area='area2'
    #         #         elif y_sumo<=1097.6: #area3
    #         #             D2=max(0.1,11.1+9.94 -math.sqrt((x_sumo - 783.6) ** 2 + (y_sumo - 1077.9) ** 2))
    #         #
    #         #
    #         #         if x_we<=773.4 and x_we>=759.4:
    #         #             D1=max(0.1, 18.5+8.7+8.2-math.sqrt((x_we - 759.4) ** 2 + (y_we - 1104.4) ** 2))
    #         #         elif x_we<=782 and x_we>773.4:
    #         #             D1 = max(0.1, 8.7+8.2-math.sqrt((x_we - 773.4) ** 2 + (y_we - 1097) ** 2))
    #         #         elif x_we<=790.1 and x_we>782:
    #         #             D1= max(0.1, 8.2-math.sqrt((x_we - 782) ** 2 + (y_we - 1096) ** 2))
    #         #
    #         #     elif we_trigerDirec=="NE":
    #         #         if x_sumo>823:
    #         #             D2 = max(0.1, 16.35 + 8.1 + 18.3 - math.sqrt((x_sumo - 839.65) ** 2 + (y_sumo - 1135.04) ** 2))
    #         #             area='area1'
    #         #         elif x_sumo<=823 and x_sumo>=814:
    #         #             D2=max(0.1,16.35+8.1-math.sqrt((x_sumo - 822.7) ** 2 + (y_sumo - 1128.29) ** 2))
    #         #             area='area2'
    #         #         elif x_sumo<814 and x_sumo>780: # the first waiting location
    #         #             D2=-1
    #         #
    #         #
    #         #
    #         #         if y_we<1113.29 and y_we>1100:
    #         #             D1=max(0.1, 34.8-math.sqrt((x_we - 793) ** 2 + (y_we - 1097.67) ** 2))
    #         #         elif y_we>=1113.29 and y_we<=1130:
    #         #             D1 = max(0.1, 34.8 -17.6- math.sqrt((x_we - 802.05) ** 2 + (y_we - 1113.29) ** 2))
    #         #         elif y_we<=1100:
    #         #             D1=0
    #         #         elif y_we>=1130 and  y_we<1200:
    #         #             D1=0.5
    #         #
    #         #
    #         #     print("888888888888888888888888888888888888888888888888888")
    #
    #     # if x_sumo>840.65:
    #     #     D2 = max(0.1, 10+24.15+16.35 + 8.1 + 18.3 - math.sqrt((x_sumo - 864.2) ** 2 + (y_sumo - 1141.2) ** 2))
    #     # elif x_sumo>823:
    #     #     D2 = max(0.1, 10+16.35 + 8.1 + 18.3 - math.sqrt((x_sumo - 839.65) ** 2 + (y_sumo - 1135.04) ** 2))
    #     # elif x_sumo<=823 and x_sumo>=814:
    #     #     D2=max(0.1,10+6.35+8.1-math.sqrt((x_sumo - 822.7) ** 2 + (y_sumo - 1128.29) ** 2))
    #     # elif x_sumo<814 and x_sumo>802:#795:
    #     #     D2=max(0.1,10+16.35-math.sqrt((x_sumo - 813.2) ** 2 + (y_sumo - 1124.4) ** 2))
    #     #     #D2=max(0.1,16.35-math.sqrt((x_sumo - 814.68) ** 2 + (y_sumo - 1127.20) ** 2))
    #     # elif x_sumo<802 and x_sumo>785:
    #     #     D2=-2
    #     # else:
    #     #     D2=-1
    #         #     AggsetSpeed(Chosen_Tobe_Aggveh,x_we,y_we,x_sumo,y_sumo,D1,D2,l1,w1,l2,w2,speed1,speed2,heading1,heading2,area,WeDirc,AgreesiveLevel)
    #
    #     def addAggvehicle(Aggdirection, currentT):  # Num_change_D is the number of changing direction (direc="WE","NE")
    #         car = "Agg"
    #         Route = str("short_route" + Aggdirection)
    #         traci.vehicle.addFull(vehID=car, routeID=Route, typeID="HV", depart=currentT, departLane="first",
    #                               departPos='base', departSpeed="random",
    #                               arrivalLane='current', arrivalPos='max',
    #                               arrivalSpeed='current', line='', personCapacity=0, personNumber=0)
    #         traci.vehicle.setColor(car, (225, 0, 0))  # red
    #         return car
    #
    #     #
    #     # def AggsetSpeed(Chosen_Tobe_Aggveh,x_we,y_we,x_sumo,y_sumo,D1,D2,l1,w1,l2,w2,speed1,speed2,heading1,heading2,area,WeDirc,AgreesiveLevel):
    #     #     if D1!=0 and D2!=0 and D2!=-1 and area=='area1': #area 1
    #     #         if AgreesiveLevel == 1:
    #     #             X = 1#1
    #     #         if AgreesiveLevel == 2:
    #     #             X =0.5 #0.5
    #     #         if AgreesiveLevel == 0:
    #     #             X = 2.5  # 0.5
    #     #         denominator=(D1 / speed1) - X
    #     #         if denominator>0:
    #     #             if WeDirc == "WN":
    #     #                 speed_sumo = D2 / ((D1 / speed1) - X)
    #     #                 speed_sumo = max(speed_sumo, 6.7)
    #     #             if WeDirc == "WE":
    #     #                 speed_sumo = D2 / ((D1/ speed1) - X)
    #     #                 speed_sumo = max(speed_sumo, 6.7)
    #     #             speed_sumo=max(speed_sumo,speed1)
    #     #             print("webotspeed", speed1)
    #     #             print("sumospeed", speed_sumo)
    #     #             traci.vehicle.setSpeed("Agg", speed_sumo)
    #     #
    #     #             if AgreesiveLevel!=0:
    #     #                 AggressiveVehSettingWithSpeed(Chosen_Tobe_Aggveh,speed_sumo)
    #     #             if AgreesiveLevel == 0:
    #     #                 traci.vehicle.setSpeed("Agg", speed_sumo)
    #     #
    #     #     elif D1!=0 and D2!=0 and D2!=-1 and area=='area2': #area2
    #     #         if AgreesiveLevel == 1:
    #     #             X = 1#1
    #     #         if AgreesiveLevel == 2:
    #     #             X =0.5#0.5
    #     #         if AgreesiveLevel == 0:
    #     #             X =2.5#0.5
    #     #         denominator = (D1 / speed1) - X
    #     #         if denominator > 0:
    #     #             if WeDirc == "WN":
    #     #                 speed_sumo = D2 / ((D1 / speed1) - X)
    #     #                 speed_sumo = max(speed_sumo, 6.7)
    #     #             if WeDirc == "WE":
    #     #                 speed_sumo = D2 / ((D1 / speed1) - X)
    #     #                 speed_sumo = max(speed_sumo, 6.7)
    #     #             print("webotspeed", speed1)
    #     #             print("sumospeed", speed_sumo)
    #     #             speed_sumo = max(speed_sumo,speed1)
    #     #
    #     #
    #     #             if AgreesiveLevel != 0:
    #     #                 AggressiveVehSettingWithSpeed(Chosen_Tobe_Aggveh, speed_sumo)
    #     #
    #     #             if AgreesiveLevel == 0:
    #     #                 traci.vehicle.setSpeed("Agg", speed_sumo)
    #     #
    #     #     if AgreesiveLevel == 0 and "Agg" in traci.vehicle.getIDList():
    #     #         Chosen_Tobe_Aggveh = ""
    #     #         if x_we < 782 and yAgg < 1140 and WeDirc == "WN":  # x_we>772 and
    #     #             traci.vehicle.setSpeed("Agg", 0)
    #     #
    #     #         if y_we < 1133 and xAgg < 808 and WeDirc == "WE":  # y_we>1118 and
    #     #             traci.vehicle.setSpeed("Agg", 0)
    #     #
    #     #         traci.vehicle.setSpeedMode("Agg", 31)
    #     #     print('==========================================================================')
    #
    #     def AggressiveVehSettingWithSpeed(Chosen_Tobe_Aggveh,speed):
    #         traci.vehicle.setSpeed(Chosen_Tobe_Aggveh,max(speed,1))
    #         traci.vehicle.setMinGap(Chosen_Tobe_Aggveh,0.)
    #         traci.vehicle.setTau(Chosen_Tobe_Aggveh,0.0)
    #         traci.vehicle.setLaneChangeMode(Chosen_Tobe_Aggveh,0)
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "ignore-accidents",True)
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "collision.mingap-factor",str(0))
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "time-to-impatience",str(0))
    #
    #     def AggressiveVehSetting(Chosen_Tobe_Aggveh,AgreesiveLevel):
    #         if AgreesiveLevel==1:
    #             traci.vehicle.setSpeed(Chosen_Tobe_Aggveh,8.2)
    #         if AgreesiveLevel==2:
    #             traci.vehicle.setSpeed(Chosen_Tobe_Aggveh, 10.2)
    #         traci.vehicle.setMinGap(Chosen_Tobe_Aggveh,0.)
    #         traci.vehicle.setTau(Chosen_Tobe_Aggveh,0.0)
    #         traci.vehicle.setLaneChangeMode(Chosen_Tobe_Aggveh,0)
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "ignore-accidents",True)
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "collision.mingap-factor",str(0))
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "time-to-impatience",str(0))
    #
    #     def AggressiveVehSetting_curve(Chosen_Tobe_Aggveh,AgreesiveLevel):
    #         if AgreesiveLevel == 1:
    #             traci.vehicle.setSpeed(Chosen_Tobe_Aggveh,5.6)
    #         if AgreesiveLevel == 1:
    #             traci.vehicle.setSpeed(Chosen_Tobe_Aggveh,6.6)
    #         traci.vehicle.setMinGap(Chosen_Tobe_Aggveh,0.)
    #         traci.vehicle.setTau(Chosen_Tobe_Aggveh,0.0)
    #         traci.vehicle.setLaneChangeMode(Chosen_Tobe_Aggveh,0)
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "ignore-accidents",True)
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "collision.mingap-factor",str(0))
    #         traci.vehicle.setParameter(Chosen_Tobe_Aggveh, "time-to-impatience",str(0))
    #
    #
    #     def getLastTrailvehList(idList):
    #         newList=list(idList)
    #         if "webotsVehicle0" in newList:
    #             newList.remove("webotsVehicle0")
    #         if "webotsVehicle1" in newList:
    #             newList.remove("webotsVehicle1")
    #         return newList
    #
    #     def deleteVehicle(LastTrailvehList):
    #         for veh in LastTrailvehList:
    #             traci.vehicle.remove(veh)
    #
    #
    #     def checkConflictDirec(WebotsDirec,GenerDirec):
    #         ConflictDirec=False
    #         if WebotsDirec=="WE":
    #             if GenerDirec=="SN" or GenerDirec=="SW":
    #                 ConflictDirec=True
    #         elif WebotsDirec=="WN":
    #             if GenerDirec=="ES" or GenerDirec=="EW":
    #                 ConflictDirec=True
    #         elif WebotsDirec=="SN":
    #             if GenerDirec=="ES" or GenerDirec=="EW":
    #                 ConflictDirec=True
    #         elif WebotsDirec=="SW":
    #             if GenerDirec=="NS" or GenerDirec=="NE":
    #                 ConflictDirec=True
    #         elif WebotsDirec=="EW":
    #             if GenerDirec=="NS" or GenerDirec=="NE":
    #                 ConflictDirec=True
    #         elif WebotsDirec=="ES":
    #             if GenerDirec=="WE" or GenerDirec=="WN":
    #                 ConflictDirec=True
    #         elif WebotsDirec=="NS":
    #             if GenerDirec=="WE" or GenerDirec=="WN":
    #                 ConflictDirec=True
    #         elif WebotsDirec=="NE":
    #             if GenerDirec=="SN" or GenerDirec=="SW":
    #                 ConflictDirec=True
    #         return ConflictDirec
    #
    #     def getInfluence_Flow(direc1):
    #         if direc1=="WE" or direc1=="WN":
    #             InfluenceFlow_Direction =["NS","NE"]
    #         if direc1=="SN" or direc1=="SW":
    #             InfluenceFlow_Direction =["WE","WN"]
    #         if direc1=="EW" or direc1=="ES":
    #             InfluenceFlow_Direction =["SN","SW"]
    #         if direc1=="NS" or direc1=="NE":
    #             InfluenceFlow_Direction =["EW","ES"]
    #         return InfluenceFlow_Direction
    #
    #     def addvehicle(direc1,currentT,lastID,df_veh_add_drop,Num_change_D):#Num_change_D is the number of changing direction (direc="WE","NE")
    #         vehNumber=16
    #         startRow=(Num_change_D)*vehNumber
    #         endRow=(Num_change_D+1)*vehNumber
    #         dfx=df_veh_add_drop[startRow:endRow]
    #         dfx=dfx.reset_index()
    #         TimeBase=round(float(df_veh_add_drop.loc[0, 'time'])/10,2) # shrik 10 times to speed up the show up of the next 30 vehicles
    #         randomNumber=np.random.uniform(0,1)
    #         InfluenceFlow_Direction=getInfluence_Flow(direc1)
    #         for i in range(0,vehNumber):
    #             Time=round(float(dfx.loc[i, 'time'])/10,2)
    #             departT=str(float(currentT)+(Time-TimeBase))
    #             car =str(int(lastID)+i+1)
    #             if randomNumber>0.3:
    #                 direction=InfluenceFlow_Direction[0]
    #             else:
    #                 direction = InfluenceFlow_Direction[1]
    #             Route=str("route"+direction)
    #             traci.vehicle.addFull(vehID=car, routeID=Route, typeID="HV",depart=departT, departLane="first",departPos='base', departSpeed="random",
    #                                   arrivalLane='current', arrivalPos='max',
    #                                   arrivalSpeed='current', line='', personCapacity=0, personNumber=0)
    #             traci.vehicle.setColor(car, (225, 0, 0))  # red
    #
    #
    #



        self.traci = traci
        self.sumolib = sumolib
        self.radius = radius
        self.enableHeight = enableHeight
        self.sumoClosed = False
        self.temporaryDirectory = directory
        self.rootChildren = self.getRoot().getField("children")
        self.viewpointPosition = self.get_viewpoint_position_field()
        self.maxWebotsVehicleDistanceToLane = 15
        self.webotsVehicleNumber = 0
        self.webotsVehicles = {}
        self.vehicleNumber = 0
        self.vehicles = {}
        self.vehiclesLimit = maxVehicles
        self.vehiclesClass = {}

        # for backward compatibility
        if self.traci.constants.TRACI_VERSION <= 15:
            self.traci.trafficlight = self.traci.trafficlights

        # get sumo vehicles already present in the world
        web=self.get_initial_vehicles()

        # parse the net and get the offsets
        self.net = sumolib.net.readNet((directory + "/sumo.net.xml").replace('/', os.sep))
        xOffset = -self.net.getLocationOffset()[0]
        yOffset = -self.net.getLocationOffset()[1]

        # Load plugin to the generic SUMO Supervisor (if any)
        self.usePlugin = False
        if os.path.exists((directory + "/plugin.py").replace('/', os.sep)):
            self.usePlugin = True
            sys.path.append(directory)
            import plugin
            sumoSupervisorPlugin = plugin.SumoSupervisorPlugin(self, self.traci, self.net)

        # Get all the LEDs of the traffic lights
        if not disableTrafficLight:
            trafficLightsList = self.traci.trafficlight.getIDList()
            self.get_traffic_light(trafficLightsList)
            for id in trafficLightsList:
                # subscribe to traffic lights state
                self.traci.trafficlight.subscribe(id, [self.traci.constants.TL_RED_YELLOW_GREEN_STATE])

        # Subscribe to new vehicles entering the simulation
        self.traci.simulation.subscribe([
            self.traci.constants.VAR_DEPARTED_VEHICLES_IDS,
            self.traci.constants.VAR_MIN_EXPECTED_VEHICLES
        ])

        # Create the vehicle variable subscription list
        self.vehicleVariableList = [
            self.traci.constants.VAR_POSITION,
            self.traci.constants.VAR_ANGLE,
            self.traci.constants.VAR_LENGTH,
            self.traci.constants.VAR_ROAD_ID,
            self.traci.constants.VAR_LANE_INDEX
        ]
        if rotateWheels:
            self.vehicleVariableList.append(self.traci.constants.VAR_SPEED)
        if enableHeight:
            self.vehicleVariableList.extend([
                self.traci.constants.VAR_ROAD_ID,
                self.traci.constants.VAR_LANEPOSITION,
                self.traci.constants.VAR_LANE_ID
            ])



        # create the SUMO display
        self.sumoDisplay = None
        if useDisplay:
            view = self.traci.gui.getIDList()[0]
            display = self.getDevice('sumo')
            if display is not None:
                from SumoDisplay import SumoDisplay
                self.sumoDisplay = SumoDisplay(display, displayZoom, view, directory, displayRefreshRate, displayFitSize,
                                               self.traci)
        x=0
        Road = ""
        s = 0
        diction={}
        # self.cleanfile('total_data.txt')

        checklist=[]
        checklist1=[]


        with open("speed_from_sumo.csv", "w",newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["time","speed_we", "speed1kmh","x_we","y_we","speedsumo", "x_sumo","y_sumo","AnotherTrail", "WeDirc", "AgreesiveLevel","inroundabout","passyeildlane"])
            AggVehList = []
            AggVehListOld=[]
            edge_we_List=[]
            Flag=0


            # Main simulation loop
            while self.step(step) >= 0:

                if self.usePlugin:
                    sumoSupervisorPlugin.run(step)

                if self.sumoDisplay is not None:
                    self.sumoDisplay.step(step)

                # try to perform a SUMO step, if it fails it means SUMO has been closed by the user
                try:
                    self.traci.simulationStep()
                except self.traci.exceptions.FatalTraCIError:
                    print("Sumo closed")
                    self.sumoClosed = True
                    break

                result = self.traci.simulation.getSubscriptionResults()

                # SUMO simulation over (no more vehicle are expected)
                if result[self.traci.constants.VAR_MIN_EXPECTED_VEHICLES] == 0:
                    break

                # subscribe to new vehicle
                for id in result[self.traci.constants.VAR_DEPARTED_VEHICLES_IDS]:
                    if not id.startswith("webotsVehicle"):
                        self.traci.vehicle.subscribe(id, self.vehicleVariableList)
                    elif self.sumoDisplay is not None and len(self.webotsVehicles) == 1:
                        # Only one vehicle controlled by Webots => center the view on it
                        self.traci.gui.trackVehicle(view, 'webotsVehicle0')

                # get result from the vehicle subscription and apply it
                idList = self.traci.vehicle.getIDList()


                vehicles = []
                Flag1=0
                flag = 0
                start_vel = 5
                stopList = set()
                changeLane = {}
                IDList=[]
                inroundabout = 0
                passyeildlane=0

                # set up the start speed of each vehicle
                currentT=traci.simulation.getTime()
                for ID in idList:
                    traci.vehicle.setLaneChangeMode(ID, 0)
                    if ID == "1":
                        traci.vehicle.setSpeed(ID, 0)
                    if ID=="webotsVehicle0":
                        if currentT == 0.1:
                            [webotx, weboty] = traci.vehicle.getPosition(ID)
                            webotxold = webotx - 1
                            webotyold = weboty - 1
                        if currentT > 0.1:
                            webotxold = webotx
                            webotyold = weboty

                        [webotx, weboty] = traci.vehicle.getPosition(ID)
                        WEBOTS_VEHICLE1 = 'webotsVehicle1'
                        if WEBOTS_VEHICLE1 in traci.vehicle.getIDList():
                            [supervisor_location_x, supervisor_location_y] = traci.vehicle.getPosition(WEBOTS_VEHICLE1)




                    Vcur_road=self.get_vehicles_position(ID, self.traci.vehicle.getSubscriptionResults(ID),
                                               step, xOffset, yOffset, maximumLateralSpeed, maximumAngularSpeed,
                                               laneChangeDelay)

                    if traci.vehicle.getRouteID(ID)[-2:]=="SN" or  traci.vehicle.getRouteID(ID)[-2:]=="SW":
                        if traci.vehicle.getRoadID(ID)=="-376034498_3":
                            traci.vehicle.setSpeed(ID, 10 + 1.2 * traci.vehicle.getSpeed("webotsVehicle0"))

                    if ID=='webotsVehicle0':
                        Type='webot'


                self.disable_unused_vehicles(idList)

                # hide unused vehicles
                self.hide_unused_vehicles()

                if not disableTrafficLight:
                    for id in self.trafficLights:
                        self.update_traffic_light_state(id, self.traci.trafficlight.getSubscriptionResults(id))

                self.update_vehicles_position_and_velocity(step, rotateWheels)
                self.update_webots_vehicles(xOffset, yOffset)

            if not self.sumoClosed:
                self.traci.close()
            else:
                self.stop_all_vehicles()

            #csvfile.close()
            sys.stdout.flush()

