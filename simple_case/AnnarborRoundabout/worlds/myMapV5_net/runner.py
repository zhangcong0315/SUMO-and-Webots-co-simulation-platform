from __future__ import absolute_import
from __future__ import print_function


import os
import subprocess
import sys
import shutil
import asyncio

try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

sumoBinary = checkBinary('sumo')

# run simulation
retcode = subprocess.call(
    [sumoBinary, "-c", "./sumo.sumocfg", "--no-step-log"], stdout=sys.stdout, stderr=sys.stderr)
print(">> Simulation closed with status %s" % retcode)
sys.stdout.flush()

# start the simulation and connect to it with the script: 
import traci
#import compute_density
import parse

out_file = './netstate_out.xml'
net_file = './sumo.net.xml'
(data,vehicleList) = parse.parse_out(out_file)

step_length = 0.1
traci.start([checkBinary('sumo-gui'), '-c',"./sumo.sumocfg", '--step-length', str(step_length)])

vehicles = []
flag = 0
start_vel = 5
step = 0
stopList = set()
changeLane = {}
aggrVehicle = set()

# set up the start speed of each vehicle
for v in vehicleList:

    Id = traci.vehicle.getTypeID(str(v))
    if Id == "AggrCar":
        aggrVehicle.add(v)
        traci.vehicle.setSpeedMode(v, 0)
        traci.vehicle.setSpeed(v, start_vel)
        traci.vehicle.setLaneChangeMode(v, 0)

step = 1
# Run a simulation until all vehicles have arrived
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    step += 1
    
print(step)
traci.close()