import math

import coppeliasim_zmqremoteapi_client as zmq

client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

cuboid = sim.getObjectHandle("/Cuboid")
pose = sim.getObjectPose(cuboid, -1)

print(pose)

sim.setObjectPose(cuboid, -1, [0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0])

pose = sim.getObjectPose(cuboid, -1)
print(pose)

initial_time = sim.getSimulationTime()

while True:
    current_time = sim.getSimulationTime()
    time = (current_time - initial_time) % 100.0
    print(math.sin(time))

    sim.setObjectPose(cuboid, -1, [0.0, 0.0, math.sin(time / 5.0), 0.0, 0.0, 0.0, 1.0])