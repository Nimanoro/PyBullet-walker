import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)


cubeStartPos = [0, 0, 1]
cubeId = p.loadURDF("r2d2.urdf", cubeStartPos)

for i in range(240):
    p.stepSimulation()
    time.sleep(1. / 240.)

p.disconnect()