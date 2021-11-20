#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import time
import random

# start the simulation with a GUI (p.DIRECT is without GUI)
p.connect(p.GUI)

# we can load plane and cube from pybullet_data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# load a plane
p.loadURDF("plane.urdf", [0, 0, -0.1], useFixedBase=True)

#setup gravity (without it there is no gravity at all)
p.setGravity(0,0,-10)

# load our robot definition
robot = p.loadURDF("robot.urdf")

# load a cube
# cube_1 = p.loadURDF("cube.urdf", [0.3, -1, 0.1], globalScaling = 0.05)
# p.changeVisualShape(cube_1, -1, rgbaColor=[1,0.5,0.7,1])

cubes_num = 4
cubes = []
for i in range(cubes_num):
  pos = random.random() * 2 - 1, random.random() * 2 - 1, 0.1
  cubes.append(p.loadURDF("cube.urdf", pos, globalScaling = 0.05))
  p.changeVisualShape(cubes[i], -1, rgbaColor=[1,0.5,0.7,1])

# display info about robot joints
numJoints = p.getNumJoints(robot)
for joint in range(numJoints):
  print(p.getJointInfo(robot, joint))

# add four sliders to GUI
p0_id = p.addUserDebugParameter("z", -0.1, 0, 0)
p1_id = p.addUserDebugParameter("y", -1, 1, 0)
p2_id = p.addUserDebugParameter("x", -1, 1, 0)
p3_id = p.addUserDebugParameter("pos", 0.0, 6.28, 0)

p.stepSimulation()

while True:
  # set joint parameters (we can control position, velocity, acceleration, force, and other)
  p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, p.readUserDebugParameter(p0_id))
  p.setJointMotorControl2(robot, 1, p.POSITION_CONTROL, p.readUserDebugParameter(p1_id))
  p.setJointMotorControl2(robot, 2, p.POSITION_CONTROL, p.readUserDebugParameter(p2_id))
  p.setJointMotorControl2(robot, 3, p.POSITION_CONTROL, p.readUserDebugParameter(p3_id))
  robotPos, robotRot = p.getBasePositionAndOrientation(robot)
  print(robotPos)
  # step Simulation
  p.stepSimulation()
  time.sleep(0.01) # sometimes pybullet crashes, this line helps a lot
