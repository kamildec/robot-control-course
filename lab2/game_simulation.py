#!/usr/bin/env python3

import pybullet as p
import numpy as np
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
p.setGravity(0, 0, -10)

# load our robot definition
robot = p.loadURDF("robot.urdf")

cubes_num = 4
cubes = []
for i in range(cubes_num):
    pos = random.random() * 1.8 - 0.9, random.random() * 1.8 - 0.9, 0.1
    cubes.append(p.loadURDF("cube.urdf", pos, globalScaling = 0.05))
    p.changeVisualShape(cubes[i], -1, rgbaColor=[1,0.5,0.7,1])

# display info about robot joints
numJoints = p.getNumJoints(robot)
joints = []
for joint in range(numJoints):
    joints.append(p.getJointInfo(robot, joint))
    print(joints[joint])

# add four sliders to GUI
p0_id = p.addUserDebugParameter("z", -0.1, 0, 0)
p1_id = p.addUserDebugParameter("y", -1, 1, 0)
p2_id = p.addUserDebugParameter("x", -1, 1, 0)
p3_id = p.addUserDebugParameter("pos", 0.0, 6.28, 0)

p.stepSimulation()


def is_this_winning_position(cube, eps):
    win = True
    cubePos, cubeRot = p.getBasePositionAndOrientation(cube)
    if abs(cubePos[0]) > eps or abs(cubePos[1]) > eps:
        p.changeVisualShape(cube, -1, rgbaColor=[1, 0.5, 0.7, 1])
        win = False
    elif abs(cubePos[0]) < eps and abs(cubePos[1]) < eps:
        p.changeVisualShape(cube, -1, rgbaColor=[0, 177, 106, 1])
    return win


def find_first_loosing(cubes, eps):
    loosing = -1
    for i in range(len(cubes)):
        if not is_this_winning_position(cubes[i], eps):
            loosing = i

    return loosing


def go_to_position(robot, cube, eps, axis, target, max_vel=1):
    p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, target[0], maxVelocity=max_vel)
    p.setJointMotorControl2(robot, 1, p.POSITION_CONTROL, target[1], maxVelocity=max_vel)
    p.setJointMotorControl2(robot, 2, p.POSITION_CONTROL, target[2], maxVelocity=max_vel)
    while not np.isclose(p.getJointState(robot, axis)[0], target[axis], atol=1e-5):
        p.stepSimulation()
        time.sleep(0.01)
    is_this_winning_position(cube, eps)


eps = 0.25
eps2 = 0.1

in_the_air = True
# wait until all cubes are on the ground
while in_the_air:
    tmp = False
    for i in range(cubes_num):
        cubePos, cubeRot = p.getBasePositionAndOrientation(cubes[i])
        if cubePos[2] > 0:
            tmp = True
    in_the_air = tmp
    p.stepSimulation()
    time.sleep(0.01)

while True:

    i = find_first_loosing(cubes, eps)
    if i == -1:
        time.sleep(1)
        break

    cubePos, cubeRot = p.getBasePositionAndOrientation(cubes[i])
    sign = 1 if cubePos[0] > 0 else -1

    if abs(cubePos[0]) > eps:
        z = 0.
        y = cubePos[1]
        x = cubePos[0] + sign * eps2

        go_to_position(robot, cubes[i], eps, 2, [z, y, x])
        z = -0.08
        go_to_position(robot, cubes[i], eps, 0, [z, y, x], 10)

        x = sign * (eps - eps2)
        go_to_position(robot, cubes[i], eps, 2, [z, y, x])

        z = 0.
        go_to_position(robot, cubes[i], eps, 0, [z, y, x], 10)

    cubePos, _ = p.getBasePositionAndOrientation(cubes[i])
    sign = 1 if cubePos[1] > 0 else -1

    if abs(cubePos[1]) > eps:
        z = 0.
        y = cubePos[1] + sign * eps2
        x = cubePos[0]

        go_to_position(robot, cubes[i], eps, 1, [z, y, x])
        z = -0.08
        go_to_position(robot, cubes[i], eps, 0, [z, y, x], 10)

        y = sign * (eps - eps2)
        go_to_position(robot, cubes[i], eps, 1, [z, y, x])
        z = 0.
        go_to_position(robot, cubes[i], eps, 0, [z, y, x], 10)

    p.stepSimulation()
    time.sleep(0.01)  # sometimes pybullet crashes, this line helps a lot