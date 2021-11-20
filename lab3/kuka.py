#!/usr/bin/env python3

import math
import random
from time import sleep

import pybullet as p
import pybullet_data


# Helper functions for later
def setCubeColor(cube, color=None):
    if color is None:
        color = [1, 0.5, 0.7, 1]
    p.changeVisualShape(cube, -1, rgbaColor=color)


def loadCube(pos, size=0.05, color=None):
    print(f"Creating cube at {[f'{x:.2f}' for x in pos]}.")
    ret = p.loadURDF("cube.urdf", pos, globalScaling=size)
    setCubeColor(ret, color)
    return ret


# Basic setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])
p.setGravity(0, 0, -10)

# This time we use kuka from pybullet_data
kukaId = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")[0]
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])

# In our robot, end effector link has following index:
KUKA_TIP_IDX = 6

# We will control this number of joints:
KUKA_NUM_JOINTS = 7

JOINTS_NUM = p.getNumJoints(kukaId)
for i in range(KUKA_NUM_JOINTS, JOINTS_NUM):
    p.setJointMotorControl2(bodyIndex=kukaId,
                            jointIndex=i,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=500)
    print(i)



def move(poses):
    # Choose effector orientation (rotation)
    rpyDeg = [0, 180, 0]
    rot = p.getQuaternionFromEuler([math.radians(deg) for deg in rpyDeg])

    # IK calculates joint positions from effector position and orientation
    jointPos = p.calculateInverseKinematics(kukaId,
                                            KUKA_TIP_IDX,
                                            poses,
                                            rot)

    # Take calculated joint positions and set as setpoints
    for i in range(KUKA_NUM_JOINTS):
        p.setJointMotorControl2(bodyIndex=kukaId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPos[i],
                                targetVelocity=0,
                                force=500)

    # Do a few "dry" simulation loops
    # This gives robot some time to reach requested setpoints
    # This is essentially just a delay, do you know how to do it better? And why?
    for _ in range(400):
        p.stepSimulation()
        sleep(0.01)


cubes = []
pos = 0.6
cubes.append(loadCube([pos, pos, 0.1]))
cubes.append(loadCube([-pos, pos, 0.1]))
cubes.append(loadCube([pos, -pos, 0.1]))

eps = 0.2
eps2 = 0.1
while True:
    # Choose effector position
    move([pos, pos+eps2, 0.3])
    move([pos, pos-2*eps2, 0.3])
