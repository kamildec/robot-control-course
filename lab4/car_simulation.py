#!/usr/bin/env python3

from numpy.lib.function_base import angle
import pybullet as p
import pybullet_data
import math
import random
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

def make_boxes():
  pos_1 = [random.random(), random.random(), 0]
  angle = 3.1415 * 2 * random.random()
  pos_2 = [pos_1[0] + math.sin(angle) * 0.6, pos_1[1] - math.cos(angle) * 0.6, 0]
  return pos_1, pos_2

def build_world_with_car(pos):
  p.resetSimulation()
  p.setGravity(0, 0, -10)
  p.loadURDF("plane.urdf")
  car = p.loadURDF("racecar/racecar.urdf")
  p.resetBasePositionAndOrientation(car, pos[0], pos[1])
  return car

def simulate_car(car):
  inactive_wheels = [3, 5, 7]
  wheels = [2]
  for wheel in inactive_wheels:
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
  steering = [4, 6]
  maxForce = 10
  targetVelocity = -2
  steeringAngle = 0.174
  steps = 5000
  for wheel in wheels:
    p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=maxForce)

  for steer in steering:
    p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)
  for i in range(steps):
     p.stepSimulation()
  return p.getBasePositionAndOrientation(car)

start_pose = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
car = build_world_with_car(start_pose)
end_pose = simulate_car(car)
while True:
  pos_1, pos_2 = make_boxes()

  y_dif = pos_2[1] - pos_1[1]
  x_dif = pos_2[0] - pos_1[0]

  angle_trans_radians = -math.atan(y_dif/x_dif)
  transform_matrix = np.array([[np.cos(angle_trans_radians), np.sin(angle_trans_radians), 0], [-np.sin(angle_trans_radians), np.cos(angle_trans_radians), 0], [0, 0, 1]])
  trace = np.array(end_pose[0]) - np.array(start_pose[0])
  calculated_end_pos = (np.array(pos_2) + np.array(pos_1)) / 2
  transformed_coordinates = np.dot(transform_matrix, -trace)
  calculated_end_pos += transformed_coordinates

  # rot = p.getEulerFromQuaternion(start_pose[1])
  rot = (0, 0, -angle_trans_radians)
  rot = p.getQuaternionFromEuler(rot)
  print(angle_trans_radians * 180 / np.pi)
  # TODO: calculate calculated_pose
  calculated_pose = (calculated_end_pos, rot)

  # The car should end its route with the front wheels between two boxes as in the example

  car = build_world_with_car(calculated_pose)
  p.loadURDF("cube.urdf", pos_1, globalScaling = 0.1)
  p.loadURDF("cube.urdf", pos_2, globalScaling = 0.1)
  simulate_car(car)
  time.sleep(5)