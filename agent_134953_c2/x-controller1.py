"""c2_controller"""

from controller import Robot
import math
from collections import deque


# ROBOT DEFINITIONS
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot.

def setup_robot():
  robot = Robot()
  timestep = int(robot.getBasicTimeStep())
  gps = robot.getDevice('gps')
  camera = robot.getDevice('camera')
  compass = robot.getDevice('compass')
  left_motor = robot.getDevice('left wheel motor')
  right_motor = robot.getDevice('right wheel motor')

  gps.enable(timestep)
  camera.enable(timestep)
  compass.enable(timestep)
  left_motor.setPosition(float('inf'))
  right_motor.setPosition(float('inf'))
  
  return robot



def main():
  # TODO
  # 1. configure robot, initial position and next nodes
  robot = setup_robot()
  timestep = int(robot.getBasicTimeStep())
  
  global initial_coordinates
    
  while robot.step(timestep) != -1:
    # 2. While there are nodes to explore:
      # a. Control robot to next node
          #   i. STRAIGTH = motors
          #  ii. TURN
          # iii. STOP
          #  iv. LOST
      # b. Camera: use to anticipate turn and mark node as goal (straight)
      # c. If GS0 and GS2 see black, add to next nodes to stack
      # d. If all GSx see white = dead end
      # e. add to next nodes to stack
      # f. Pop next node to explore 
      # g. find shortest path to next node
            

main()