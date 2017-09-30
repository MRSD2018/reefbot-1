#!/usr/bin/python
'''Program that pings for the robot and if its found, turns on the console'''

import subprocess
import os.path
import sys
import re
import os
import signal
import time
import math

ROBOT_IP = "192.168.1.2"
PINGS_NEEDED = 5
CONSOLE_EXECUTABLE = ["roslaunch",
                      "/home/reefbot/src/reefbot/ros/reefbot.launch"]
ROS_DIR = "/home/reefbot/src/reefbot/ros"

def DoICMPPing(timeout=5.0):
  '''Executes an ICMP ping with the camera and returns true if it passed.'''
  retval = False
  try:
    retval =  subprocess.call(["ping", '-c', '1', '-W',
                               str(math.ceil(timeout)),
                               ROBOT_IP],
                              stdout=open('/dev/null'),
                              stderr=subprocess.STDOUT)
  except Exception as e:
    print str(e)
  return retval == 0

def IsRobotOn():
  '''Determines if the robot is on by looking for consecutive pings either failing or suceeding.'''
  pingCount = 0
  while abs(pingCount) < PINGS_NEEDED:
    if DoICMPPing():
      pingCount = pingCount + 1
    else:
      pingCount = pingCount - 1

  return pingCount > 0

def TurnOnConsole():
  print 'Turning On Console'
  proc = subprocess.Popen(CONSOLE_EXECUTABLE)
  return proc

def TurnOffConsole(rosProc):
  print 'Turning Off Console'
  rosProc.terminate()
  rosProc.wait()
  return None

def SetROSPath():
  os.environ['ROS_PACKAGE_PATH'] = '%s:%s' % (os.environ['ROS_PACKAGE_PATH'],
                                              ROS_DIR)

if __name__ == '__main__':
  SetROSPath()
  proc = None
  while True:
    try:
      if proc is None and IsRobotOn():
        proc = TurnOnConsole()
      elif proc is not None and not IsRobotOn():
        proc = TurnOffConsole(proc)  

    except Exception as e:
      print str(e)

    time.sleep(10) 
