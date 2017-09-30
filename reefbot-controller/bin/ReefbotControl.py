#!/usr/bin/env python

import roslib; roslib.load_manifest('reefbot-controller')
import rospy
import copy
import thread
import threading
from joy.msg import Joy
from reefbot_msgs.msg import RobotStatus
from reefbot_msgs.msg import CameraPower
from robot_api import *
from CommandAdapter import CommandAdapter
from SpinTracker import SpinTracker
from ButtonMapper import ButtonMapper

EMPTY_JOY = Joy(axes=[0.0, 0.0, 0.0, 0.0, 0.0],
                buttons=[0,0,0,0,0,0,0,0,0,0,0])

def joystick_handler(data, video_ray_api, command_adapter, pub_signal,
                     thruster_limit, robot_status, thread_signal,
                     spin_tracker, button_mapper, last_joy_msg, min_depth,
                     max_depth, max_spin_time):
  thread_signal.acquire()
  
  # Record the Joystick message
  if len(last_joy_msg) > 0:
    last_joy_msg.pop()
  last_joy_msg.append(data)
  
  try:
    vertical_thrust = button_mapper.GetDiveAxis(data)
    video_ray_api.depth_keeping(abs(vertical_thrust) < 0.1)

    #TODO(mdesnoyer) Commenting this to avoid thrusters spinning while
    #on bench
    video_ray_api.set_vertical(vertical_thrust)
    
    (port_thrust,starboard_thrust) = command_adapter.get_command(
      button_mapper.GetFwdAxis(data),
      button_mapper.GetTurnAxis(data))
    
    #TODO(mdesnoyer) Commenting this to avoid the thursters spinning on the bench
    video_ray_api.set_velocity(port_thrust*thruster_limit,starboard_thrust*thruster_limit)

    # TODO(mdesnoyer): Commenting this out so that we can't control the lights
    #if data.axes[JoystickButtons.DPAD_UD] == 1.:
    #  video_ray_api.set_lights(video_ray_api.lights+1)

    #if data.axes[JoystickButtons.DPAD_UD] == -1.:
    #  video_ray_api.set_lights(video_ray_api.lights-1)

    
    video_ray_api.disable_ceiling(button_mapper.GetCeilingDisable(data))
  
    robot_status.left_speed = port_thrust
    robot_status.right_speed = starboard_thrust
    robot_status.vertical_speed = float(video_ray_api.thrust_target[2])
    robot_status.spin_count = spin_tracker.getSpinCount()

    status = video_ray_api.send_command()
    if not status.WasGoodResponse():
      return
    publish_status(pub_signal, status,robot_status)

    # Using this location as a hook to take away control from user 
    # to either untwist the tether or to prevent them from going above or 
    # below a certain depth.
    # This probably doesn't belong here, but it's the best place for now.
    video_ray_api.disable_up(status.depth < min_depth)
    video_ray_api.disable_down(status.depth > max_depth)

    spin_tracker.updateHeading(status.heading)
     
    # if number of twists > limit, then actively unwind until you
    # are okay again.
    if not spin_tracker.isInBounds():
      unwind_robot(pub_signal, video_ray_api, robot_status, spin_tracker,
                   max_spin_time)
      # Put an empty joystick message on the stack to stop motion
      del last_joy_msg[:]
      last_joy_msg.append(EMPTY_JOY)
    
  finally:
    thread_signal.release()

def safe_float(num, default=-1.):
  '''Returns the number as a float or default if it cannot be converted.'''
  try:
    return float(num)
  except TypeError:
    return default

def publish_status(pub_signal, robot_response, robot_status):
  if not robot_response.WasGoodResponse():
    return

  robot_status.heading = safe_float(robot_response.heading)
  robot_status.pitch = safe_float(robot_response.pitch)
  robot_status.roll = safe_float(robot_response.roll)
  robot_status.water_temp = safe_float(robot_response.water_temp)
  robot_status.internal_humidity = safe_float(robot_response.internal_humidity)
  robot_status.internal_temp = safe_float(robot_response.internal_temp)
  robot_status.total_power = safe_float(robot_response.total_power)
  robot_status.voltage_drop = safe_float(robot_response.voltage_drop)
  robot_status.tether_voltage = safe_float(robot_response.tether_voltage)
  robot_status.bus_voltage = safe_float(robot_response.bus_voltage)
  robot_status.bus_current = safe_float(robot_response.bus_current)
  robot_status.comm_error_count = safe_float(robot_response.comm_error_count)

  # For depth default to 3.3 so that the thruster doesn't go down
  if robot_response.depth is None:
    robot_status.depth = 3.3
  else:
    robot_status.depth = safe_float(robot_response.depth) 

  robot_status.header.stamp = rospy.Time.now()

  pub_signal.acquire()
  pub_signal.notify()
  pub_signal.release()

# Function that polls the robot's state periodically by simulating the
# same joystick command running again.
def status_poll_loop(pub_status, video_ray_api, last_joy_msg, joystick_cb):
  r = rospy.Rate(2)
  while not rospy.is_shutdown():
    if len(last_joy_msg) > 0:
      joystick_cb(last_joy_msg.pop())
    r.sleep()

# A function to act in a thread and send the robot status out
# independently of the comms to the robot itself. We need this because
# sometimes the publish will block if the receiver has died and left
# its socket open.
def status_pub_loop(pub, robot_status, pub_signal):
  while not rospy.is_shutdown():
    pub_signal.acquire()
    pub_signal.wait()

    msg = copy.deepcopy(robot_status)
    pub_signal.release()
    
    pub.publish(msg)    

def camera_power_handler(camera_msg, video_ray_api):
  video_ray_api.toggle_camera(camera_msg.turn_camera_on)
  video_ray_api.send_command()

def unwind_robot(pub_signal, video_ray_api, robot_status, spin_tracker,
                 max_spin_time):
  rospy.logwarn("Unwinding robot")
  timeStarted = rospy.Time.now()
  while (not spin_tracker.isDoneUnwind() and
         rospy.Time.now() < timeStarted + max_spin_time):
    if spin_tracker.getSpinDirection() == SpinTracker.NEEDS_PORT:
      port_thrust = -1.0
      starboard_thrust = 1.0
    else:
      port_thrust = 1.0
      starboard_thrust = -1.0

    # Set the turning motion
    video_ray_api.set_velocity(port_thrust, starboard_thrust)
    robot_status.left_speed = port_thrust
    robot_status.right_speed = starboard_thrust
    robot_status.spin_count = spin_tracker.getSpinCount()

    # Set the vertical motion
    video_ray_api.depth_keeping(True)
    robot_status.vertical_speed = 0

    # Fire off the command to turn the bot
    status = video_ray_api.send_command()
    publish_status(pub_signal, status, robot_status)

    if status.WasGoodResponse():
      spin_tracker.updateHeading(status.heading)

  # Now send a command to stop the turning
  video_ray_api.set_velocity(0, 0)
  robot_status.left_speed = 0
  robot_status.right_speed = 0
  robot_status.spin_count = spin_tracker.getSpinCount()
  status = video_ray_api.send_command()
  publish_status(pub_signal, status, robot_status)

  if not spin_tracker.isDoneUnwind():
    spin_tracker.resetHeading()

  rospy.logwarn("Done unwinding robot")

def listener():
  robot_status = RobotStatus()
  thread_signal = threading.Semaphore(1)
  pub_signal = threading.Condition()
  rospy.init_node('robot_controller',anonymous=True)

  host = rospy.get_param("~serial_host", "192.168.1.2")
  port = rospy.get_param("~serial_port", 100)
  timeout = rospy.get_param("~timeout",1.0)

  max_forward_thrust = rospy.get_param("~max_forward_thrust",0.5)
  max_rotation_thrust = rospy.get_param("~max_rotation_thrust",1.0)
  max_depth = rospy.get_param("~max_depth", 8.0)
  min_depth = rospy.get_param("~min_depth", 0.0)
  thruster_limit = rospy.get_param("~thruster_limit", 0.7)
  flip_thrusters = rospy.get_param("~flip_thrusters", True)
  spin_limit = rospy.get_param("~spin_limit", 3600) # In degrees
  max_spin_time = rospy.Duration.from_sec(
    rospy.get_param("~max_spin_time", 30.0)) # In seconds

  video_ray_api = VideoRayAPI(flip_thrusters, comm_timeout=timeout)
  rospy.loginfo("Connecting to " + host + ":"+str(port))
  video_ray_api.connect(host,port)  
  
  pub = rospy.Publisher(rospy.get_param('robot_status_topic', 'robot_status'),
                        RobotStatus)

  command_adapter = CommandAdapter(max_forward_thrust,max_rotation_thrust)

  spin_tracker = SpinTracker(spin_limit)

  button_mapper = ButtonMapper()

  last_joy_msg = []

  # Subscribe to the joystick messages
  callback = lambda x: joystick_handler(x, video_ray_api, command_adapter,
                                        pub_signal, thruster_limit,
                                        robot_status,
                                        thread_signal, spin_tracker,
                                        button_mapper,
                                        last_joy_msg, min_depth,
                                        max_depth, max_spin_time) 
  rospy.Subscriber(rospy.get_param('joystick_topic', 'joy'), Joy,
                   callback, queue_size=1)

  # Subscribe to CameraPower messages to toggle the camera power
  cameraPowerCallback = lambda x: camera_power_handler(x, video_ray_api)
  rospy.Subscriber(rospy.get_param('camera_power_topic', 'camera_power'),
                   CameraPower,
                   cameraPowerCallback,
                   queue_size=1)

  # Start a thread to poll the robot's state periodically
  poll_loop = lambda x,y:status_poll_loop(x, y, last_joy_msg, callback)
  thread.start_new_thread(poll_loop, (pub_signal, video_ray_api))

  # Start a new thread to publish the robot status
  thread.start_new_thread(status_pub_loop, (pub, robot_status, pub_signal))
  
  rospy.spin()
  video_ray_api.toggle_camera(False) # probably won't reach here, 

if __name__=="__main__":
  listener()
