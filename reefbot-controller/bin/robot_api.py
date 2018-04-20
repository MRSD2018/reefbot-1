# The purpose of this code is to control the VideoRay robot via the 
# ethernet adapter.  
#
# Author: Michael Furlong
# Date: July 2010
import roslib; roslib.load_manifest('reefbot-controller')
import rospy
from reefbot_msgs.msg import RobotHealth
from reefbot_msgs.msg import RobotStatus
import struct,operator,socket,threading
import sys
import time

#  These values are taken from the file pro4_specific_response.h
#  as given to me (PMF) from Andy Goldstein

RESPONSE_ACK=1 # one byte reply to signify receiving the payload.
RESPONSE_ATTITUDE=2 # eight bytes for heading, pitch, roll (degress) and depth  in mpsi
RESPONSE_WATER_TEMP=3 # full attitude message plus 2 bytes for temp in mC
RESPONSE_HEALTH=4 # ATTITUDE_DEPTH_WATER_TEMP message plus 2 bytes for internal humidity
RESPONSE_CSR_BLOCK= 0x80 # Flag that specifies that we want to read data from the robot's memory

class RobotState:
  '''Class to define the state of the robot.

  comms_ok - Can we talk to the robot?
  heading - From 0-360 degrees
  pitch - In degrees
  roll - In degrees
  depth - In meters
  water_temp - In degrees C
  internal_humidity - ?? Not sure of units
  internal_temp - Internal temperature in degrees C
  total_power - Watts?
  voltage_drop - Volts
  tether_voltage - Volts
  bus_voltage - Voltage of the 12V bus
  bus_current - Current drawn on the 12V bus
  comm_error_count - Number of comm errors the robot identified
  '''
  def __init__(self, comms_ok=False, heading=None, pitch=None, roll=None,
               depth=None, water_temp=None, internal_humidity=None,
               internal_temp=None, total_power=None, voltage_drop=None,
               tether_voltage=None, bus_voltage=None, bus_current=None,
               comm_error_count=None):
    self.comms_ok = comms_ok
    self.heading = heading
    self.pitch = pitch
    self.roll = roll
    self.depth = depth
    self.water_temp = water_temp
    self.internal_humidity = internal_humidity
    self.internal_temp = internal_temp
    self.total_power = total_power
    self.voltage_drop = voltage_drop
    self.tether_voltage = tether_voltage
    self.bus_voltage = bus_voltage
    self.bus_current = bus_current
    self.comm_error_count = comm_error_count

  def setInternalTempMC(self, temp):
    '''Sets the internal temperature when given in degrees mC.'''
    if temp is not None:
      self.internal_temp = temp/1000.

  def setWaterTempMC(self, temp):
    '''Sets the water temperature when given in degrees mC.'''
    if temp is not None:
      self.water_temp = temp/10.

  def WasGoodResponse(self):
    return self.comms_ok

##############################################################################
def mbar_to_meters(pressure,atmospheric=1000.):
  """
  Converts the atmosphere from milliBarrs to depth in metres.
  approximate algorithm given by Andy Goldstein
  @param pressure The sensed pressure in milliBars
  @param atmospheric The atmospheric (above tank) pressure in mBar.
    1 atmosphere is approximately 1 bar.
    approximately 101.9716 milliBar per metre of depth.
  """
  relativePressure = pressure-atmospheric
  if pressure > 1e-4:
    rospy.logerr("The depth sensor is not working.")
    return None
  
  MILLI_BAR_PER_METRE= 101.9716 / 1.7
  return (pressure-atmospheric)/MILLI_BAR_PER_METRE
  psi = float(pressure)/1000. # convert milli-psi to psi
  feet = psi/0.5 # convert psi to depth under water in feet
  return 0.3048*feet # convert feet to metres
  
#######################################################################
def unpack_int16(payload, msb, lsb, base_address=0):
  '''Helper function to unpack a 16bit value from the payload.'''
  # Make sure that we actually have this data in the payload
  rel_msb = msb - base_address
  rel_lsb = lsb - base_address
  if (rel_msb >= len(payload) or rel_lsb >= len(payload) or
      rel_msb < 0 or rel_lsb < 0):
    rospy.logdebug("Addresses (0x%X, 0x%X) was not in the payload" %
                   (msb, lsb))
    return None
    
  return float(struct.unpack('<h', payload[rel_msb]+payload[rel_lsb])[0])

#######################################################################
def unpack_int8(payload, lsb, base_address=0):
  '''Helper function to unpack a 8bit value from the payload.'''
  # Make sure that we actually have this data in the payload
  rel_lsb = lsb - base_address
  if (rel_lsb < 0 or rel_lsb >= len(payload)):
    rospy.logdebug("Addresses (0x%X) was not in the payload" %
                   (lsb))
    return None
  
  return float(ord(payload[rel_lsb]))

######################################################################
def safe_divide(num, den):
  '''Calculate num/den but return None if its not possible.'''
  try:
    return num/den
  except TypeError:
    return None

##############################################################################

def build_request_header(id,flags,address,length):
  header = [0xFA,0xAF,id,flags,address,length]
  header_bytes = map(lambda x: struct.pack("!B",x),header)
  header_string =  reduce(lambda x,y:x+y,header_bytes)
  
  header_string = header_string + pro4_checksum(header_bytes)
  return header_string

#######################################################################
def pro4_checksum(buff):
  return struct.pack("<B",reduce(operator.xor,[ord(b) for b in buff]))

#######################################################################
def write_bytes(msg):
  for b in msg:
    print hex(ord(b)) +' ',
  print "\n"

#######################################################################

class VideoRayAPI:
  MAX_THRUSTER_CMD = 100
  ROV_ID = 1
  
  def __init__(self, flip_thrusters=False, comm_timeout=4.0):
    self.socket = None
    self.socket_lock = threading.Lock()
    self.thrust_target = [0,0,0]
    self.lights = 0
    self.last_depth = '\xFF\xFF'
    self.depth_bytes = '\xFF\xFF'
    self.keep_depth = True 
    self.activate_camera = False
    self.flip_thrusters = flip_thrusters
    self.disable_up_thrust = False
    self.disable_down_thrust = False
    self.disable_ceil = False
    self.ip_address = None
    self.port = None
    self.healthPub = rospy.Publisher(
      rospy.get_param('robot_health_topic', 'robot_health'), RobotHealth, queue_size=10)
    self.robotHealth = RobotHealth(voltage=48.0, router_comms_ok=True,
                              robot_comms_ok=True, left_motor_ok=True,
                              right_motor_ok=True, vertical_motor_ok=True,
                              depth_sensor_ok=True, heading_sensor_ok=True,
                              robot_error_code=0)
    self.statusPub = rospy.Publisher(
      rospy.get_param('robot_status_topic', 'robot_status'), RobotStatus, queue_size=10)
    self.robotStatus = RobotStatus(left_speed=0, right_speed=0, vertical_speed=0,
                              depth = 0,
                              heading = 0, roll = 0, pitch = 0,
                              internal_humidity = 0, water_temp=0, 
                              spin_count = 0,
                              internal_temp=0, total_power=0, voltage_drop=0, tether_voltage =0, bus_voltage=0, bus_current=0, comm_error_count=0)
    self.timeout=comm_timeout

  #######################################################################
  def connect(self,ip_address,port):
    self.ip_address = ip_address
    self.port = port
    connected = False
    while not connected:
      if self.socket is not None:
        self.socket.close()
      self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.socket.settimeout(self.timeout)
      try:
        self.socket.connect((ip_address,port))
        connected = True
        self.robotHealth.robot_comms_ok = True
      except socket.error as socketError:
        if socketError[0] == 4:
          # Interrupted system call (ctrl-c)
          raise
        rospy.logerr("Could not connect to robot at ip %s port %s. "
                     "Retrying: %s" %
                       (ip_address, port, str(socketError)))
        self.robotHealth.robot_comms_ok = False
      except socket.timeout as timeoutError:
        rospy.logerr("Timed out connected to robot. Retrying.")
        self.robotHealth.robot_comms_ok = False
      # mdesnoyer: Not a userful message so disabling
      #self.healthPub.publish(self.robotHealth)

  #######################################################################
  def set_lights(self,value):
    self.lights = value 
  #######################################################################
  def set_velocity(self,percent_port,percent_starboard):
    if self.flip_thrusters:
      self.thrust_target[0] = int(self.MAX_THRUSTER_CMD*percent_starboard)
      self.thrust_target[1] = int(self.MAX_THRUSTER_CMD*percent_port)
    else:
      self.thrust_target[0] = int(self.MAX_THRUSTER_CMD*percent_port)
      self.thrust_target[1] = int(self.MAX_THRUSTER_CMD*percent_starboard)
  #######################################################################
  def set_vertical(self,percent_vertical):
    scale_factor = percent_vertical
    if self.disable_up_thrust:
      scale_factor = -1
    if self.disable_down_thrust:
      scale_factor = 1
    self.thrust_target[2] = int(self.MAX_THRUSTER_CMD*scale_factor)
  #######################################################################
  def depth_keeping(self,do_depth_keeping=True):

    # Don't do depth keeping when we're out of bounds
    if (self.disable_up_thrust or self.disable_down_thrust or
        not self.robotHealth.depth_sensor_ok):
      do_depth_keeping = True

    # If we're already keeping depth, don't update the target
    if self.keep_depth:
      self.keep_depth = do_depth_keeping
      return
      
    self.keep_depth = do_depth_keeping
    if do_depth_keeping:
      self.depth_bytes = self.last_depth
    
  #######################################################################
  def toggle_camera(self,camera_on=False):
    self.activate_camera = camera_on
  #######################################################################
  def send_status_query(self):
    """
    this function is to be used to query the robot for its current state.
    The idea is that this will be repeatedly called in a thread.
    It returns the current robot state in a RobotState object
    '''
    """
    header_string = build_request_header(self.ROV_ID,
                                         RESPONSE_CSR_BLOCK | 0x29,
                                         0x5F,
                                         0x0)
    payload_xsum = '\x00\x00'
    # need to write the strings to the socket.
    msg_string = header_string+payload_xsum
    
    return self.get_response(msg_string)

  #######################################################################
  def send_command(self):
    '''Sends the current command to the robot and returns the robot status.

    return value as a RobotState object.
    '''
    # Put together the command packet
    thrust_bytes = map(lambda x: struct.pack("<h",x),self.thrust_target)
    light_bytes = struct.pack("<B",int(max(min(self.lights,255),0)))
    camera_tilt_focus_bytes = '\x00\x00'
    camera_control_bytes = '\x00\x00'
    if self.activate_camera:
      camera_control_bytes = '\x01\x00'
    auto_depth_bytes = '\xFF\xFF'
    if self.keep_depth:
      auto_depth_bytes = self.depth_bytes
    
    payload_bytes = thrust_bytes 
    payload_bytes.append(light_bytes)
    payload_bytes.append(camera_tilt_focus_bytes)
    payload_bytes.append(camera_control_bytes)
    payload_bytes.append(auto_depth_bytes)

    cmd_payload_string = reduce(lambda x,y:x+y,payload_bytes)
    cmd_payload_xsum = pro4_checksum(cmd_payload_string)
  
    cmd_header_string = build_request_header(self.ROV_ID,RESPONSE_ACK,0x0,
                                             len(cmd_payload_string))

    # need to write the strings to the socket.
    cmd_msg_string = cmd_header_string+cmd_payload_string+cmd_payload_xsum
    cmd_response = self.get_response(cmd_msg_string)
    if cmd_response is None or not cmd_response.WasGoodResponse():
      rospy.logerr("Invalid response to the command: " + str(cmd_response))
      return cmd_response

    # Now query the robot for its status
    return self.send_status_query()
  #######################################################################
  def get_response(self,msg):
    """
    A (theoretically) thread safe way of getting responses from the robot.
    Sends a message and then waits for the response.  This all happens in a lock
    So no thread can co-opt the other.  Responses should be very fast, given
    that the hardware is expecting serial in/out comms.
    """
    response = RobotState(comms_ok=False)
    comm_done = False
    self.socket_lock.acquire()
    try:
      while not comm_done:
        try:
          self.socket.send(msg)
          response = self.parse(self.socket.recv(1024))
          comm_done = True
        except socket.error as socketError:
          if socketError[0] == 4:
            # Interrupted system call (ctrl-c)
            raise
          rospy.logerr("Lost connection to robot. Retrying: " +
                       str(socketError))
          self.robotHealth.robot_comms_ok = False
          # mdesnoyer: Not a userful message
          #self.healthPub.publish(self.robotHealth)
          self.connect(self.ip_address, self.port)
        except socket.timeout as timeoutError:
          rospy.logerr("Timed out sending to robot. "
                       "Trying to reestablish connection")
          self.robotHealth.robot_comms_ok = False
	  # mdesnoyer: Not a useful message
          # self.healthPub.publish(self.robotHealth)
          self.connect(self.ip_address, self.port)
    except Exception as e:
      rospy.logerr("unexpected error: " + str(e))
    finally:
      self.socket_lock.release()
    return response
  
  #######################################################################
  def parse(self,msg):
    '''Parses the message into a RobotState object.
    '''
    
    if not (msg[0] == '\xfd' and msg[1] == '\xdf'):
      rospy.logerr("Received an invalid package %s" % msg)
      return RobotState(comms_ok=False)
    if not len(msg) > 8:
      rospy.logerr("Received incomplete package " + msg)
      return RobotState(comms_ok=False)
    rov_id = ord(msg[2])
    flags = ord(msg[3])
    address = ord(msg[4])
    message_length = ord(msg[5])
    checksum = ord(msg[6])
    device_type = ord(msg[7]) # This is the cause of many off-by-one errors
    # on the part of inattentive programmers, according to Andy.
    # Makes the payload coming back look like it is MSB first order.

    payload = msg[8:]

    robotStatus = RobotState(comms_ok=True)
    robot_status_msg = RobotStatus

    #write_bytes(msg) # writes hex bytes to the display for debugging
    if flags==RESPONSE_ACK:
      return robotStatus
      # no action required, so we don't need it here.  Kept for completeness

    if (flags & RESPONSE_CSR_BLOCK) > 0:
      # We're asking for a data dump of all the status bytes direct from memory.
      robotStatus.heading = unpack_int16(payload, 0x68, 0x69, address)
      robotStatus.pitch = unpack_int16(payload, 0x6A, 0x6B, address)
      robotStatus.roll = unpack_int16(payload, 0x6C, 0x6D, address)
      print "Heading:  ", robotStatus.heading, "  Pitch:  ", robotStatus.pitch, "  Roll:  ", robotStatus.roll
      robotStatus.setWaterTempMC(unpack_int16(payload, 0x7A, 0x7B, address))
      robotStatus.internal_humidity = unpack_int16(payload, 0x84, 0x85,
                                                   address)
      robotStatus.setInternalTempMC(unpack_int16(payload, 0x82, 0x83,address))
      robotStatus.total_power = unpack_int8(payload, 0x5F, address)
      robotStatus.voltage_drop = safe_divide(
        unpack_int8(payload, 0x60, address), 1000.)
      robotStatus.tether_voltage = safe_divide(
        unpack_int16(payload, 0x7C, 0x7D, address), 100.)
      robotStatus.bus_voltage = safe_divide(
        unpack_int16(payload, 0x7E, 0x7F, address), 100.)
      robotStatus.bus_current = safe_divide(
        unpack_int16(payload, 0x80, 0x81, address), 1000.)
      robotStatus.comm_error_count = unpack_int16(payload, 0x86, 0x87,
                                                  address)
      
      robotStatus.depth = unpack_int16(payload, 0x66, 0x67, address)
      # if robotStatus.depth is not None:
      #   self.last_depth = payload[0x66-address]+payload[0x67-address]
      # robotStatus.depth = mbar_to_meters(robotStatus.depth)
      
    
    if flags==RESPONSE_ATTITUDE or flags==RESPONSE_WATER_TEMP or flags==RESPONSE_HEALTH:
      robotStatus.heading = unpack_int16(payload, 0, 1)
      robotStatus.pitch = unpack_int16(payload, 2, 3)
      robotStatus.roll = unpack_int16(payload, 4, 5)
      # self.last_depth = payload[6]+payload[7]
      # robotStatus.depth = mbar_to_meters(unpack_int16(payload, 6, 7))

    if flags==RESPONSE_WATER_TEMP or flags==RESPONSE_HEALTH:
      robotStatus.setWaterTempMC(unpack_int16(payload, 8, 9))

    if message_length==RESPONSE_HEALTH:
      robotStatus.internal_humidity = unpack_int16(payload, 10, 11)

    # Publish the health of the robot
    self.robotHealth.depth_sensor_ok = robotStatus.depth is not None
    # Not a useful message
    # self.healthPub.publish(self.robotHealth)

    # if robotStatus.depth is None:
    #   robotStatus.depth = 1.8 # Safe value for now

    #Publish Robot Status
    robot_status_msg.left_speed = -self.thrust_target[0]
    robot_status_msg.right_speed = -self.thrust_target[1]
    robot_status_msg.vertical_speed = abs(self.thrust_target[2])


    robot_status_msg.heading = robotStatus.heading
    robot_status_msg.pitch = robotStatus.pitch
    robot_status_msg.roll  = robotStatus.roll
    robot_status_msg.internal_humidity = robotStatus.internal_humidity
    robot_status_msg.water_temp = robotStatus.water_temp
    robot_status_msg.internal_temp = robotStatus.internal_temp
    robot_status_msg.total_power = robotStatus.total_power
    robot_status_msg.voltage_drop = robotStatus.voltage_drop
    robot_status_msg.tether_voltage = robotStatus.tether_voltage
    robot_status_msg.bus_voltage = robotStatus.bus_voltage
    robot_status_msg.bus_current = robotStatus.bus_current
    robot_status_msg.comm_error_count = robotStatus.comm_error_count
    robot_status_msg.depth = unpack_int16(payload, 0x66, 0x67, address)
    self.statusPub.publish(self.robotStatus)

    
    
    return robotStatus
  #############################################################################i
  def disable_up(self,disable):
    '''
    Used to prevent swimming up when you\'ve reached the minimum depth.
    '''
    if self.disable_ceil:
      self.disable_up_thrust = False
      return
    
    self.disable_up_thrust = disable
  #############################################################################i
  def disable_down(self,disable):
    '''
    Used to prevent swimming down when you\'ve reached the maximum depth.
    Mainly to prevent pesting the eels.
    '''
    if self.disable_ceil:
      self.disable_down_thrust = False
      return
    
    self.disable_down_thrust = disable
    
#########################################################################
  def disable_ceiling(self, disable):
    '''If true, change the functionality of disable_down so it does not work.

    This is used to allow the super user to get the robot back up
    without pulling on the tether.
    '''
    self.disable_ceil = disable


