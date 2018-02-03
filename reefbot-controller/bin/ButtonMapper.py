'''Maps buttons for the Reefbot control.'''

import roslib; roslib.load_manifest('reefbot-controller')
import rospy

class JoystickButtons:
  DPAD_LR = 4
  DPAD_UD = 5
  
  ALOG_LEFT_UD = 1
  ALOG_LEFT_LR = 0

  ALOG_RIGHT_UD = 3
  ALOG_RIGHT_LR = 2

  BUTTON_1 = 0
  BUTTON_2 = 1
  BUTTON_3 = 2
  BUTTON_4 = 3
  BUTTON_5 = 4
  BUTTON_6 = 5
  BUTTON_7 = 6
  BUTTON_8 = 7
  BUTTON_9 = 8
  BUTTON_10 = 9
  BUTTON_11 = 10
  BUTTON_12 = 11

class ButtonMapper:
  def __init__(self):
    self.diveDownButton = rospy.get_param("~dive_down_button",
                                          JoystickButtons.BUTTON_7)
    self.diveUpButton = rospy.get_param("~dive_up_button",
                                        JoystickButtons.BUTTON_5)
    self.diveAxis = rospy.get_param("~dive_axis", None)
    self.leftTurnButton = rospy.get_param("~left_turn_button", None)
    self.rightTurnButton = rospy.get_param("~right_turn_button", None)
    self.turnAxis = rospy.get_param("~turn_axis",
                                    JoystickButtons.ALOG_LEFT_LR)
    self.fwdButton = rospy.get_param("~fwd_button", None)
    self.backButton = rospy.get_param("~back_button", None)
    self.fwdBackAxis = rospy.get_param("~fwd_back_axis",
                                       JoystickButtons.ALOG_LEFT_UD)

  def GetFwdAxis(self, joyMsg):
    '''Returns the value of the move fwd/backward axis. +1 is full forward.'''
    return self._GetAxisValue(joyMsg, self.fwdBackAxis, self.fwdButton,
                              self.backButton)

  def GetTurnAxis(self, joyMsg):
    '''Returns the value of the turning axis. +1 is full left.'''
    return self._GetAxisValue(joyMsg, self.turnAxis, self.leftTurnButton,
                              self.rightTurnButton)

  def GetDiveAxis(self, joyMsg):
    '''Returns the value of the dive axis. +1 is full up.'''
    return self._GetAxisValue(joyMsg, self.diveAxis, self.diveUpButton,
                              self.diveDownButton)

  def _GetAxisValue(self, joyMsg, axis, posButton, negButton):
    if axis is not None and axis >= 0:
      return joyMsg.axes[axis]

    axisVal = 0.
    if joyMsg.buttons[posButton] and not joyMsg.buttons[negButton]:
      axisVal = 1.
    if not joyMsg.buttons[posButton] and joyMsg.buttons[negButton]:
      axisVal = -1.

    return axisVal

  def GetCeilingDisable(self, joyMsg):
    '''Returns the value of the button that disables the ceiling.'''
    return self._GetButtonValue(joyMsg, JoystickButtons.BUTTON_10)

  def _GetButtonValue(self, joyMsg, button):
    return joyMsg.buttons[button]
