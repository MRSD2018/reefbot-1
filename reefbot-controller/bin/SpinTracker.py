'''Module that keeps track of the total spin of the robot.

This is necessary so that we can unwind the robot.
'''
import math

class SpinTracker:
  # Constants for how much spinning has been done
  IN_BOUNDS = 0
  IN_TIGHT_BOUNDS = 1
  NEEDS_STARBOARD = -2  # We need to spin to starboard to unwind
  NEEDS_PORT = -1 # We need to spin to port to unwind
  
  def __init__(self, spinLimit):
    '''Initialize the spin limit in degrees.'''
    self.spinCount = 0.0 # In total degrees
    self.lastHeading = None
    self.spinLimit = spinLimit
    self.tightLimit = 90

  def updateHeading(self, heading):
    if heading is None:
      return
    
    # See if we've gone over the 360 degree limit
    if self.lastHeading is None:
      self.lastHeading = heading
      return
      
    diff = heading - self.lastHeading
    if diff > 90.0:
      diff = diff - 360.0
    elif diff < -90.0:
      diff = diff + 360.0

    self.spinCount = self.spinCount + diff
    self.lastHeading = heading

  def resetHeading(self):
    self.spinCount = 0.0

  def getSpinCount(self):
    return self.spinCount

  def getSpinState(self):
    '''Returns constants about which way we need to turn.'''
    if self.spinCount > self.spinLimit:
      return SpinTracker.NEEDS_PORT

    if self.spinCount < -self.spinLimit:
      return SpinTracker.NEEDS_STARBOARD

    if abs(self.spinCount) < self.tightLimit:
      return SpinTracker.IN_TIGHT_BOUNDS

    return SpinTracker.IN_BOUNDS

  def isInBounds(self):
    return (self.getSpinState() == SpinTracker.IN_BOUNDS or
            self.getSpinState() == SpinTracker.IN_TIGHT_BOUNDS)

  def isDoneUnwind(self):
    return self.getSpinState() == SpinTracker.IN_TIGHT_BOUNDS

  def getSpinDirection(self):
    if self.spinCount < 0:
      return SpinTracker.NEEDS_STARBOARD

    return SpinTracker.NEEDS_PORT

  
      
