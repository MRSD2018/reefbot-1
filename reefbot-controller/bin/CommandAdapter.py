import math 

class CommandAdapter:
  def __init__(self,max_thrust,max_rotate):
    '''
    Takes a max forward truster value and a max turning thruster value.
    '''
    self.max_thrust = math.fabs(max_thrust)
    self.max_rotate = math.fabs(max_rotate)

  def get_command(self,x,y):
    '''
    Performs a simple dot-product resolution, them clamps the 
    values to be within the maximum thruster commands. 
    '''
    # This will send you arc-ing to the right (starboard) as you 
    # move the joystick to the right.
    # and vice-versa
    starboard = self.clamp(x*self.max_thrust + y*self.max_rotate) 
    port = self.clamp(x*self.max_thrust - y*self.max_rotate)
    return (port,starboard)

  def clamp(self,value):
    return max(min(value,self.max_rotate),-self.max_rotate)
