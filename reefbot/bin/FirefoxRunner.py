#!/usr/bin/python
'''Small utility that runs an instance of firefox in full screen at a screen position.

Author: Mark Desnoyer(markd@cmu.edu)
Date: Oct 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
import Xlib.display
import subprocess

if __name__ == '__main__':
  rospy.init_node('FirefoxRunner')
  
  # URL to display
  url = rospy.get_param("~url", "")

  # Name of the diplay as defined in the html file
  title = '%s - Mozilla Firefox' % rospy.get_param("~title", "")

  # Coordinates and size to display the screen
  x = rospy.get_param("~x", 0)
  y = rospy.get_param("~y", 0)
  width = rospy.get_param("~width", 1920)
  height = rospy.get_param("~height", 1080)

  # The xscreen to display the window on
  xscreen = rospy.get_param("~xscreen", "0.0")

  # The firefox profile to use. Each x screen needs a different profile
  profile = rospy.get_param("~profile", "default")

  # Delay time in seconds
  rospy.sleep(rospy.get_param("~delay", 0.0))

  # Open the firefox window
  proc = subprocess.Popen(['/usr/bin/firefox', '-height', str(height),
                           '-P', profile, '--display=:'+xscreen,
                           '-width', str(width), '-new-window', url])

  try:

    # Now talk to X and find the firefox window
    #disp = Xlib.display.Display(':0')
    #rootWin = disp.screen().root
    #browserWin = None
    #for child in rootWin.query_tree().children:
    #  if child.get_wm_name() == title:
        # Found the window
    #    browserWin = child
    #    break

    # Finally, move the window
    # TODO(mdesnoyer): This won't work because
    # firefox won't have created the window yet. So fix this.
    #browserWin.configure(x=x, y=y, width=width, height=height)


    # And wait for firefox to die, it hopefully won't
    if proc.wait() != 0:
      rospy.logerr('There was a problem with Firefox and it died')

  finally:
    proc.terminate()
    proc.wait()

  rospy.signal_shutdown('The Firefox window was closed')
  
  
