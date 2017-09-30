#!/usr/bin/python
"""Sends a message that is read by the StillCaptureModule."""
import roslib; roslib.load_manifest('reefbot')
import rospy
from reefbot_msgs.msg import ImageCaptured

if __name__ == "__main__": 
  rospy.init_node('TestStillCaptureModule')

  publisher = rospy.Publisher(
    rospy.get_param('still_image_topic', 'still_image'),
    ImageCaptured,
    tcp_nodelay=True, latch=False)

  request = ImageCaptured()
  request.image_id = 23
  request.image.height = 16
  request.image.width = 20
  request.image.data = range(16*20)
  request.image.encoding = "8UC1"
  request.image.step = 20*4


  r = rospy.Rate(1)
  while not rospy.is_shutdown():
    publisher.publish(request)
    r.sleep()
