#!/usr/bin/python
'''
Identifies the entire frame as the fish to identify. This is a placeholder.

Input message: ImageCaptured
Output message: SpeciesIDRequest

Author: Mark Desnoyer
Date: Oct 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
from reefbot_msgs.msg import ImageCaptured
from reefbot_msgs.msg import SpeciesIDRequest
from reefbot_msgs.msg import ImageRegion
from sensor_msgs.msg import RegionOfInterest

def callback(msg, publisher):
  roi = RegionOfInterest(0, 0, msg.image.height, msg.image.width)
  region = ImageRegion(bounding_box = roi)
  request = SpeciesIDRequest(image_id=msg.image_id, image=msg.image,
                             regions=[region])
  request.header.stamp = rospy.Time.now()
  publisher.publish(request)

if __name__ == '__main__':
  rospy.init_node('NOPFishFinder')

  publisher = rospy.Publisher(
    rospy.get_param('~species_id_request_topic', 'request_species_id'),
    SpeciesIDRequest,
    tcp_nodelay=True)
  cb = lambda x: callback(x, publisher)
  
  rospy.Subscriber(rospy.get_param('~still_image_topic',
                                   'still_image'),
                   ImageCaptured,
                   cb)

  rospy.spin()
