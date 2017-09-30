#!/usr/bin/python
'''
Finds the human face in the image that is closest to the center.

Input message: ImageCaptured
Output message: SpeciesIDRequest

Author: Mark Desnoyer
Date: Dec 2010
'''
import roslib; roslib.load_manifest('reefbot')
import rospy
from reefbot_msgs.msg import ImageCaptured
from reefbot_msgs.msg import SpeciesIDRequest
from reefbot_msgs.msg import ImageRegion
from sensor_msgs.msg import RegionOfInterest
import cv
import math
from cv_bridge import CvBridge

class ObjectFinder:
  def __init__(self, classifierFilename):
    self.classifier = cv.Load(classifierFilename)
    self.bridge = CvBridge()

  def FindObjects(self, image):
    '''Finds all the instances of the object in the image.

    Inputs:
    image - ROS image message.

    Outputs:
    List of (x,y,w,h) of the bounding boxes for the objects.
    '''
    cvImage = self.bridge.imgmsg_to_cv(image, "mono8");
    return cv.HaarDetectObjects(cvImage, self.classifier,
                                cv.CreateMemStorage(),
                                flags=cv.CV_HAAR_DO_CANNY_PRUNING)
    


def callback(msg, publisher, objectFinder):
  objects = objectFinder.FindObjects(msg.image)

  # Find the object closest to the center of the image
  imageCenter = (msg.image.width/2, msg.image.height/2)
  bestObject = None
  dist = float('Infinity')
  for (x,y,w,h),n in objects:
    objCenter = (x+w/2, y+h/2)
    objDist = (math.pow(imageCenter(0) - objCenter(0), 2) +
               math.pow(imageCenter(1) - objCenter(1), 2))
    if objDist < dist:
      dist = objDist
      bestObject = RegionOfInterest(x,y,w,h)

  if bestObject is not None:
    region = ImageRegion(bounding_box = bestObject)
    request = SpeciesIDRequest(image_id=msg.image_id, image=msg.image,
                               regions=[region])
    request.header.stamp = rospy.Time.now()
    publisher.publish(request)

if __name__ == '__main__':
  rospy.init_node('FaceFinder')

  publisher = rospy.Publisher(
    rospy.get_param('species_id_request_topic', 'request_species_id'),
    SpeciesIDRequest,
    tcp_nodelay=True)

  objectFinder = ObjectFinder(
    rospy.get_param('~object_classifier',
                    'haarcascade_frontalface_default.xml'))

  
  cb = lambda x: callback(x, publisher, objectFinder)
  
  rospy.Subscriber(rospy.get_param('still_image_topic',
                                   'still_image'),
                   ImageCaptured,
                   cb)

  rospy.spin()
