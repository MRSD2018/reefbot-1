import numpy as np
import math
import scipy.ndimage as ndimage

image = np.zeros((20,30), np.float64)

for i in range(3,14):
  image[i, (i+10)/2] = i / 20.

for i in range(9,18):
  image[6, i] = 0.6

angle = 1.0
desiredTransform = np.array(((math.cos(angle*math.pi/180.),
                            math.sin(angle*math.pi/180),
                            0),
                            (-math.sin(angle*math.pi/180.),
                            math.cos(angle*math.pi/180),
                            0)), np.float64)



                                                           
warpedImage = ndimage.affine_transform(image, desiredTransform[0:2,0:2],
                                       desiredTransform[0:2,2],
                                       output_shape=(20,30),
                                       order=1)

works = False;
