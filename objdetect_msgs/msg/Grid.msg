# Specifies a  w,h,x,y dense grid
# The starting points for the location search
uint32 minX
uint32 minY

# The strides in the location space
uint32 strideX
uint32 strideY

# The starting points for the scaling
uint32 minW
uint32 minH

# The strides in the w, h space. In this case, we step by growing by a
# fraction, so that width_i is round(minWidth*strideW^i)
float64 strideW
float64 strideH

# True if the width and height should be a consistent aspect ratio that are 
# defined by minW and minH. This reduces the grid to (s,x,y)
bool fixAspect