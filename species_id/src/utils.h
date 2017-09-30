#include <stdio.h>
#include "cv.h"
#include "highgui.h"

using namespace cv;

namespace reefbot {
// Writes an image to a file but first normalizes it to the 0-255 range
void WriteNormalizedImage(const string filename, const Mat& image);

}
