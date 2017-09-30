#ifndef _DISPLAY_IMAGES_H_
#define _DISPLAY_IMAGES_H_

#include <string>
#include <opencv2/core/core.hpp>
#include <vector>

namespace cv_utils {

// Displays an image to a UI window on the screen. First normalizes it
// to the 0-255 range.
void DisplayNormalizedImage(const cv::Mat& image,
                            const char* windowName="image");

// Displays an LAB image on the screen converted back to RGB
void DisplayLabImage(const cv::Mat& image, const char* windowName);

// Displays an image with some bounding boxes overlayed
void DisplayImageWithBoxes(const cv::Mat& image,
                           const std::vector<cv::Rect>& boxes,
                           const char* windowName="image");

// Displays all the windows created by DisplayNormalizedImage until a
// key is pressed on the window.
void ShowWindowsUntilKeyPress();

} // namespace 

#endif // _DISPLAY_IMAGES_H_
