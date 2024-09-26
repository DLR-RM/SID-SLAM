
#include "../../include/Frame.h"


bool IDNav::Frame::loadImage(Mat& rgbImg_, Mat& grayImg_, dataType& grayImgTs_,
                             Mat& depthImage_, dataType& validDepth_,
                             size_t& id_,
                             Mat& maskImg_) {
    frameId = id_;
    grayImgTs = grayImgTs_;
    pose.ts = grayImgTs;
    validDepth = validDepth_;

    depthImg = depthImage_;
    maskImg = maskImg_;

    cvtColor(rgbImg_,grayImg_,cv::COLOR_RGB2GRAY);
    grayImgDist = grayImg_.clone();
    cam->readAndUndistortImage(grayImg_);
    Mat_gray_clone = grayImg_.clone();

    return true;
}