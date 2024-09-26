
#ifndef IDNAV_DEFINESANDINCLUDES_H
#define IDNAV_DEFINESANDINCLUDES_H
/////////////////////////////////////////////////////////////////////////////////// options
#define PLANE_ESTIMATION
#define ONLINE_WINDOW_OPTIMIZATION
#ifndef ONLINE_WINDOW_OPTIMIZATION
//#define SEQUENTIAL_WINDOW_OPTIMIZATION
#endif
#define GEO_DEPTH_OPTIMIZATION
//#define SEMANTIC_SEGMENTATION                 // NOT AVAILABLE YET
//#define ON_ORIENTED_MASK                      // NOT AVAILABLE YET
#define MAX_DEPTH_VALUE 4.0
/////////////////////////////////////////////////////////////////////////////////// Features
#define ACTIVE_FEATURES
//#define DEACTIVATE_HGP
#define DESCRIPTOR_SIZE 61
#ifdef ACTIVE_FEATURES
#define MATCH_HAMMING_RADIUS 121.5f /* 1/4 of the descriptor size */
#define AKAZE_THRESHOLD_MAX 0.001f
#define AKAZE_THRESHOLD_MIN 0.00005f
#define AKAZE_NUM_POINTS 500
    #define ACTIVE_FEATURES_TRACKING
    #define ACTIVE_LC
#endif
//////////////////////////////////////////////////////////////////////////////// Debug
//#define COUT_PIPELINE_TIMING
#ifdef COUT_PIPELINE_TIMING
//#define COUT_COMPLETE_PIPELINE
//#define COUT_OPTIMIZER_PIPELINE
#endif
//#define COUT_OPTIMIZER_PIPELINE
//#define OPTIMIZER_DEBUG
//#define COMPLETE_PROFILING
/////////////////////////////////////////////////////////////////////////////////// settings
#define PATCH_SIZE 9
#define PYR_LEVELS_MAX 4
#define DEPHT_MAT_TYPE CV_32F
#define DEPHT_VALUE_TYPE float
namespace IDNav{
    extern double maxLambdaVis;
    extern double minLambdaVis;
    extern double minCosVis;
}
//////////////////////////////////////////////////////////////////////////////////// includes
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <map>

#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

#include <pangolin/pangolin.h>

#include <boost/algorithm/string.hpp>

#include "glog/logging.h"

#include "ceres/ceres.h"
#include "ceres/problem.h"
#include "ceres/rotation.h"
#include "ceres/evaluation_callback.h"

#include "fast_akaze/features2d_akaze2.hpp"
#include "fast_akaze/akaze/AKAZEFeatures.h"

#include "LCDetector.h"

/////////////////////////////////////////////////////////////////////////////////// data types
using namespace std;
using namespace Eigen;
using namespace cv;
//using namespace ceres;

namespace IDNav
{
    typedef double dataType;
    typedef Eigen::MatrixXd matX;
    typedef Eigen::MatrixXi matXi;

    typedef Eigen::VectorXd vecX;
    typedef Eigen::Matrix3d mat3;
    typedef Eigen::Vector3d vec3;
    typedef Eigen::Matrix<dataType, 9, 1> vec9;

    typedef Eigen::Matrix<dataType, 2, 1> vec2;
    typedef Eigen::Matrix<dataType, 4, 1> vec4;
    typedef Eigen::Matrix<dataType, 7, 1> vec7;
    typedef Eigen::Matrix<dataType, 2, 2> mat2;
    typedef Eigen::Matrix<dataType, 6, 1> vec6;
    typedef Eigen::Matrix<dataType, 4, 4> mat4;
    typedef Eigen::Matrix<dataType, 6, 6> mat6;
    typedef Eigen::Matrix<dataType, 8, 1> vec8;
    typedef Eigen::Matrix<dataType, 8, 8> mat8;
    typedef Eigen::Matrix<dataType, 1,6> row6;
    typedef Eigen::Matrix<dataType, 2,6> row26;
    typedef Eigen::Matrix<dataType, 2,3> mat23;
    typedef Eigen::Matrix<dataType, 3,2> mat32;
    typedef Eigen::Matrix<dataType, 2,1> mat21;
    typedef Eigen::Matrix<dataType, 1,2> mat12;
    typedef Eigen::Matrix<dataType, 2,6> mat26;
    typedef Eigen::Matrix<dataType, 1,6> mat16;
    typedef Eigen::Matrix<dataType, 3,4> mat34;

    typedef Eigen::Matrix<dataType, 1, PATCH_SIZE> rowPatch;
    typedef Eigen::Matrix<dataType, PATCH_SIZE ,1> colPatch;
    typedef Eigen::Matrix<float, 4, 4> mat4_float;

    typedef vector<Mat> vecImg;
    typedef vector<cv::KeyPoint> vecKeyPt;
}

///////////////////////
#define RESET_COUT   "\033[0m"
#define BLACK_COUT   "\033[30m"      /* Black */
#define RED_COUT     "\033[31m"      /* Red */
#define GREEN_COUT   "\033[32m"      /* Green */
#define BLUE_COUT   "\033[34m"      /* Green */
#define BOLDBLACK_COUT   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED_COUT     "\033[1m\033[31m"      /* Bold Red */
#define BOLDMAGENTA_COUT "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDYEL_COUT  "\033[1m\033[33m"         /* Bold Yellow */
#define YEL_COUT  "\x1B[33m"
#define CYAN_COUT  "\x1B[36m"
#define LIGTH_YEL_COUT  "\x1B[93m"
#define LIGTH_GREEN_COUT  "\x1B[92m"
#define LIGTH_RED_COUT  "\x1B[91m"


/*FG_DEFAULT = 39,
FG_BLACK = 30,
FG_RED = 31,
FG_GREEN = 32,
FG_YELLOW = 33,
FG_BLUE = 34,
FG_MAGENTA = 35,
FG_CYAN = 36,
FG_LIGHT_GRAY = 37,
FG_DARK_GRAY = 90,
FG_LIGHT_RED = 91,
FG_LIGHT_GREEN = 92,
FG_LIGHT_YELLOW = 93,
FG_LIGHT_BLUE = 94,
FG_LIGHT_MAGENTA =95,
FG_LIGHT_CYAN = 96,
FG_WHITE = 97,
BG_RED = 41,
BG_GREEN = 42,
BG_BLUE = 44,
BG_DEFAULT = 49`*/


#endif //IDNAV_DEFINESANDINCLUDES_H
