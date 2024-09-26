#include <random>
#include "../../include/PointSelector.h"

// This function is just an interface to the extract hgp function: extract_hgp_grid_kinect(...)
void IDNav::PointSelector::extract_hgp(vecPt& hgp_ ,
                                       dataType& imgGradient_,
                                       const Mat& grayImg_,
                                       const Mat& depthImg_,
                                       const Mat& maskImg_) {
#ifdef COUT_COMPLETE_PIPELINE
    hgpExtractionProfiler.begin();
#endif
    extract_hgp_grid_kinect(hgp_, imgGradient_, grayImg_, depthImg_, maskImg_,
                            imageGrid, numPointsToExtract, maxDepthKinect, minPhotoGradient);
#ifdef COUT_COMPLETE_PIPELINE
    hgpExtractionProfiler.end();
#endif
}

// This function extracts hgp (high gradient pixels) using a grid and depth kinect information
void IDNav::PointSelector::extract_hgp_grid_kinect(vecPt& hgp_,
                                                   dataType& imgGradient_,
                                                   const Mat& grayImg_,
                                                   const Mat& depthImg_,
                                                   const Mat& maskImg_,
                                                   const ImageGrid& imageGrid_,
                                                   const int num_hgpToExtract_,
                                                   const float& maxDepthKinect_,
                                                   const float& GminForHgpExtract0
                                                   ){
    //Function Parameters
    const static int minSearchAreaSize0{4};

    // Check inputs
    if (grayImg_.channels() != 1){
        std::cout << BOLDRED_COUT << "[PointSelector] extract_hgp_grid_kinect() : Image is not grayscale"
                  << RESET_COUT << std::endl;
        terminate();
    }

    cv::Size imgSize = grayImg_.size();
    if ((imgSize.width != imageGrid_.w) || (imgSize.height != imageGrid_.h)) {
        std::cout << BOLDRED_COUT << "[PointSelector] extract_hgp_grid_kinect() : Image dimensions have changed : " << imgSize <<
                  " != [" << imgSize.width << " x " <<imgSize.height << "]"
                  << RESET_COUT << std::endl;
        terminate();
    }

    imgSize = depthImg_.size();
    if ((imgSize.width != imageGrid_.w) || (imgSize.height != imageGrid_.h)) {
        std::cout << BOLDRED_COUT << "[PointSelector] extract_hgp_grid_kinect() : Depth image dimensions have changed : " << imgSize <<
                  " != [" << imgSize.width << " x " <<imgSize.height << "]"
                  << RESET_COUT << std::endl;
        terminate();
    }

    imgSize = maskImg_.size();
    if ((imgSize.width != imageGrid_.w) || (imgSize.height != imageGrid_.h)) {
        std::cout << BOLDRED_COUT << "[PointSelector] extract_hgp_grid_kinect() : Mask image dimensions have changed : " << imgSize <<
                  " != [" << imgSize.width << " x " <<imgSize.height << "]"
                  << RESET_COUT << std::endl;
        terminate();
    }

    if(depthImg_.type() != DEPHT_MAT_TYPE){
        std::cout << BOLDRED_COUT <<  "[PointSelector] extract_hgp_grid_kinect() : Depth image is not CV_32F : type " << depthImg_.type() <<
                " != " << DEPHT_MAT_TYPE
                << RESET_COUT << std::endl;
        terminate();
    }

    // Init Variables
    hgp_.clear();
    Mat mask{maskImg_.clone()};
    imgGradient_ = 0.0;

    // Compute gradient img with bilinear interpolation
    Mat grad_u{}, grad_v{};
    float gradientScale{}, pixelBias{};
    gradFunction(grad_u, grad_v, gradientScale, pixelBias, grayImg_,1);

    Mat G{cv::abs(grad_u) + cv::abs(grad_v)};

    cv::Scalar mean_G_tmp{}, stdDev_G_tmp{};
    cv::meanStdDev(G, mean_G_tmp, stdDev_G_tmp);
    float threshold_G_img{float(mean_G_tmp[0])};
    if (threshold_G_img*gradientScale < GminForHgpExtract0) threshold_G_img = GminForHgpExtract0/gradientScale;

    // Extract hgp all across the image
    float mean_G_cell{}, stddev_G_cell{}, threshold_G_cell{};
    int pointValidity{}, maxPointValidity{}; // Values between [0,10] = [minVal, maxVal]
    int uiFix{}, viFix{}, uRef{}, vRef{};
    float depthi{}, depthRef{};
    bool addPt{},validCell{};
    short Gi{}, Gmax{};

    std::unordered_map<std::shared_ptr<GridElement>, vector<ptTemp> > hgp_map{};
    int numCells = 0;
    int numPointsFound = 0;
    for (const std::shared_ptr<GridElement> &cell: imageGrid_.grid) {
        validCell = false;
        // Compute a gradient threshold for each image cell in the grid
        cv::meanStdDev(G(cv::Range(cell->vi, cell->vf), cv::Range(cell->ui, cell->uf)), mean_G_tmp, stdDev_G_tmp);
        mean_G_cell = (float) mean_G_tmp[0];
        stddev_G_cell = (float) stdDev_G_tmp[0];
        // Iterate decreasing the gradient threshold
        for(int num_stdDev_G{3}; num_stdDev_G >= 0; --num_stdDev_G){
            threshold_G_cell = mean_G_cell + float(num_stdDev_G) * stddev_G_cell; // Gthreshold = mean(G) + num_stdDev_G * stdDev(G)
            // Iterate through each cell in patches of size = minSearchAreaSize
            for (int u0{cell->ui}; u0 <= cell->uf; u0 += minSearchAreaSize0) {
                for (int v0{cell->vi}; v0 <= cell->vf; v0 += minSearchAreaSize0) {
                    Gmax = 0;
                    maxPointValidity = 0;
                    addPt = false;
                    uiFix = u0 + minSearchAreaSize0;
                    viFix = v0 + minSearchAreaSize0;
                    // Grab the point with the highest gradient of each patch
                    for (int ui{u0}; (ui < uiFix) && (ui <= cell->uf); ++ui) {
                        for (int vi{v0}; (vi < viFix) && (vi <= cell->vf); ++vi) {
                            pointValidity = int(mask.at<uchar>(vi, ui));
                            if (pointValidity != 0) {// Check if the point is static
                                Gi =  G.at<short>(vi, ui);
                                if(float(Gi) > threshold_G_img){
                                    if (float(Gi) > threshold_G_cell) { // Check if the gradient is high enough
                                        if (pointValidity*Gi > maxPointValidity*Gmax) { // Check if it is the highest gradient in the area
                                            if (thereIsDepthKinect(depthi, ui, vi, depthImg_,MAX_DEPTH_VALUE)) {
                                                uRef = ui;
                                                vRef = vi;
                                                depthRef = depthi;
                                                Gmax = Gi;
                                                maxPointValidity = pointValidity;
                                                addPt = true;
                                                validCell = true;
                                            } else mask.at<uchar>(vi, ui) = 0;
                                        }
                                    }
                                }else mask.at<uchar>(vi, ui) = 0;
                            }
                        }
                    }
                    // Add found point
                    if (addPt) {
                        ptTemp pt_aux{uRef, vRef, depthRef, Gmax};
                        hgp_map[cell].push_back(pt_aux);
                        imgGradient_ += gradientScale * (float)Gmax;
                        ++numPointsFound;
                        for (int uP{}; uP < 2; ++uP) {
                            for (int vP{}; vP < 2; ++vP) {
                                mask.at<uchar>(vRef + vP, uRef + uP) = 0;
                            }
                        }
                    }
                }
            }
            if(validCell) {
                ++numCells;
                break;
            }
        }
    }
    imgGradient_ /= (float) numPointsFound;
    if (numPointsFound < 64) {
        throw PointSelector::PointSelectorFail{"[Point Selector](extract hgp grid) : Image not textured enough or incorrect depth information"};
    }

    // Filter points to have a reduce set of size = num_hgpToExtract_
    vector<ptTemp> hgpTemp{};
    {
        int numPointsExcess = numPointsFound - num_hgpToExtract_;
        if(numPointsExcess > 10){
            const float percPointsExcess{float(numPointsExcess)/float(numPointsFound)};
            int numPointsToSelectPerCell{};
            for (auto &hgp_cell: hgp_map) {
                int numPointsToRemove = int(percPointsExcess*hgp_cell.second.size());
                numPointsToSelectPerCell = hgp_cell.second.size() - numPointsToRemove;
                filterAndAddPoints(hgpTemp, hgp_cell.second, numPointsToSelectPerCell);
            }
            if(hgpTemp.size() > num_hgpToExtract_){
                vector<ptTemp> hgpTempTemp{hgpTemp};
                hgpTemp.clear();
                filterAndAddPoints(hgpTemp, hgpTempTemp, num_hgpToExtract_);
            }
        }else{
            for (auto &hgp_cell: hgp_map) {
                for (int i{0}; i < hgp_cell.second.size(); ++i) {
                    hgpTemp.push_back(hgp_cell.second[i]);
                }
            }
        }
    }

    // Register magnitude and direction of the photometric gradient in the point objects
    {
        for (int i{0}; i < hgpTemp.size()  ;++i){
            typePt pt = createPt(hgpTemp[i].uRef + pixelBias,hgpTemp[i].vRef + pixelBias, hgpTemp[i].depthRef);

            pt->Gu_ref = gradientScale * (dataType) grad_u.at<short>(hgpTemp[i].vRef, hgpTemp[i].uRef);
            pt->Gv_ref = gradientScale * (dataType) grad_v.at<short>(hgpTemp[i].vRef, hgpTemp[i].uRef);
            pt->G_ref = sqrt(pow(pt->Gu_ref ,2) + pow(pt->Gv_ref,2));

            pt->dirRef = vecX::Zero(2);
            pt->dirRef[0] = pt->Gu_ref/pt->G_ref;
            pt->dirRef[1] = pt->Gv_ref/pt->G_ref;
            hgp_.push_back(pt);
            if(pt->uRef < 4){
                cout << "pt->uRef = " << pt->uRef << endl;
                cout << "pt->vRef = " << pt->vRef << endl;
                terminate();
            }
        }
    }
}

void IDNav::PointSelector::gradFunction(cv::Mat &grad_x_,
                                        cv::Mat &grad_y_,
                                        float& gradientScale_,
                                        float& pixelBias_,
                                        const cv::Mat& grayImg_,
                                        const int method_ ){
    switch (method_) { // "Bilinear interpolation";
        case 0:
        {
            // I  = I00(1-x)(1-y) + I10(x)(1-y) + I01(1-x)(y) + I11(xy)
            // Gx(x = 0.5, y = 0.5) = -I00(1-y) + I10(1-y) - I01(y)   + I11(y) = 0.5(I11 - I00) - 0.5(I01 - I10)
            // Gy(x = 0.5, y = 0.5) = -I00(1-x) - I10(x)   + I01(1-x) + I11(x) = 0.5(I11 - I00) + 0.5(I01 - I10)
            //        -------------
            //        | IOO | I10 |
            //        | I01 | I11 |
            //        -------------
            Mat grayImage{};
            grayImg_.convertTo(grayImage, CV_16SC1);
            int w = grayImage.cols;
            int h = grayImage.rows;
            Mat I1 = grayImage( cv::Rect( 1, 1, w-1, h-1))
                     - grayImage( cv::Rect( 0, 0, w-1, h-1));
            Mat I2 = grayImage( cv::Rect( 0, 1, w-1, h-1))
                     - grayImage( cv::Rect( 1, 0, w-1, h-1));
            grad_x_ = I1 - I2;
            grad_y_ = I1 + I2;

            gradientScale_ = 0.5f;
            pixelBias_ = 0.5f;
            break;
        }
        case 1: { // "Scharr"
            Scharr(grayImg_, grad_x_, CV_16S, 1, 0, 1.0, 0, cv::BORDER_DEFAULT);
            Scharr(grayImg_, grad_y_, CV_16S, 0, 1, 1.0, 0, cv::BORDER_DEFAULT);
            gradientScale_ = (1.0f/16.0f);
            pixelBias_ = 0.0f;
            break;
        }
    }
}

// This function picks "numPoints_" equally distributed of "pointsIn_" and add them to "pointsOut_"
void IDNav::PointSelector::filterAndAddPoints(vector<ptTemp>& pointsOut_, const vector<ptTemp>& pointsIn_, const int& numPoints_){
    if(pointsIn_.size() <= numPoints_) {
        for (int i{0}; i < numPoints_; ++i) {
            pointsOut_.push_back(pointsIn_[i]);
        }
    }else{
        float selectRate = float(pointsIn_.size() - 1) / float(numPoints_);
        int ptIndex = 0;
        for (int i{0}; i < numPoints_; ++i) {
            pointsOut_.push_back(pointsIn_[ptIndex]);
            ptIndex = (int) roundf(float(i) * selectRate);
        }
    }
}