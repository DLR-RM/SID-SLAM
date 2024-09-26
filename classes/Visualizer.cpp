#include "../include/Visualizer.h"



void IDNav::MapVisualizer::setTwc(pangolin::OpenGlMatrix& Twc_ ,const std::shared_ptr<IDNav::Frame>& frame){
    Twc_.SetIdentity();
    {
        std::mutex mMutex;
        unique_lock<mutex> lock(mMutex);
        Twc_(0,0) = frame->pose.Rwc(0,0);
        Twc_(0,1) = frame->pose.Rwc(0,1);
        Twc_(0,2) = frame->pose.Rwc(0,2);
        Twc_(1,0) = frame->pose.Rwc(1,0);
        Twc_(1,1) = frame->pose.Rwc(1,1);
        Twc_(1,2) = frame->pose.Rwc(1,2);
        Twc_(2,0) = frame->pose.Rwc(2,0);
        Twc_(2,1) = frame->pose.Rwc(2,1);
        Twc_(2,2) = frame->pose.Rwc(2,2);
        Twc_(0,3) = frame->pose.twc(0);
        Twc_(1,3) = frame->pose.twc(1);
        Twc_(2,3) = frame->pose.twc(2);
    }

}

void IDNav::MapVisualizer::drawTrajectory(){

    glLineWidth(3.5);
    glColor3f(1.0f,0.2f,1.0f);

    glBegin(GL_LINES);
    for(int i_pt{1}; i_pt < trajectory.size(); ++i_pt){
        glVertex3f(trajectory[i_pt].X,trajectory[i_pt].Y,trajectory[i_pt].Z);
        glVertex3f(trajectory[i_pt-1].X,trajectory[i_pt-1].Y,trajectory[i_pt-1].Z);
    }
    if(!groundtruth->empty()){
        glLineWidth(3.5);
        glColor3f(0.2f,1.0f,0.2f);
        glBegin(GL_LINES);
        for(int i_pt{1}; i_pt < trajectory.size(); ++i_pt){
            auto it = (*groundtruth)[i_pt];
            glVertex3f(it.twc(0),it.twc(1),it.twc(2));
            it = (*groundtruth)[i_pt-1];
            glVertex3f(it.twc(0),it.twc(1),it.twc(2));
        }
    }

    glEnd();
    glPopMatrix();
}

void IDNav::MapVisualizer::drawMapPoints(){
    glPointSize(12.0);
    glColor4f(1.0, 0.3900, 0.1176,1.0);
    glBegin(GL_POINTS);
    if(drawFeatures){
        for(IDNav::Point3D& pt: trackedFeatures){
            glVertex3f(pt.X,pt.Y,pt.Z);
        }
    }

    glColor4f(0.0, 0.5364,0.8892,1.0);
    if(drawHgp){
        for(IDNav::Point3D& pt: trackedPoints){
            glVertex3f(pt.X,pt.Y,pt.Z);
        }
    }
    glEnd();
}

void IDNav::MapVisualizer::drawGraph() {
    for (GraphEdge &edge: graphEdges)
        drawEdge(edge, edgeColor, 3.0);
}

void IDNav::MapVisualizer::drawLocalWindow(){
    //drawCamera(Twc_ref, refKeyColor, 0.75, 6.0);
    for (pangolin::OpenGlMatrix &Twc_: Twc_localMapKeyframes) {
        drawCamera(Twc_, optWindowColor, 0.75, 6.0);
    }
}

void IDNav::MapVisualizer::drawOptimizationWindow(){
    //drawCamera(Twc_ref, refKeyColor, 0.75, 6.0);
    for (pangolin::OpenGlMatrix &Twc_: Twc_keyframesToOptimize) {
        drawCamera(Twc_, optWindowColor, 0.75, 6.0);
    }
}
void IDNav::MapVisualizer::drawTrackingWindow(){
    drawCamera(Twc_ref, localWindowColor, 0.75, 6.0);
    /*for (pangolin::OpenGlMatrix &Twc_: Twc_trackingKeyframes) {
        drawCamera(Twc_, trackWindowColor, 0.75, 6.0);
    }*/
}
void IDNav::MapVisualizer::drawKeyframeCameras() {
    for (pangolin::OpenGlMatrix &Twc_: Twc_keyframes) {
        drawCamera(Twc_, keyframeColor, 0.25, 3.0);
        //drawCamera(Twc_, keyframeColor, 0.75, 12.0);
    }
}
void IDNav::MapVisualizer::drawEdge(GraphEdge& edge, GLfloat color[4], dataType width){
    glLineWidth(width);
    glColor4f(color[0],color[1],color[2],color[3]);

    glBegin(GL_LINES);
    glVertex3f(edge.X1,edge.Y1,edge.Z1);
    glVertex3f(edge.X2,edge.Y2,edge.Z2);
    glEnd();

    glPopMatrix();
}

void IDNav::MapVisualizer::drawCamera(pangolin::OpenGlMatrix& Twc_, GLfloat color[4], dataType scale, dataType width){

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc_.m);
#endif

    //glLineWidth(mCameraLineWidth);

    glLineWidth(width);
    glColor4f(color[0],color[1],color[2],color[3]);

    dataType w_ = w*scale;
    dataType h_ = h*scale;
    dataType z_ = z*scale;

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w_,h_,z_);
    glVertex3f(0,0,0);
    glVertex3f(w_,-h_,z_);
    glVertex3f(0,0,0);
    glVertex3f(-w_,-h_,z_);
    glVertex3f(0,0,0);
    glVertex3f(-w_,h_,z_);

    glVertex3f(w_,h_,z_);
    glVertex3f(w_,-h_,z_);

    glVertex3f(-w_,h_,z_);
    glVertex3f(-w_,-h_,z_);

    glVertex3f(-w_,h_,z_);
    glVertex3f(w_,h_,z_);

    glVertex3f(-w_,-h_,z_);
    glVertex3f(w_,-h_,z_);
    glEnd();

    glPopMatrix();
}

void IDNav::MapVisualizer::drawPoints(std::vector<IDNav::Point3D>& hgp,std::vector<IDNav::Point3D>& features,
                cv::Mat& image){

    int recThickness = 3*image_w/640;
    int sizeRect = 10*image_w/640;

    int circleThickness = 2*image_w/640;
    int circleRadius = 8*image_w/640;

    int centerThickness = 2*image_w/640;
    int centerRadius = (2*image_w/640);

    cv::Scalar color_aux = color_hgp;
    float factor;
    if(drawHgp){
        for(IDNav::Point3D& pt: hgp) {
            Point pt0 = Point(pt.u, pt.v);
            color_aux = color_hgp;
            factor = (maxCov_hgp-pt.cov)/(maxCov_hgp - minCov_hgp);
            if(factor < 0.0f) factor = 0.0f;
            color_aux *= 1.0;//factor;

            circle(image,pt0,centerRadius, color_aux, centerThickness,0);
            circle(image,pt0,circleRadius, color_aux, circleThickness,0);
        }
    }
    if(drawFeatures){
        for(IDNav::Point3D& pt: features) {
            Point pt0 = Point(pt.u, pt.v);
            /*if(pt.reUsedFeature){
                circle(image, pt0, centerRadius, 1.5*color_features, centerThickness);
                Point pt1 = Point(pt.u - sizeRect, pt.v - sizeRect);
                Point pt2 = Point(pt.u + sizeRect, pt.v + sizeRect);
                rectangle(image, pt1, pt2, 1.5*color_features, recThickness);
            }else{*/
            circle(image, pt0, centerRadius, color_features, centerThickness);
            Point pt1 = Point(pt.u - sizeRect, pt.v - sizeRect);
            Point pt2 = Point(pt.u + sizeRect, pt.v + sizeRect);
            rectangle(image, pt1, pt2, color_features, recThickness);
            //}
        }
    }
}

