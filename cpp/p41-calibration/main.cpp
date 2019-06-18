#ifndef RAI_OPENCV
#define RAI_OPENCV 1
#endif
#ifndef RAI_ROS
#define RAI_ROS 1
#endif
#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>

#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <RosCom/baxter.h>

#include <Operate/robotOperation.h>

#include <Kin/cameraview.h>
#include "board.h"







/*

void Vision::onlinetracking(cv::Mat input_img){
    // save and set current joint state
    arr joint_state = KinModel::instance().B->getJointPositions();
    K.setJointState(joint_state);
    
    // calc world coordinates of the ball
    arr y,J;
    K.evalFeature(y,J,FS_position,{"calibL"});

    // calc position and depth of the ball in the image
    arr pt = globalPointToCameraPoint(y);
    arr res1 = cameraPointToImgCoords(pt);
    arr res2 = cameraPointToImgCoords({pt(0)+.02,pt(1),pt(2)});

    double radius = sqrt(pow(res1(0)-res2(0),2) + pow(res1(1)-res2(1),2));
    Board circle(res1,radius);


    // Return if ball is not within the image
    if(res1(0) < 0 || res1(0) > input_img.cols ||
        res1(1) < 0 || res1(1) > input_img.rows ||
        res1(2) <= 0){
        return;
    }



    //Drawing output:
    cv::circle(input_img, cv::Point(res1(0),res1(1)), radius, cv::Scalar(0,0,255), 4);


    
    // find three nearest neighbours in the image
    std::vector<Board> ret;


    ret = findNNObjects(input_img,circle,3);



    
}
*/



arr cameraPointToImgCoords(arr pt, arr Fxypxy){
    pt(0) = pt(0)*Fxypxy(0) / -pt(2) + Fxypxy(2);
    pt(1) = pt(1)*Fxypxy(1) / pt(2) + Fxypxy(3);
    pt(2) = -pt(2);

    return pt;
}

arr globalPointToCameraPoint(arr pt, arr T){

    arr y_homo = {pt(0), pt(1),pt(2), 1.};

    arr y_cam = T*y_homo;

    pt(0) = y_cam(0)/y_cam(3);
    pt(1) = y_cam(1)/y_cam(3);
    pt(2) = y_cam(2)/y_cam(3);

    return pt;

}



cv::Mat selectColor(cv::Mat rgb_img, int selector) {
    cv::Mat hsv_image;
    cv::cvtColor(rgb_img, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat binary_img;
    switch (selector) {
    case 0: // Blue
        cv::inRange(hsv_image, cv::Scalar(80, 100, 100),
                    cv::Scalar(120, 255, 255), binary_img);
        break;
    case 4: // Dark Blue
        cv::inRange(hsv_image, cv::Scalar(80, 100, 30),
                    cv::Scalar(120, 255, 255), binary_img);
        break;

    case 1: // Green
        cv::inRange(hsv_image, cv::Scalar(35, 100, 40),
                    cv::Scalar(80, 255, 255), binary_img);
        break;

    case 3: // Dark-Green
        cv::inRange(hsv_image, cv::Scalar(35, 0, 30), cv::Scalar(80, 255, 255),
                    binary_img);
        break;

    default: // Red
        cv::Mat lower_red_hue_range;
        cv::Mat upper_red_hue_range;
        cv::inRange(hsv_image, cv::Scalar(0, 100, 100),
                    cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(hsv_image, cv::Scalar(160, 100, 100),
                    cv::Scalar(179, 255, 255), upper_red_hue_range);
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0,
                        binary_img);
        break;
    }

    return binary_img;
}

std::vector<cv::Point> findBiggestContourArea(std::vector<std::vector<cv::Point>> contours) {
    std::vector<cv::Point> c = *std::max_element(
        contours.begin(), contours.end(),
        [](std::vector<cv::Point> const &lhs,
           std::vector<cv::Point> const &rhs) {
            return contourArea(lhs, false) < contourArea(rhs, false);
        });
    return c;
}

std::vector<Board> findNNObjects(cv::Mat input_img,Board circle,int nn){
    
    //cut-off distance in cm:
    double cutoff = 5;
    int x = std::max(0,(int) (circle.getCenter()(0)-(circle.getRadius()*cutoff/2.)));
    int y = std::max(0,(int) (circle.getCenter()(1)-(circle.getRadius()*cutoff/2.)));
    int x_len = std::min((int) (circle.getRadius()*cutoff),input_img.cols-x);
    int y_len = std::min((int) (circle.getRadius()*cutoff),input_img.rows-y);

    // cut input image for better performance
    cv::Mat region = input_img(cv::Rect(x,y,x_len,y_len));

    

    // color selection
    cv::Mat binary_img = selectColor(region, 4); //blue

    //cv::imshow("reg:",binary_img);

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(binary_img,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    cv::Point2f center(-1.0,-1.0);
    float radius = -1.0;
    //cout << "contours.size: " << contours.size() << endl;
    if(contours.size() > 0){
        std::vector<cv::Point> c;
        c = findBiggestContourArea(contours);
        cv::minEnclosingCircle(c,center,radius);
        cv::circle(region, center, radius, cv::Scalar(0, 255, 0), 3);
    }
    else{
        cout << "No contours found..." << endl;
    }
    cv::imshow("reg1:",binary_img);
    cv::imshow("reg:",region);


    std::vector<Board> ret;
    if(radius > 0){
        Board board({center.x, center.y},radius);
        ret.push_back(board);
    }


    return ret;

}


float medianFilter(cv::Mat depth_img) {
    cv::Mat mask = cv::Mat(depth_img == depth_img);

    depth_img = depth_img * 128.;

    double median_index = (mask.rows * mask.cols) / 2;
    int bin = 0;
    double med = -1.0;

    int histSize = 256;
    float range[] = {0, 256};
    const float *histRange = {range};
    bool uniform = true;
    bool accumulate = false;
    cv::Mat hist;

    cv::calcHist(&depth_img, 1, 0, mask, hist, 1, &histSize, &histRange,
                 uniform, accumulate);

    for (int i = 0; i < histSize && med < 0.0; ++i) {
        bin += cvRound(hist.at<float>(i));
        if (bin > median_index && med < 0.0) {
            med = i;
            break;
        }
    }

    med = med / 128.;

    return med;
}

void updateParams(rai::Frame *cameraFrame,arr y_world,int x_im,int y_im,double d){
    arr cam_pos = cameraFrame->getPosition();
    arr cam_rot = cameraFrame->getQuaternion();
}


void onlineCalibration(){
    // load a configuration
    rai::KinematicWorld C;
    C.addFile("model.g");
    arr q_home = C.getJointState();
    arr Wmetric = diag(1., C.getJointStateDimension());

    // launch camera
    Var<byteA> _rgb;
    Var<floatA> _depth;
    RosCamera cam(_rgb, _depth, "cameraRosNodeJulian", "/camera/rgb/image_raw", "/camera/depth/image_rect");


    // launch robot interface
    RobotOperation B(C);
    B.sync(C);
    /*B.move(q_home, {5.});
    rai::wait();*/


    // Setting up Cameraframe
    rai::Frame *cameraFrame = C.addFrame("pclframe", "head");
    cameraFrame->calc_X_from_parent();
    cameraFrame->setPosition({-0.0499056, 0.231561, 1.7645});
    cameraFrame->setQuaternion({0.971032, 0.237993, -0.00607315, 0.0204557});
    arr Fxypxy = {539.637, 540.941, 317.533, 260.024};


    // looping
    while(true){
        _depth.waitForNextRevision();

        // grap copies of rgb and depth
        cv::Mat rgb = CV(_rgb.get()).clone();
        cv::Mat depth = CV(_depth.get()).clone();

        if(rgb.rows != depth.rows) continue;

        // calc world coordinates of the ball
        arr y_world,J;
        C.setJointState(B.getJointPositions());
        C.evalFeature(y_world,J,FS_position,{"calibL"});

        arr y_cam = globalPointToCameraPoint(y_world,cameraFrame->X.getInverseAffineMatrix());
        arr y_img = cameraPointToImgCoords(y_cam,Fxypxy);


        // Return if ball is not within the image
        if(y_img(0) < 0 || y_img(0) > rgb.cols ||
            y_img(1) < 0 || y_img(1) > rgb.rows ||
            y_img(2) <= 0){
            if(rgb.total()>0 && depth.total()>0){
                cv::imshow("rgb", rgb);
                cv::imshow("depth", 0.5*depth); //white=2meters
                cv::waitKey(1);
            }
            continue;
        }


        arr res = cameraPointToImgCoords({y_cam(0)+.02,y_cam(1),y_cam(2)},Fxypxy);
        double radius = sqrt(pow(y_img(0)-res(0),2) + pow(y_img(1)-res(1),2));
        Board circle(res,radius);

        cv::Mat input_img = rgb.clone();
        //Drawing output:
        cv::circle(rgb, cv::Point(y_img(0),y_img(1)), radius, cv::Scalar(0,0,255), 4);


        
        // find three nearest neighbours in the image
        std::vector<Board> ret;
        ret = findNNObjects(input_img,circle,3);

        if(ret.empty()){
            if(rgb.total()>0 && depth.total()>0){
                cv::imshow("rgb", rgb);
                cv::imshow("depth", 0.5*depth); //white=2meters
                cv::waitKey(1);
            }
            continue;
        }




        Board board_exp = ret[0];
        int x_im = (int) board_exp.getCenter()(0);
        int y_im = (int) board_exp.getCenter()(1);
        cv::Mat mask = depth(cv::Rect(x_im - (int) radius/2, y_im-(int) radius/2, (int)radius, (int)radius));
        double d = medianFilter(mask);

        cout << "updating parameters..." << endl;
        updateParams(cameraFrame,y_world,x_im,y_im,d);
        
        if(rgb.total()>0 && depth.total()>0){
            cv::imshow("rgb", rgb);
            cv::imshow("depth", 0.5*depth); //white=2meters
            cv::waitKey(1);
        }
    }
}




int main(int argc,char **argv){
    rai::initCmdLine(argc,argv);

    onlineCalibration();

    return 0;
}
