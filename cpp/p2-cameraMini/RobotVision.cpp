
/*

#ifndef RAI_OPENCV
    #define RAI_OPENCV 1
#endif
#ifndef RAI_ROS
    #define RAI_ROS 1
#endif

#define DISP_J 1



//kill /end_effector_publisher


#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <RosCom/baxter.h>*/
#ifndef CIRCLE
    #include "Circle.cpp"
    #define CIRCLE
#endif





class RobotVision{
    public:

    RobotVision(){

    }









    std::vector<cv::Point> findBiggestContourArea(std::vector<std::vector<cv::Point> > contours){
        std::vector<cv::Point> c = *std::max_element(contours.begin(),
        contours.end(),
        [](std::vector<cv::Point> const& lhs, std::vector<cv::Point> const& rhs)
        {
            return contourArea(lhs, false) < contourArea(rhs, false);
        });
        return c;
    }














    cv::Mat selectColor(cv::Mat hsv_image, int selector){
        cv::Mat final_img;
        switch (selector)
        {
        case 0: //Blue
            cv::inRange(hsv_image, cv::Scalar(80, 100, 100), cv::Scalar(120, 255, 255), final_img);
            break;

        case 1: //Green
            cv::inRange(hsv_image, cv::Scalar(35, 100, 100), cv::Scalar(80, 255, 255), final_img);
            break;
        
        default: //Red
            cv::Mat lower_red_hue_range;
            cv::Mat upper_red_hue_range;
            cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
            cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, final_img);
            break;
        }

        return final_img;
    }









    Circle detectionProcess(cv::Mat img){
        cv::Mat contour_img = img.clone();

        // Transforming to hsv image
        cv::Mat blurred_img, hsv_image;
        cv::GaussianBlur(img, blurred_img, cv::Size(11, 11), 0, 0);
        cv::cvtColor(blurred_img, hsv_image, cv::COLOR_BGR2HSV);

        // Select red parts of the image
        cv::Mat hue_image;
        hue_image = selectColor(hsv_image,1);
        cv::imshow("Color selected image", hue_image);

        // Erosion and Dilation to delete small circles
        cv::Mat img_final = hue_image;
        cv::erode(img_final, img_final, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        cv::dilate(img_final, img_final, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);


        // Find contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(img_final,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        cv::Point2f center(-1.0,-1.0);
        float radius = -1.0;
        if(contours.size() > 0){
            std::vector<cv::Point> c;
            c = findBiggestContourArea(contours);
            cv::minEnclosingCircle(c,center,radius);
            cv::circle(contour_img, center, radius, cv::Scalar(0, 255, 0), 3);
        }
        else{
            cout << "No contours found..." << endl;
        }

        cv::imshow("circles", contour_img);
        return Circle(center.x,center.y,radius);
    }











    float medianCircle(cv::Mat depth,double x, double y,float radius){
        //create mask (black background)
        cv::Mat mask(cv::Size(radius*2,radius*2), CV_8U, cv::Scalar(0));
        cv::circle(mask, cv::Point(radius,radius), radius, cv::Scalar(255), -1);

        //extract region from depthimage
        cv::Rect region(x-radius,y-radius,radius*2,radius*2);
        cv::Mat roi(depth, region);

        cv::Mat buff2_image;
        //
        cv::bitwise_and(roi, roi, buff2_image, mask);
        cout << "Typical mean: " << cv::mean(mask)[0] - cv::mean(buff2_image)[0] << endl;
        return 0.1f;

    }

};





