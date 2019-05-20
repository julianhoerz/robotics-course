
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
        
        int x_start = 0;
        int y_start = 0;
        int x_length = 0;
        int y_length = 0;
        if(x-radius < 0){
            x_start = 0;
            x_length += x;
        }
        else{
            x_start = x-radius;
            x_length += radius;
        }

        if(y-radius < 0){
            y_start = 0;
            y_length += y;
        }
        else{
            y_start = y-radius;
            y_length += radius;
        }


        if((x + radius) > depth.cols){
            x_length += depth.cols - x;
        }
        else{
            x_length += radius;
        }

        if((y + radius) > depth.rows){
            y_length += depth.rows - y;
        }
        else{
            y_length += radius;
        }


        if((x - radius) < 0 || 
            (y - radius) < 0 ||  
            (x + radius) > depth.cols || 
            (y + radius) > depth.rows){
              //cout << "Skipping... big circle" << endl;
            return -1.;
        }

        cv::Rect region(x - radius,y - radius,2*radius,2*radius);
        cv::Mat roi(depth, region);
        cv::Mat mask2 = cv::Mat(roi == roi);

        roi = roi*128.;

        cv::Mat circle;
        cv::bitwise_and(mask, mask2, mask, cv::Mat());

        //cv::imshow("mask: ", mask);
        //cv::imshow("roi: ", roi);
        
        double m = (mask.rows*mask.cols) / 2;
        int bin = 0;
        double med = -1.0;

        int histSize = 256;
        float range[] = { 0, 256 };
        const float* histRange = { range };
        bool uniform = true;
        bool accumulate = false;
        cv::Mat hist;
        
        cv::calcHist( &roi, 1, 0, mask, hist, 1, &histSize, &histRange, uniform, accumulate );
        
        for ( int i = 0; i < histSize && med < 0.0; ++i )
        {
            bin += cvRound( hist.at< float >( i ) );
            if ( bin > m && med < 0.0 ){
                med = i;
                break;
            }
        }

        med = med/128.;

        return med;

    }

};





