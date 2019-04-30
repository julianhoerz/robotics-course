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
#include <Kin/frame.h>

void minimal_use(){

  Var<byteA> rgb;
  Var<floatA> depth;
  

#if 1 //using ros
  RosCom ROS;
  
  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb(rgb, "/camera/rgb/image_rect_color");
//  SubscriberConv<sensor_msgs::Image, floatA, &conv_imageu162floatA> subDepth(depth, "/camera/depth_registered/image_raw");
#else //using a webcam
  OpencvCamera cam(rgb);
#endif

  //looping images through opencv
  for(uint i=0;i<100;i++){
    cv::Mat img = CV(rgb.get());
    if(img.total()>0){
      cv::Mat hsv_image;
      cv::cvtColor(img, hsv_image, cv::COLOR_BGR2HSV);
      cv::Mat orig_image = img.clone();
      cv::Mat lower_red_hue_range;
      cv::Mat upper_red_hue_range;
      cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
      cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

      cv::Mat red_hue_image;
      cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
      cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);


      if(circles.size() == 0){
        cout << "Endend" << endl;
        continue;
      } 
      for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
        cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
        int radius = std::round(circles[current_circle][2]);
        cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
      }


      cv::imshow("RGB", orig_image);


      cv::waitKey(1);
    }
    rai::wait(.1);
  }
}

//void get_objects_into_configuration(){
//  RosCom ROS;

//  Var<byteA> rgb;
//  Var<floatA> depth;

//  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb(rgb, "/camera/rgb/image_rect_color");
//  SubscriberConv<sensor_msgs::Image, floatA, &conv_imageu162floatA> subDepth(depth, "/camera/depth_registered/image_raw");


//  Depth2PointCloud d2p(depth, 1.);

//  rai::KinematicWorld C;
//  C.addFile("model.g");
//  rai::Frame *pcl = C.addFrame("pcl", "camera", "shape:pointCloud");
//  for(uint i=0;i<100;i++){
////    cout <<d2p.points.get()->N <<endl;
//    pcl->shape->mesh().V = d2p.points.get();
//    pcl->shape->mesh().V.reshape(640*480,3);
//    C.watch(false);
//    rai::wait(.1);
//  }

//  rai::wait();
//}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  minimal_use();

  return 0;
}
