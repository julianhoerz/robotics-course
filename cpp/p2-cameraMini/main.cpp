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

void minimal_use(){

  Var<byteA> _rgb;
  Var<floatA> _depth;

  int cnt = 0;
  

  #if 0 //using ros
    RosCamera cam(_rgb, _depth, "cameraRosNodeJulian", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");
  #else //using a webcam
    OpencvCamera cam(_rgb);
  #endif

  //looping images through opencv
  for(int i=0;i>-1;i++){
    cv::Mat img = CV(_rgb.get());
    //cv::Mat depth = CV(_depth.get());
    if(img.total()>0){
      cv::medianBlur(img, img, 11);
      cv::Mat hsv_image;
      cv::cvtColor(img, hsv_image, cv::COLOR_BGR2HSV);
      cv::Mat orig_image = img.clone();
      cv::Mat lower_red_hue_range;
      cv::Mat upper_red_hue_range;
      cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
      cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

      cv::Mat red_hue_image;
      cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
      cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 100, 100);

      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

      //cv::ellipse( im2, Point( 120, 130 ), Size( 50.0, 60.0 ), 0, 0, 360, Scalar( 255, 255, 255), -1, 8 );
      cv::Mat buff_image = orig_image.clone();
      cv::Mat buff2_image = red_hue_image.clone();
      cv::Mat mymask;

      //buff_image.setTo(cv::Scalar(0,0,0));
      if(circles.size() == 0){
        cnt ++;
        cout << "Skipping... " << cnt << endl;
      } 
      else{

        //for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
        for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
          cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
          int radius = std::round(circles[current_circle][2]);
          if((center.x - radius) < 0 || 
            (center.y - radius) < 0 ||  
            (center.x + radius) > img.cols || 
            (center.y + radius) > img.rows){
              //cout << "Skipping... big circle" << endl;
              continue;
          }
          else{
            cv::Mat mask(cv::Size(radius*2,radius*2), CV_8U, cv::Scalar(0)); // all black
            cv::circle(mask, cv::Point(radius,radius), radius, cv::Scalar(255), -1);
            cv::Rect region(center.x-radius,center.y-radius,radius*2,radius*2); // example roi
            cv::Mat roi(red_hue_image, region); 
            cv::bitwise_and(roi, roi, buff2_image, mask);
            cout << "Typical mean: " << cv::mean(mask)[0] - cv::mean(buff2_image)[0] << endl;
            if(cv::mean(cv::mean(mask)[0] - buff2_image)[0] < 100){
              //cout << "real circle" << endl;
              cv::circle(buff_image, center, radius, cv::Scalar(255, 255, 255), 5);
            }
            else{
              // 
              //cv::circle(buff_image, center, radius, cv::Scalar(0, 255, 0), 5);
            }

            
            mymask =mask;
          }


        }
      }
      cv::Mat res;
      //cout << buff_image.shape
      //cv::bitwise_and(,red_hue_image,res,buff_image);


      cv::imshow("red_hue_image", red_hue_image);
      cv::imshow("RGB", orig_image);
      cv::imshow("AND", buff_image);
      //cv::imshow("depth", 0.5*depth); //white=2meters
      //cv::imshow("Mask: ", mymask);
      //cv::imshow("Mask2: ", buff2_image);


      cv::waitKey(1);
    }
/*
  //looping images through opencv
  for(uint i=0;i<100;i++){
    _rgb.waitForNextRevision();
    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        cv::waitKey(1);
      }
    }
  }*/
  }
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






cv::Point2f detectionProcess(cv::Mat img){
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
  if(contours.size() > 0){
    std::vector<cv::Point> c;
    c = findBiggestContourArea(contours);
    float radius;
    cv::minEnclosingCircle(c,center,radius);
    cv::circle(contour_img, center, radius, cv::Scalar(0, 255, 0), 3);
  }
  else{
    cout << "No contours found..." << endl;
  }

  cv::imshow("circles", contour_img);
  return center;
}



void ball_tracking(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

  #if 0 //using ros
    RosCamera cam(_rgb, _depth, "cameraRosNodeJulian", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");
  #else //using a webcam
    OpencvCamera cam(_rgb);
  #endif

  //looping images through opencv
  for(int i=0;i>-1;i++){
    cv::Mat img = CV(_rgb.get());
    if(img.total()<=0){
      continue;
    }

    detectionProcess(img);
    

    cv::waitKey(1);
  }
}


float medianCircle(cv::Mat depth,cv::Point2f point,float radius){
  //create mask (black background)
  /*cv::Mat mask(cv::Size(radius*2,radius*2), CV_8U, cv::Scalar(0));
  cv::circle(mask, cv::Point(radius,radius), radius, cv::Scalar(255), -1);

  //extract region from depthimage
  cv::Rect region(center.x-radius,center.y-radius,radius*2,radius*2);
  cv::Mat roi(depth, region);

  cv::Mat buff2_image;
  //
  cv::bitwise_and(roi, roi, buff2_image, mask);
  cout << "Typical mean: " << cv::mean(mask)[0] - cv::mean(buff2_image)[0] << endl;*/
  return 0.1f;

}







void get_objects_into_configuration(){
  Var<byteA> _rgb;
  Var<floatA> _depth;


  RosCamera cam(_rgb, _depth, "cameraRosNodeJulian", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");

  Depth2PointCloud d2p(_depth, 600.f, 600.f, 320.f, 240.f);

  BaxterInterface B(true);

  rai::KinematicWorld C;
  C.addFile("model.g");
  C.setJointState(B.get_q());


  rai::Frame *pcl = C.addFrame("pcl", "camera");
  rai::Frame *marker = C.addFrame("marker", "camera");
  cv::Point2f point;
  for(uint i=0;i<10000;i++){
    _rgb.waitForNextRevision();


    if(d2p.points.get()->N>0){
      C.gl().dataLock.lock(RAI_HERE);
      pcl->setPointCloud(d2p.points.get());
      C.gl().dataLock.unlock();
      C.watch(false);
    }

    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        point = detectionProcess(rgb);
        cv::Vec3b bgrPixel = depth.at<cv::Vec3b>(point.x, point.y);

        //float d = medianCircle(depth,point,radius);

        //cout << depth.size() << endl;
        /*float d = depth.at(point);
        float p_x = rgb.cols / 2.0;
        float p_y = rgb.rows / 2.0;
        float f_x = 600.f;
        float f_y = 600.f;
        point.x;
        float x_new = d*(point.x - p_x)/f_x;
        float y_new = -d*(point.y - p_y)/f_y;
        float z_new = -d;
        C.gl().dataLock.lock(RAI_HERE);
        marker->setPosition({x_new,y_new,z_new});
        C.gl().dataLock.unlock();*/
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        cv::waitKey(1);
      }
    }
  }

  rai::wait();
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  //minimal_use();
  //ball_tracking();
  get_objects_into_configuration();

  return 0;
}
