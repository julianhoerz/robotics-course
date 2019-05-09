


/*


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

  //looping images through opencv
  for(uint i=0;i<100;i++){
    _rgb.waitForNextRevision();
    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        int key = cv::waitKey(1);
        if((key&0xff)=='q') break;
      }
    }
  }
  }
}*/















/*
void ball_tracking(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

  #if 0 //using ros
    RosCamera cam(_rgb, _depth, "cameraRosNodeJulian", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");
  #else //using a webcam
    OpencvCamera cam(_rgb);
  #endif

  RobotVision vision;

  //looping images through opencv
  for(int i=0;i>-1;i++){
    cv::Mat img = CV(_rgb.get());
    if(img.total()<=0){
      continue;
    }

    vision.detectionProcess(img);
  
    cv::waitKey(1);
  }
}



*/