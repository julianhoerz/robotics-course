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

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
  //The following doesn't work due to NaNs
  //double minVal, maxVal; 
  //minMaxLoc(float_img, &minVal, &maxVal);
  //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  //mono8_img = cv::Scalar(0);
  //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
}

void vision(){

  // launch camera
  Var<byteA> _rgb;
  Var<floatA> _depth;
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth_registered/image_raw");


  cv::VideoCapture cap(0); 
 
  // Check if camera opened successfully

 
  // Default resolution of the frame is obtained.The default resolution is system dependent. 
  int frame_width = 640;//cap.get(cv::CV_CAP_PROP_FRAME_WIDTH); 
  int frame_height = 480;//cap.get(cv::CV_CAP_PROP_FRAME_HEIGHT); 
   
  // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
  cv::VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(frame_width,frame_height),false); 
  int cnt = 0;
  while(true)
  { 
    cnt ++;
    _depth.waitForNextRevision();

    // grap copies of rgb and depth
    cv::Mat rgb = CV(_rgb.get()).clone();
    cv::Mat depth = CV(_depth.get()).clone();
    //cv::Mat frame = rgb.clone();

    //cv::Mat depth_float_img = cv_ptr->image;
    cv::Mat frame;
    depthToCV8UC1(depth, frame);
     
    // Capture frame-by-frame 
    //cap >> frame;
  
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
     
    // Write the frame into the file 'outcpp.avi'
    video.write(frame);
    
    // Display the resulting frame    
    imshow( "Frame", frame );
  
    // Press  ESC on keyboard to  exit
    char c = (char)cv::waitKey(1);
    if( c == 27 ) 
      break;
  }
 
  // When everything done, release the video capture and write object
  cap.release();
  video.release();
 
  // Closes all the windows
  cv::destroyAllWindows();
/*
  // looping
  for(uint i=0;i<1000;i++){
    _depth.waitForNextRevision();

    // grap copies of rgb and depth
    cv::Mat rgb = CV(_rgb.get()).clone();
    cv::Mat depth = CV(_depth.get()).clone();

    cout << "width: " << depth.cols << " height: " << depth.rows << endl;

    if(rgb.rows != depth.rows) continue;

    

    if(rgb.total()>0 && depth.total()>0){

      cv::imshow("rgb", rgb);
      cv::imshow("depth", 0.5*depth); //white=2meters
      int key = cv::waitKey(1);
      if((key&0xff)=='q') break;
    }
  }*/
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  vision();

  return 0;
}
