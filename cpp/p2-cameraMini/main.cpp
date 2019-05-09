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
#include <RosCom/baxter.h>
#include "RobotVision.cpp"
#include "FollowObject.cpp"
#include "Circle.cpp"





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




void get_objects_into_configuration(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

  RobotVision vision;

  RosCamera cam(_rgb, _depth, "cameraRosNodeJulian", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");

  double f = 1./tan(0.5*62.8*RAI_PI/180.);
  f *= 320.;
  arr Fxypxy = {f, f, 320., 240.}; //intrinsic camera parameters

  Depth2PointCloud d2p(_depth, Fxypxy);

  BaxterInterface B(true);

  rai::KinematicWorld C;
  C.addFile("model.g");

  //followObject test(C,&B);


  rai::Frame *pcl = C.addFrame("pcl", "head");
  rai::Frame *marker = C.addFrame("marker", "head");
  cv::Point2f point;
  for(uint i=0;i<10000;i++){
    _rgb.waitForNextRevision();

    marker->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1) ");
    marker->calc_X_from_parent();
    marker->setShape(rai::ST_sphere, {.1});

    pcl->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1) ");
    pcl->calc_X_from_parent();


    arr q_real = B.get_q();
    if(q_real.N==C.getJointStateDimension())
      C.setJointState(q_real);

    if(d2p.points.get()->N>0){
      C.gl().dataLock.lock(RAI_HERE);
      pcl->setPointCloud(d2p.points.get());
      C.gl().dataLock.unlock();
      int key = C.watch(false);
      if(key=='q') break;
    }

    { //display
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        Circle circle = vision.detectionProcess(rgb);

        float d = depth.at<float>(circle.y,circle.x);
        cout << d << endl;
        //how to convert image to 3D coordinates:
        double x_pixel_coordinate=circle.x;
        double y_pixel_coordinate=circle.y;
        double depth_from_depthcam=d;
        arr pt = { x_pixel_coordinate, y_pixel_coordinate, depth_from_depthcam };
        depthData2point(pt, Fxypxy); //transforms the point to camera xyz coordinates
        marker->X.applyOnPoint(pt); //transforms into world coordinates

        //float d = medianCircle(depth,point,radius);

        C.gl().dataLock.lock(RAI_HERE);
        marker->setPosition(pt);
        C.gl().dataLock.unlock();
        int key = C.watch(false);
        if(key=='q') break;
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        cv::waitKey(1);
      }
    }
  }
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  //ball_tracking();
  get_objects_into_configuration();

  return 0;
}
