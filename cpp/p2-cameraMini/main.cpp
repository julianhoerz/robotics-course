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
<<<<<<< HEAD
#ifndef CIRCLE
    #include "Circle.cpp"
    #define CIRCLE
#endif
#include "RobotVision.cpp"
#include "RobotKinematics.cpp"
#include "game-logic.cpp"


void testFilter(){

    Var<byteA> _rgb;
    Var<floatA> _depth;
    
    #if 0 //using ros
      RosCamera cam(_rgb, _depth, "cameraRosNodeJulian", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");
    #else //using a webcam
      OpencvCamera cam(_rgb);
    #endif
=======
#include <Kin/cameraview.h>

//===========================================================================

void minimal_use(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

#if 1 //using ros
//  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");

  RosCamera kin(_rgb, _depth, "cameraRosNodeMarc", "/kinect/rgb/image_rect_color", "/kinect/depth_registered/sw_registered/image_rect_raw", true);
#else //using a webcam
  OpencvCamera cam(_rgb);
#endif
>>>>>>> 57969502b0c07ed0fb3e1bf7c3a27d6e79136b07

    for(int i=0;i>-1;i++){
        cv::Mat img = CV(_rgb.get());
        //cv::Mat depth = CV(_depth.get());
        if(img.total()>0){
            RobotVision vision;
            Circle cr = vision.detectionProcess(img);
            cv::Mat grey;
            cv::cvtColor(img, grey, CV_BGR2GRAY);
            float med = -1.;
            med = vision.medianCircle(grey,cr.x,cr.y,cr.r);
            cout << "Median: " << med << endl;
            cv::imshow("RGB: ", img);
            cv::imshow("Grey: ", grey);
            cv::waitKey(1);
        }
    }
}

//===========================================================================

<<<<<<< HEAD

void followObjectProcess(){
    Var<byteA> _rgb;
    Var<floatA> _depth;

    RobotVision vision;

    RosCamera cam(_rgb, _depth, "cameraRosNodeMarcToussaint", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");

    double f = 1./tan(0.5*60.8*RAI_PI/180.);
    f *= 320.;
    arr Fxypxy = {f, f, 320., 240.}; //intrinsic camera parameters

    Depth2PointCloud d2p(_depth, Fxypxy);

    BaxterInterface B(true);

    rai::KinematicWorld C;
    C.addFile("model.g");


    RobotKinematics kinematics;


    rai::Frame *pcl = C.addFrame("pcl", "head");
    rai::Frame *testframe = C.addFrame("testframe","head");
    //rai::Frame *marker = C.addFrame("marker", "testframe");
    rai::Frame *ball = C.addFrame("ball","base");
    cv::Point2f point;

    testframe->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1) ");
    testframe->calc_X_from_parent();

    //marker->setShape(rai::ST_sphere, {.1});
    ball->setShape(rai::ST_sphere, {.05});

    pcl->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1) ");
    pcl->calc_X_from_parent();

=======
void wrist_camera(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

 RosCamera kin(_rgb, _depth, "cameraRosNodeMarc", "/cameras/right_hand_camera/image", "", true);

  //looping images through opencv
  for(uint i=0;i<1000;i++){
    _rgb.waitForNextRevision();
    {
      byteA RGB = _rgb.get();
      cv::Mat rgb = cv::Mat(RGB.d0, RGB.d1, CV_8UC4, RGB.p);
      cv::cvtColor(rgb, rgb, cv::COLOR_BGRA2RGBA);

      if(rgb.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        int key = cv::waitKey(1);
        if((key&0xff)=='q') break;
      }
    }
  }
}

//===========================================================================

void get_objects_into_configuration(){
  //subscribe to image and depth
  Var<byteA> _rgb;
  Var<floatA> _depth;
  RosCamera cam(_rgb, _depth, "cameraRosNode", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");

  //set the intrinsic camera parameters
  double f = 1./tan(0.5*60.8*RAI_PI/180.);
  f *= 320.;
  arr Fxypxy = {f, f, 320., 240.};

  //convert to point cloud (in camera frame, used only for display)
  Depth2PointCloud d2p(_depth, Fxypxy);

  //interface to baxter to query it's real pose
  BaxterInterface B(true);

  //load a configuration
  rai::KinematicWorld C;
  C.addFile("../../rai-robotModels/baxter/baxter_new.g");

  //add a frame for the camera
  rai::Frame *cameraFrame = C.addFrame("camera", "head");
  cameraFrame->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");
  cameraFrame->calc_X_from_parent();

  for(uint i=0;i<10000;i++){
    //wait for the next rgb image
    _rgb.waitForNextRevision();
>>>>>>> 57969502b0c07ed0fb3e1bf7c3a27d6e79136b07

    //sync C with the real baxter pose
    arr q_real = B.get_q();
    if(q_real.N==C.getJointStateDimension())
      C.setJointState(q_real);

<<<<<<< HEAD
    arr q_zero = q_real*0.;

    arr prev_pt = {0,0,0};
    //B.send_q(q_zero);

    arr low_pass = {0,0,0,0,0,0,0};

    int lp_cnt = 0;

    int startcnt = 0;

    double dist_mean;


    for(uint i=0;i<10000;i++){
        _rgb.waitForNextRevision();
        C.watch(false);

        //cout << "Test" << endl;

        if(d2p.points.get()->N>0){
            C.gl().dataLock.lock(RAI_HERE);
            pcl->setPointCloud(d2p.points.get(), _rgb.get());
            C.gl().dataLock.unlock();
            //int key = C.watch(false);
            //if(key=='q') break;
        }

        { //display
            cv::Mat rgb = CV(_rgb.get());
            cv::Mat depth = CV(_depth.get());

            if(rgb.total()>0 && depth.total()>0){

                Circle circle = vision.detectionProcess(rgb);
                circle.r = circle.r * 0.5;
                cout << "x and y:" << circle.x << " , " << circle.y << endl;
                if(circle.x < 0 || circle.y < 0){
                    continue;
                }
                float d_med = vision.medianCircle(depth,circle.x,circle.y,circle.r);
                cout << "mymed: " << d_med << endl;
                if(d_med < 0){
                    continue;
                }
                float d = d_med;

                /*if(startcnt >= 7){
                    if(abs(d-dist_mean/7) > 1.){
                        continue;
                    }
                    else{
                        dist_mean += d;
                        dist_mean -= low_pass(lp_cnt);
                        low_pass(lp_cnt) = d;
                        lp_cnt = (lp_cnt +1) % 7;
                    }
                }
                else{
                    low_pass(lp_cnt) = d;
                    lp_cnt = (lp_cnt +1) % 7;
                }*/
                
                
                //cout << "depth" << depth << endl;
                //float d = depth.at<float>(circle.y,circle.x);
                //cout << "Test" << endl;
                
                //how to convert image to 3D coordinates:
                double x_pixel_coordinate=circle.x;
                double y_pixel_coordinate=circle.y;
                double depth_from_depthcam=d;
                arr pt = { x_pixel_coordinate, y_pixel_coordinate, depth_from_depthcam };
                depthData2point(pt, Fxypxy); //transforms the point to camera xyz coordinates
                //cout << "pos: "<<pt<< "relative pos1: " << C.getFrameByName("marker")->getRelativePosition() <<endl;
                //cout << "Test" << endl;
                //marker->setRelativePosition(pt);
                //C.watch(true);
                pcl->X.applyOnPoint(pt); //transforms into world coordinates

                //float d = medianCircle(depth,point,radius);
                cout << pt << endl;
                cout << "Distance: " << ~(pt-prev_pt)*(pt-prev_pt) << endl;
                if((~(pt-prev_pt)*(pt-prev_pt))(0) > 1.){
                    prev_pt = pt;
                    continue;
                }

                prev_pt = pt;

                //C.gl().dataLock.lock(RAI_HERE);
                //marker->setPosition(pt);
                //marker->X.applyOnPoint(pt);
                //C.gl().dataLock.unlock();
                ball->setPosition(pt);
                

                
                
                arr qfin =C.getJointState();
                qfin = kinematics.updatePosition(pt,&qfin,C.getJointState());
                C.setJointState(qfin);
                C.getFrameByName("ball")->setPosition(pt);
                B.send_q(qfin);

                //cv::imshow("depth", 0.5*depth); //white=2meters
                cv::imshow("rgb", rgb);
                cv::waitKey(1);
            }
        }
=======
    //display the point cloud by setting it as cameraFrame shape
    if(d2p.points.get()->N>0){
      C.gl().dataLock.lock(RAI_HERE); //(in case you interact with the GUI while the drawing data changes)
      cameraFrame->setPointCloud(d2p.points.get(), _rgb.get());
      C.gl().dataLock.unlock();
      int key = C.watch(false);
      if(key=='q') break;
>>>>>>> 57969502b0c07ed0fb3e1bf7c3a27d6e79136b07
    }
}

<<<<<<< HEAD
void home(int select){
    arr q_home;
    if(select == 0){
        q_home = {-0.103544, -0.704864, 0.490107, -0.151481, -0.265379, 1.57693, -1.27666, 1.81508, 1.26323, -0.75817, 0.33901, 0.985583, 1.32958, -0.903515, -0.102393, 0 ,0 };
    }
    else{
        q_home = {0.0333641 ,0.078233, -0.082068, -0.998238, -0.998238 ,1.16774, -1.16698, 1.94164, 1.94164, -0.672267, 0.6715, 1.0178, 1.01856, 0.498927, -0.49816, 0, 0 };
    }

    rai::KinematicWorld C;
    C.addFile("../../rai-robotModels/baxter/baxter.g");
    BaxterInterface B(true);
    arr q = B.get_q();
    while((~(q-q_home)*(q-q_home))(0) > 0.01){
        //cout << "scalar prod:" << (~(q-q_home1)*(q-q_home1))(0) <<endl;
        rai::wait(.1);
        B.send_q(q_home);
        q = B.get_q();
        C.setJointState(q);
        C.watch(false);
=======
    //display the images
    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        int key = cv::waitKey(1);
        if((key&0xff)=='q') break;
      }
>>>>>>> 57969502b0c07ed0fb3e1bf7c3a27d6e79136b07
    }
}

void ticktacktoe(){
    int step = 1;
    arr state = {   0.,0.,0.,
                    0.,0.,0.,
                    0.,0.,0.};
    arr buff = state;

<<<<<<< HEAD
    int input;
=======
    //example on how to convert image to 3D coordinates:
    double x_pixel_coordinate=0., y_pixel_coordinate=0., depth_from_depthcam=1.2;
    arr pt = { x_pixel_coordinate, y_pixel_coordinate, depth_from_depthcam };
    depthData2point(pt, Fxypxy); //transforms the point to camera xyz coordinates
    cameraFrame->X.applyOnPoint(pt); //transforms into world coordinates
  }
>>>>>>> 57969502b0c07ed0fb3e1bf7c3a27d6e79136b07

    while (step != -1){
        step = getNextMove(state);
        cout << "step: " << step << endl;
        state(step) = 1.;
        buff = state;
        cout << "state:" << endl << buff.reshape({3,3}) << endl;
        std::cin >> input;
        state(input) = 2.;
        buff = state;
        cout << "state:" << endl << buff.reshape({3,3}) << endl;
    }
    cout << "game over" << endl;
}

//===========================================================================

void usingCameraSimulation(){
  //load a configuration
  rai::KinematicWorld C;
  C.addFile("../../rai-robotModels/baxter/baxter_new.g");

  //add a frame for the camera
  rai::Frame *cameraFrame = C.addFrame("camera", "head");
  cameraFrame->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");
  cameraFrame->calc_X_from_parent();

  //associate an opengl renderer with the camera frame
  rai::CameraView camSim(C);
  camSim.addSensor("myCam", "camera", 640, 480, 1.);
  camSim.selectSensor("myCam");

  //add an object into the rendered scene
  rai::Frame *realWorldObj = camSim.K.addFrame("obj");
  realWorldObj->setPosition({1., 0., 1.});
  realWorldObj->setShape(rai::ST_ssBox, {.1, .2, .3, .01});
  realWorldObj->setColor({.1, .7, .1});

  //compute things
  byteA rgb;
  floatA depth;
  arr pcl;
  camSim.computeImageAndDepth(rgb, depth);
  camSim.computePointCloud(pcl, depth, false);

  //display the images
  {
    cv::Mat _rgb = CV(rgb);
    cv::Mat _depth = CV(depth);
    cv::cvtColor(_rgb, _rgb, cv::COLOR_BGR2RGB);

    if(_rgb.total()>0 && _depth.total()>0){
      cv::imshow("rgb", _rgb.clone()); //white=2meters
      cv::imshow("depth", 0.5*_depth.clone()); //white=2meters
      int key = cv::waitKey(100);
//      if((key&0xff)=='q') break;
    }
  }

  //add the rendered point cloud as shape of the cameraFrame in configuration C
  cameraFrame->setPointCloud(pcl, rgb);
  //...and display it
  C.watch(true);
}

//===========================================================================

int main(int argc,char **argv){
    rai::initCmdLine(argc,argv);
    ticktacktoe();

    //home();
    /*
    switch(rai::getParameter<int>("home",4)){
        case 0:  home(0);  break;
        case 1:  home(1);  break;
        default:  break;
    } 


    switch(rai::getParameter<int>("follow",4)){
        case 0:  followObjectProcess();  break;
        default:  break;
    } */


<<<<<<< HEAD
    //followObjectProcess();
    //testFilter();
=======
//  minimal_use();
  wrist_camera();
//  get_objects_into_configuration();
//  usingCameraSimulation();
>>>>>>> 57969502b0c07ed0fb3e1bf7c3a27d6e79136b07

    return 0;
}
