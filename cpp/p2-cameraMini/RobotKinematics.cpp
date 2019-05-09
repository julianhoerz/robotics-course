//#include <Kin/kin.h>
//#include <RosCom/baxter.h>
//#include <Operate/robotOperation.h>





class RobotKinematics{

    private: 
    rai::KinematicWorld *K;
    BaxterInterface *B;
    arr q_home;

    public:

    RobotKinematics(rai::KinematicWorld *k, BaxterInterface *b){
        K = k;
        B = b;
    }

    void captureHome(){
        q_home = K->getJointState();
    }




    void updatePosition(arr pos){
        
        arr q_real = B->get_q();
        if(q_real.N==K->getJointStateDimension())
            K->setJointState(q_real);


        arr q,W,y,J,Phi,PhiJ;
        uint n = K->getJointStateDimension();
        double w = rai::getParameter("w",1e-4);
        W.setDiag(w,n);


        

        arr y_final = pos;
        
        double steps = 100.;
        arr y_target, yvec,Jvec;

        // Position of arm 
        arr pos_start;
        K->evalFeature(pos_start, J, FS_position, {"baxterR"});
        //pos_start += vec_start*-0.4;


        arr qcurrent, Jcurrent;


        
        for(int i=1; i<steps; i++){
            Phi.clear(); 
            PhiJ.clear();
            
            // Position of arm
            K->evalFeature(y,J,FS_positionDiff,{"baxterR","marker"});
            Phi.append(y/30.);
            PhiJ.append(J/30.);

            //K.evalFeature(y, J, FS_position, {"baxterL"});
            //y_target = motionProfile(pos_start,y_final,i,steps);
            //Phi.append(y_target - (y));
            //Phi.append(y);
            //PhiJ.append(J);


            // Calculate Minimum...
            q -= 0.3*inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
            K->setJointState(q);
            
        }
        K->watch(false);

        //B->send_q({q});

        }



};

