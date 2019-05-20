//#include <Kin/kin.h>
//#include <RosCom/baxter.h>
//#include <Operate/robotOperation.h>





class RobotKinematics{

    private: 
    rai::KinematicWorld *Ka;
    BaxterInterface *Ba;
    arr q_home;

    public:

    RobotKinematics(){

    }

    void captureHome(){
        q_home = Ka->getJointState();
    }




    arr updatePosition(arr pos,arr *qfin, arr js){
        arr q,W,y,J,Phi,PhiJ;
        rai::KinematicWorld K;
        K.addFile("model.g");
        K.setJointState(js);



        q = K.getJointState();
            
        
        //cout << "Working1" /*<< q_real.N*/ << endl;
        
        uint n = K.getJointStateDimension();
        double w = rai::getParameter("w",1e-4);
        W.setDiag(w,n);

        //cout << "Working2" << K.getJointStateDimension() << endl;
        
        //cout << "test" << endl;
        arr y_final = pos;
        
        double steps = 100.;
        arr y_target, yvec,Jvec;

        // Position of arm 
        arr pos_start;
        K.evalFeature(pos_start, J, FS_position, {"baxterR"});
        //pos_start += vec_start*-0.4;

        //cout << "Working3" << endl;
        arr qcurrent, Jcurrent;

        //cout << "test" << endl;

        arr q_home = {0.0333641 ,0.078233, -0.082068, -0.998238, -0.998238 ,1.16774, -1.16698, 1.94164, 1.94164, -0.672267, 0.6715, 1.0178, 1.01856, 0.498927, -0.49816, 0, 0 };

        K.addObject("myobj", rai::ST_capsule, {.2, .05}, {1., 1., 0.}, -1., 0, pos);
        
        for(int i=1; i<steps; i++){
            //cout << "Working111111" << endl;
            Phi.clear(); 
            PhiJ.clear();
            
            //cout << "Working1" << endl;
            // Position of arm
            K.evalFeature(y,J,FS_positionDiff,{"baxterR","myobj"});
            Phi.append(y/30.);
            PhiJ.append(J/30.);


            K.evalFeature(q,J,FS_qItself,{});
            Phi.append((q-q_home)/500.);
            PhiJ.append(J/500.);


            //cout << "Phi: " << Phi << endl;
            //cout << "W: " << W << endl;
            //cout << "PhiJ: " << PhiJ << endl;

            //K.evalFeature(y, J, FS_position, {"baxterL"});
            //y_target = motionProfile(pos_start,y_final,i,steps);
            //Phi.append(y_target - (y));
            //Phi.append(y);
            //PhiJ.append(J);

            //cout << "Pos of marker: " << K.getFrameByName("ball")->getPosition() << endl;
            //K.watch(true);
            // Calculate Minimum...
            q -= 0.3*inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
            //cout << q <<endl;
            K.setJointState(q);
            //cout << "step: " << i << endl;
            //K.getFrameByName("ball")->setPosition(pos);
            //K.watch(true);
            
        }
        //cout << "Joint states: " << q << endl;
        //cout << "Test: "<< *qfin->N << endl;
        /*for(int i = 0; i < q.N; i++){
            //qfin(i) = q(i);
        }*/
        //*qfin.
        //qfin* = q;
        //K.watch(true);
        return q;
        //cout << "ended" << endl;
        

        //B->send_q({q});

    }



};

