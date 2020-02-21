// Copyright  (C)  2009  Wilm Decre <wilm dot decre at mech dot kuleuven dot be>
// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Author: Wilm Decre, Ruben Smits
// Maintainer: Wilm Decre, Ruben Smits

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

//#include <ocl/ComponentLoader.hpp>

#include <rtt/Logger.hpp>

#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "RSIInterface.hpp"

#define delta_t 0.012 //sample period in seconds
//#define VELMAX 1 // maximum velocity for the axes [rad/s]

namespace RSIInterface{

  using namespace std;
  using namespace RTT;
 
  // Constructor 
  RSIInterface::RSIInterface(const string& name) : TaskContext(name,PreOperational),
      RIst(6,0.0),
      AIPos(6,0.0),
      JntVelInput(6,0.0),
      CartVelInput(6,0.0),
      RKorr(6,0.0),
      AKorr(6,0.0),
      JntPosOutput(6,0.0),
      CartPosOutput(6,0.0)
  {
      //Adding Input Ports
      addPort("RIst", RIstPort);
      addPort("AIPos", AIPosPort);
      addPort("MACur", MACurPort);
      addPort("nAxesOutputVelocity", JntVelInputPort);
      addPort("CartVelInput", CartVelInputPort);

      //Adding Output Ports
      addPort("AKorr", AKorrPort);
      addPort("RKorr", RKorrPort);
      addPort("nAxesSensorPosition", JntPosOutputPort);
      addPort("CartPosOutput", CartPosOutputPort);

      // Adding properties
      this->addProperty("controltype",controltype).doc("Control type: 0 for joint control, 1 for cartesian control");

      controlMode=AXISANGLE_PLANNING;
      
      int rows=(controlMode==AXISANGLE_PLANNING||controlMode==ZYZ_CONTROL||controlMode==ZYZ_PLANNING?6:3);

      cmdVel.setZero(6);

      Jacobian.setZero(rows,6);
      Jacobian_pinv.setZero(6,rows);
      Winv.setZero(6,rows);
      ut.setZero(3,3);
      vtt.setZero(6,rows);
      sing.setZero(rows);

      q_e.setZero(6);
      q_d.setZero(6);
      q_err.setZero(6);

      q_init.setZero(6);
      q_des.setZero(6);
      qdot_des.setZero(6);

      x_e.setZero(3);
      x_d.setZero(3);
      p_error.setZero(3);
      i_error.setZero(3);

      x_init.setZero(3);
      x_des.setZero(3);
      v_des.setZero(3);
      v_r.setZero(3);

      T_init.setIdentity();
      T_e.setIdentity();
      T_d.setIdentity();

      K_p=1.f;
      K_i=0.f;
      K_o=1.f;

      vc_max=0.4f;
      ac_max=0.1f;
      va_max=PI/4;
      aa_max=PI/8;

      v_max.setZero(6);
      float percent=0.5f; //percent of nominal max v
      v_max(0)=375.f*PI/180.f*percent;
      v_max(1)=300.f*PI/180.f*percent;
      v_max(2)=375.f*PI/180.f*percent;
      v_max(3)=410.f*PI/180.f*percent;
      v_max(4)=410.f*PI/180.f*percent;
      v_max(5)=660.f*PI/180.f*percent;

      a_max=2.f*v_max;

      std::string filename="kuka_log.txt";
      struct stat st;
      if(stat(filename.c_str(),&st)==0){
        std::cout << "Log file " << filename << " already exists. Do you wish to overwrite it and continue [y/n] ? " << std::flush;
        std::string overwrite;
        cin >> overwrite;
        std::cout << std::endl;
        if(!(overwrite[0]=='y'||overwrite[0]=='Y')){
            std::cout << "\n !!! OPERATION ABORTED !!! \n\n" << std::endl;
            throw -1;
        }
      }
      logfile.open(filename, std::ofstream::trunc);
  }    
  // Destructor
  RSIInterface::~RSIInterface()
  {
  }
  
  // Configure Hook
  bool RSIInterface::configureHook()
  {        
    // Setting initial value for write dataports
    RKorrPort.write(RKorr);
    AKorrPort.write(AKorr);
    JntPosOutputPort.write(JntPosOutput);
    CartPosOutputPort.write(CartPosOutput);
    return true;
  }
  
  // Start Hook
  bool RSIInterface::startHook(){
    RIstPort.read(RIst);
    AIPosPort.read(AIPos);
    MACurPort.read(MACur);
    RKorrPort.write(RKorr);
    AKorrPort.write(AKorr);
    JntPosOutputPort.write(JntPosOutput);
    CartPosOutputPort.write(CartPosOutput);
    ticks=0;
    return true;
  }
  
  // Update Hook
  void RSIInterface::updateHook(){
    //std::cout << "begin " << ticks << std::endl;

    // Get read dataports
    AIPosPort.read(AIPos);

    // set write ports to zero
    for(unsigned int j = 0;j<6;j++)
        AKorr[j] = 0.0;

    // unit conversion from degrees to radians
    JntPosOutput[0] = -AIPos[0]*PI/180.0;
    JntPosOutput[1] = -AIPos[1]*PI/180.0;   //NB: this is at PI/2 when at "home" position
    JntPosOutput[2] = -(AIPos[2]*PI/180.0 - PI/2);
    JntPosOutput[3] = -AIPos[3]*PI/180.0;
    JntPosOutput[4] = -AIPos[4]*PI/180.0;
    JntPosOutput[5] = -AIPos[5]*PI/180.0;

    //little test
    //cmdVel(0)= 5.f*PI/180.f;

    switch(controlMode){
        case JOINT_CONTROL: jointControl(); break;
        case JOINT_PLANNING: jointPlanning(); break;
        case POSITION_CONTROL: positionControl(); break;
        case POSITION_PLANNING: positionPlanning(); break;
        case ZYZ_CONTROL: zyzControl(); break;
        case ZYZ_PLANNING: zyzPlanning(); break;
        case AXISANGLE_PLANNING: axisAnglePlanning(); break;
        deafult: throw -1;
    }

    for(int i = 0; i < 6; i++)
    {
        JntVelInput[i] = cmdVel(i);
    }

    if(controltype == false)
    { 
        AKorr[0] = -180.0/PI*JntVelInput[0]*delta_t;
        AKorr[1] = -180.0/PI*JntVelInput[1]*delta_t;
        AKorr[2] = -180.0/PI*JntVelInput[2]*delta_t;
        AKorr[3] = -180.0/PI*JntVelInput[3]*delta_t;
        AKorr[4] = -180.0/PI*JntVelInput[4]*delta_t;
        AKorr[5] = -180.0/PI*JntVelInput[5]*delta_t;
    }

    logfile << x_e(0) << ' ' << x_e(1) << ' ' << x_e(2) << ' ' <<
               x_des(0) << ' ' << x_des(1) << ' ' << x_des(2) << ' ' <<
               v_des(0) << ' ' << v_des(1) << ' ' << v_des(2) << ' ' <<
               p_error(0) << ' ' << p_error(1) << ' ' << p_error(2) << ' ' <<
               v_r(0) << ' ' << v_r(1) << ' ' << v_r(2) << ' ' <<
               JntPosOutput[0] << ' ' << JntPosOutput[1] << ' ' << JntPosOutput[2] << ' ' <<
               JntPosOutput[3] << ' ' << JntPosOutput[4] << ' ' << JntPosOutput[5] << ' ' <<
               q_des[0] << ' ' << q_des[1] << ' ' << q_des[2] << ' ' <<
               q_des[3] << ' ' << q_des[4] << ' ' << q_des[5] << ' ' <<
               zyz_e(0) << ' ' << zyz_e(1) << ' ' << zyz_e(2) << ' ' <<
               zyz_des(0) << ' ' << zyz_des(1) << ' ' << zyz_des(2) << ' ' <<
               ang_err(0) << ' ' << ang_err(1) << ' ' << ang_err(2) << std::endl;

    AKorrPort.write(AKorr);
    //std::cout << "end " << ticks << std::endl;
    ticks++;
  };

  
  void RSIInterface::stopHook()
  {
      AKorr[0] = 0;
      AKorr[1] = 0;
      AKorr[2] = 0;
      AKorr[3] = 0;
      AKorr[4] = 0;
      AKorr[5] = 0;
      AKorrPort.write(AKorr);
  };
  void RSIInterface::cleanupHook(){
  };

  void RSIInterface::computeKinematic()
  {
      float s1, s2, /*s3,*/ s4, s5, s23;
      float c1, c2, /*c3,*/ c4, c5, c23;

      s1=sin(JntPosOutput[0]);
      s2=sin(JntPosOutput[1]);
      //s3=sin(JntPosOutput[2]);
      s4=sin(JntPosOutput[3]);
      s5=sin(JntPosOutput[4]);
      s23=sin(JntPosOutput[1]+JntPosOutput[2]);

      c1=cos(JntPosOutput[0]);
      c2=cos(JntPosOutput[1]);
      //c3=cos(JntPosOutput[2]);
      c4=cos(JntPosOutput[3]);
      c5=cos(JntPosOutput[4]);
      c23=cos(JntPosOutput[1]+JntPosOutput[2]);

      x_e(0)=d6*(s5*(c1*c4*c23+s1*s4)+c5*c1*s23)+d4*c1*s23+a3*c1*c23+a2*c1*c2+a1*c1;

      x_e(1)=d6*(s5*(s1*c4*c23-c1*s4)+c5*s1*s23)+d4*s1*s23+a3*s1*c23+a2*s1*c2+a1*s1;

      x_e(2)=h+d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2+d1;
  }
  
  void RSIInterface::computeJacobian()
  {
      float s1, s2, /*s3,*/ s4, s5, s23;
      float c1, c2, /*c3,*/ c4, c5, c23;

      s1=sin(JntPosOutput[0]);
      s2=sin(JntPosOutput[1]);
      //s3=sin(JntPosOutput[2]);
      s4=sin(JntPosOutput[3]);
      s5=sin(JntPosOutput[4]);
      s23=sin(JntPosOutput[1]+JntPosOutput[2]);

      c1=cos(JntPosOutput[0]);
      c2=cos(JntPosOutput[1]);
      //c3=cos(JntPosOutput[2]);
      c4=cos(JntPosOutput[3]);
      c5=cos(JntPosOutput[4]);
      c23=cos(JntPosOutput[1]+JntPosOutput[2]);


      Jacobian(0,0)=d6*(s5*(-s1*c4*c23+c1*s4)-c5*s1*s23)-d4*s1*s23-a3*s1*c23-a2*s1*c2-a1*s1;
      Jacobian(0,1)=-c1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2);
      Jacobian(0,2)=-c1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23);
      Jacobian(0,3)=d6*s5*(-c1*s4*c23+s1*c4);
      Jacobian(0,4)=d6*(c5*(c1*c4*c23+s1*s4)-s5*c1*s23);
      Jacobian(0,5)=0;

      Jacobian(1,0)=d6*(s5*(c1*c4*c23+s1*s4)+c5*c1*s23)+d4*c1*s23+a3*c1*c23+a2*c1*c2+a1*c1;
      Jacobian(1,1)=-s1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2);
      Jacobian(1,2)=-s1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23);
      Jacobian(1,3)=d6*s5*(-s1*s4*c23-c1*c4);
      Jacobian(1,4)=d6*(c5*(s1*c4*c23-c1*s4)-s5*s1*s23);
      Jacobian(1,5)=0;

      Jacobian(2,0)=0;
      Jacobian(2,1)=d6*(c4*s5*c23+c5*s23)+d4*s23+a3*c23+a2*c2;
      Jacobian(2,2)=d6*(c4*s5*c23+c5*s23)+d4*s23+a3*c23;
      Jacobian(2,3)=d6*(-s4*s5*s23);
      Jacobian(2,4)=d6*(c4*c5*s23+s5*c23);
      Jacobian(2,5)=0;
  }

  void RSIInterface::computeFullKinematic(){
      float s1, s2, /*s3,*/ s4, s5, s6, s23;
      float c1, c2, /*c3,*/ c4, c5, c6, c23;

      s1=sin(JntPosOutput[0]);
      s2=sin(JntPosOutput[1]);
      //s3=sin(JntPosOutput[2]);
      s4=sin(JntPosOutput[3]);
      s5=sin(JntPosOutput[4]);
      s23=sin(JntPosOutput[1]+JntPosOutput[2]);
      s6=sin(JntPosOutput[5]);

      c1=cos(JntPosOutput[0]);
      c2=cos(JntPosOutput[1]);
      //c3=cos(JntPosOutput[2]);
      c4=cos(JntPosOutput[3]);
      c5=cos(JntPosOutput[4]);
      c23=cos(JntPosOutput[1]+JntPosOutput[2]);
      c6=cos(JntPosOutput[5]);

      x_e(0)=d6*(s5*(c1*c4*c23+s1*s4)+c5*c1*s23)+d4*c1*s23+a3*c1*c23+a2*c1*c2+a1*c1;

      x_e(1)=d6*(s5*(s1*c4*c23-c1*s4)+c5*s1*s23)+d4*s1*s23+a3*s1*c23+a2*s1*c2+a1*s1;

      x_e(2)=h+d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2+d1;

      R_e(0,0) = s6*(s1*c4-c23*c1*s4)+c6*(c5*(s1*s4+c23*c1*c4)-s23*c1*s5);
      R_e(0,1) = c6*(s1*c4-c23*c1*s4)-s6*(c5*(s1*s4+c23*c1*c4)-s23*c1*s5);
      R_e(0,2) = s5*(s1*s4+c23*c1*c4)+s23*c1*c5;

      R_e(1,0) =-s6*(c1*c4+c23*s1*s4)-c6*(c5*(c1*s4-c23*s1*c4)+s23*s1*s5);
      R_e(1,1) =-c6*(c1*c4+c23*s1*s4)+s6*(c5*(c1*s4-c23*s1*c4)+s23*s1*s5);
      R_e(1,2) =-s5*(c1*s4-c23*s1*c4)+s23*s1*c5;

      R_e(2,0) = c6*(c23*s5+s23*c4*c5)-s23*s4*s6;
      R_e(2,1) =-s6*(c23*s5+s23*c4*c5)-s23*s4*c6;
      R_e(2,2) = s23*c4*s5-c23*c5;

      T_e.linear()=R_e;
      T_e.translation()=x_e;
  }

  void RSIInterface::computeFullJacobian(){
      float s1, s2, /*s3,*/ s4, s5, s23;
      float c1, c2, /*c3,*/ c4, c5, c23;

      s1=sin(JntPosOutput[0]);
      s2=sin(JntPosOutput[1]);
      //s3=sin(JntPosOutput[2]);
      s4=sin(JntPosOutput[3]);
      s5=sin(JntPosOutput[4]);
      s23=sin(JntPosOutput[1]+JntPosOutput[2]);

      c1=cos(JntPosOutput[0]);
      c2=cos(JntPosOutput[1]);
      //c3=cos(JntPosOutput[2]);
      c4=cos(JntPosOutput[3]);
      c5=cos(JntPosOutput[4]);
      c23=cos(JntPosOutput[1]+JntPosOutput[2]);


      Jacobian(0,0)=d6*(s5*(-s1*c4*c23+c1*s4)-c5*s1*s23)-d4*s1*s23-a3*s1*c23-a2*s1*c2-a1*s1;
      Jacobian(0,1)=-c1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2);
      Jacobian(0,2)=-c1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23);
      Jacobian(0,3)=d6*s5*(-c1*s4*c23+s1*c4);
      Jacobian(0,4)=d6*(c5*(c1*c4*c23+s1*s4)-s5*c1*s23);
      Jacobian(0,5)=0;

      Jacobian(1,0)=d6*(s5*(c1*c4*c23+s1*s4)+c5*c1*s23)+d4*c1*s23+a3*c1*c23+a2*c1*c2+a1*c1;
      Jacobian(1,1)=-s1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23+a2*s2);
      Jacobian(1,2)=-s1*(d6*(c4*s5*s23-c5*c23)-d4*c23+a3*s23);
      Jacobian(1,3)=d6*s5*(-s1*s4*c23-c1*c4);
      Jacobian(1,4)=d6*(c5*(s1*c4*c23-c1*s4)-s5*s1*s23);
      Jacobian(1,5)=0;

      Jacobian(2,0)=0;
      Jacobian(2,1)=d6*(c4*s5*c23+c5*s23)+d4*s23+a3*c23+a2*c2;
      Jacobian(2,2)=d6*(c4*s5*c23+c5*s23)+d4*s23+a3*c23;
      Jacobian(2,3)=d6*(-s4*s5*s23);
      Jacobian(2,4)=d6*(c4*c5*s23+s5*c23);
      Jacobian(2,5)=0;

      Jacobian(3,0)=0;
      Jacobian(3,1)=s1;
      Jacobian(3,2)=s1;
      Jacobian(3,3)=c1*s23;
      Jacobian(3,4)=s1*c4-c23*c1*s4;
      Jacobian(3,5)=s5*(s1*s4+c23*c1*c4)+s23*c1*c5;
      
      Jacobian(4,0)=0;
      Jacobian(4,1)=-c1;
      Jacobian(4,2)=-c1;
      Jacobian(4,3)=s1*s23;
      Jacobian(4,4)=-c1*c4-c23*s1*s4;
      Jacobian(4,5)=-s5*(c1*s4-c23*s1*c4)+s23*s1*c5;

      Jacobian(5,0)=1;
      Jacobian(5,1)=0;
      Jacobian(5,2)=0;
      Jacobian(5,3)=-c23;
      Jacobian(5,4)=-s23*s4;
      Jacobian(5,5)=s23*c4*s5-c23*c5;
  }

  //assumes Nx6 matrix with N<=6
  void RSIInterface::computeDampedPseudoInverse()
  {
      svd=Jacobian.jacobiSvd(ComputeThinU | ComputeThinV);

      sing = svd.singularValues();
      ut = svd.matrixU();
      ut.transposeInPlace();
      vtt = svd.matrixV();

      Winv=MatrixXf::Zero(6,Jacobian.rows());

      float sigMin,sigMax,sigMin2,sigMax2,damping;

      sigMin = sing(min(6,Jacobian.rows())-1);
      sigMax = sing(0);
      sigMax2=sigMax*sigMax;
      sigMin2=sigMin*sigMin;

      if ((sigMax2/sigMin2) > CONDITIONAL_NUMBER_LIMIT)
      {
         damping=(CONDITIONAL_NUMBER_LIMIT*sigMin2 - sigMax2)/(1 - CONDITIONAL_NUMBER_LIMIT);
      } else
          {
              damping=0;
          }

      for (int i=0; i<Jacobian.rows(); i++)
      {
         Winv(i,i)=sing(i) / (sing(i)*sing(i) + damping);
      }

      Jacobian_pinv=vtt*Winv*ut;
  }

  //assumes Nx6 matrix with N<=6
  /*void RSIInterface::pseudoInverse(const MatrixXf& A, float tol)
  {
      svd=A.jacobiSvd(ComputeThinU | ComputeThinV);

      sing = svd.singularValues();
      ut = svd.matrixU();
      ut.transposeInPlace();
      vtt = svd.matrixV();

      Winv.setZero(6,A.rows());

      for (int i=0; i<A.rows(); i++)
         Winv(i,i)=sing(i)>tol ? 1/sing(i) : 0;

      Jacobian_pinv=vtt*Winv*ut;
  }*/

  void RSIInterface::saturate(VectorXf& v){
      for(int i=0;i<6;++i)
          if(v[i]>v_max[i])
              v[i]=v_max[i];
          else if(v[i]<-v_max[i])
              v[i]=-v_max[i];
  }

  void RSIInterface::scale(VectorXf& v){
    float sc=(v.array().abs()/v_max.array()).maxCoeff();
    if(sc>1)
        v=v/sc;
  }

  void RSIInterface::bang_bang(float L,float t,float sdot_max,float sddot_max,float& s,float&sdot){
    float T,T_s;
    if(L>sdot_max*sdot_max/sddot_max){
        T_s=sdot_max/sddot_max;
        T=T_s+L/sdot_max;
    }else{
        T_s=sqrt(L/sddot_max);
        T=2*T_s;
    }

    if(t<T_s){
        sdot=sddot_max*t;
        s=0.5*sdot*t;
    }else if(t<T-T_s){
        sdot=sddot_max*T_s;
        s=sdot*(t-T_s/2);
    }else if(t<T){
        sdot=sddot_max*(T-t);
        s=L-0.5*sdot*(T-t);
    }else{
       sdot=0.f;
       s=L;
    }
  }
  
  void RSIInterface::bang_bang_init(float L,float sdot_max,float sddot_max,float& T,float& T_s){
    if(L>sdot_max*sdot_max/sddot_max){
        T_s=sdot_max/sddot_max;
        T=T_s+L/sdot_max;
    }else{
        T_s=sqrt(L/sddot_max);
        T=2*T_s;
    }
  }
  
  void RSIInterface::bang_bang_calc(float L,float t,float sddot_max,float T,float T_s, float& s,float&sdot){
    if(t<T_s){
        sdot=sddot_max*t;
        s=0.5*sdot*t;
    }else if(t<T-T_s){
        sdot=sddot_max*T_s;
        s=sdot*(t-T_s/2);
    }else if(t<T){
        sdot=sddot_max*(T-t);
        s=L-0.5*sdot*(T-t);
    }else{
       sdot=0.f;
       s=L;
    }
  }

  void RSIInterface::jointControl(){
    for(int k=0;k<6;++k)
        q_e(k)=JntPosOutput[k];
    computeKinematic(); //just for the record
    
    if(ticks==0){
        q_init=q_e; //just for the record
        x_init=x_e;
        setDesired();   
    }
    q_des=q_d;
    q_err=q_des-q_e;
    cmdVel = K_p*q_err;
    scale(cmdVel);
  }

  void RSIInterface::jointPlanning(){
    for(int k=0;k<6;++k)
        q_e(k)=JntPosOutput[k];
    computeKinematic(); //just for the record
        
    if(ticks==0){
        q_init=q_e;
        x_init=x_e;
        setDesired();
    }
    float t=ticks*delta_t;

    float s,sdot;
    float L=(q_d-q_init).norm();
    
    if(L>0){
        bang_bang(L,t,v_max.minCoeff(),a_max.minCoeff(),s,sdot);
        q_des=q_init+s/L*(q_d-q_init);
        qdot_des=sdot/L*(q_d-q_init);
    }else{
        q_des=q_d;
        qdot_des.setZero(3);
    }
    q_err=q_des-q_e;

    cmdVel=qdot_des+K_p*q_err;
    scale(cmdVel);
  }

  void RSIInterface::positionControl(){
    computeKinematic();
    if(ticks==0){
        x_init=x_e;
        setDesired();
    }
    
    x_des = x_d;
    p_error = x_des - x_e;
    v_r = K_p*p_error;

    computeJacobian();
#ifdef USE_DLS_PINV
    computeDampedPseudoInverse();
    cmdVel=Jacobian_pinv*v_r;
#else
    cmdVel=Jacobian.jacobiSvd(ComputeThinU | ComputeThinV).solve(v_r);
#endif
    scale(cmdVel);
  }

  void RSIInterface::positionPlanning(){
    float t=ticks*delta_t;

    computeKinematic();
    if(ticks==0){
        x_init=x_e;
        setDesired();
    }

    float s,sdot;
    float L=(x_d-x_init).norm();
    
    if(L>0){
        bang_bang(L,t,vc_max,ac_max,s,sdot);
        x_des=x_init+s/L*(x_d-x_init);
        v_des=sdot/L*(x_d-x_init);
    }else{
        x_des=x_d;
        v_des.setZero(3);
    }
    p_error = x_des - x_e;
    i_error+=p_error*delta_t;
    v_r=v_des+K_p*p_error+K_i*i_error;

    computeJacobian();
#ifdef USE_DLS_PINV
    computeDampedPseudoInverse();
    cmdVel=Jacobian_pinv*v_r;
#else
    cmdVel=Jacobian.jacobiSvd(ComputeThinU | ComputeThinV).solve(v_r);
#endif
    scale(cmdVel);
  }

  void RSIInterface::zyzControl(){
    computeFullKinematic();
    zyz_e = tr2eul(R_e);

    if(ticks==0){
        T_init=T_e;
        x_init=x_e;
        setDesired();
    }

    x_des = x_d;
    zyz_des = zyz_d;
    p_error = x_des - x_e;
    ang_err = zyz_des - zyz_e;
    v_r = K_p*p_error;
    wrapToPi(ang_err);
    angv_r = eul2jac(zyz_e)*(K_o*ang_err);
    vo_r << v_r, angv_r;

    computeFullJacobian();
#ifdef USE_DLS_PINV
    computeDampedPseudoInverse();
    cmdVel=Jacobian_pinv*vo_r;
#else
    cmdVel=Jacobian.jacobiSvd(ComputeThinU | ComputeThinV).solve(vo_r);
#endif
    scale(cmdVel);
  }

  void RSIInterface::zyzPlanning(){
    float t=ticks*delta_t;

    computeFullKinematic();
    zyz_e = tr2eul(R_e);
    if(ticks==0){
        T_init=T_e;
        x_init=x_e;
        zyz_init=zyz_e;
        setDesired();
        ang_err = zyz_d - zyz_init;
        wrapToPi(ang_err);
        zyz_d = zyz_init+ang_err;
    }
    
    float s,sdot,st,stdot;
    float Time,Time_s,Time_a,Time_as;
    float L=(T_d.translation()-T_init.translation()).norm();
    bang_bang_init(L,vc_max,ac_max,Time,Time_s);   
    float L_zyz=(zyz_d-zyz_init).norm();
    bang_bang_init(L_zyz,va_max,aa_max,Time_a,Time_as);
    float Time_max=max(Time,Time_a);
    float frac=Time/Time_max, frac_a=Time_a/Time_max;
    bang_bang_calc(L,t*frac,ac_max,Time,Time_s,s,sdot);
    sdot*=frac;
    bang_bang_calc(L_zyz,t*frac_a,aa_max,Time_a,Time_as,st,stdot);
    stdot*=frac_a;

    //float s,sdot;
    //float L=(T_d.translation()-T_init.translation()).norm();
    //bang_bang(L,t,vc_max,ac_max,s,sdot);

    if(L>0){
        x_des=x_init+s/L*(x_d-x_init);
        v_des=sdot/L*(x_d-x_init);
    }else{
        x_des=x_d;
        v_des.setZero(3);
    }
    p_error = x_des - x_e;
    i_error+=p_error*delta_t;
    v_r=v_des+K_p*p_error+K_i*i_error;

    //float st,stdot;
    //float L_zyz=(zyz_d-zyz_init).norm();
    //bang_bang(L_zyz,t,va_max,aa_max,st,stdot);

    if(L>0){
        zyz_des=zyz_init+st/L_zyz*(zyz_d-zyz_init);
        angv_des=stdot/L_zyz*(zyz_d-zyz_init);
    }else{
        zyz_des=zyz_d;
        angv_des.setZero(3);
    }
    zyz_des=zyz_init+st/L_zyz*(zyz_d-zyz_init);
    angv_des=stdot/L_zyz*(zyz_d-zyz_init);
    ang_err = zyz_des - zyz_e;
    wrapToPi(ang_err);
    angv_r = eul2jac(zyz_e)*(angv_des+K_o*ang_err);
    vo_r << v_r, angv_r;

    computeFullJacobian();
#ifdef USE_DLS_PINV
    computeDampedPseudoInverse();
    cmdVel=Jacobian_pinv*vo_r;
#else
    cmdVel=Jacobian.jacobiSvd(ComputeThinU | ComputeThinV).solve(vo_r);
#endif
    scale(cmdVel);
  }

  void RSIInterface::axisAnglePlanning(){
    float t=ticks*delta_t;

    computeFullKinematic();
    if(ticks==0){
        T_init=T_e;
        x_init=x_e;
        setDesired();
    }
    
    float s,sdot,st,stdot;
    float Time,Time_s,Time_a,Time_as;
    float L=(T_d.translation()-T_init.translation()).norm();
    bang_bang_init(L,vc_max,ac_max,Time,Time_s);   
    aa=AngleAxisf(T_init.linear().transpose()*T_d.linear());
    float theAngle=aa.angle();
    wrapToPi(theAngle);
    bang_bang_init(abs(theAngle),va_max,aa_max,Time_a,Time_as);
    float Time_max=max(Time,Time_a);
    float frac=Time/Time_max, frac_a=Time_a/Time_max;
    bang_bang_calc(L,t*frac,ac_max,Time,Time_s,s,sdot);
    sdot*=frac;
    bang_bang_calc(abs(theAngle),t*frac_a,aa_max,Time_a,Time_as,st,stdot);
    st*=sign(theAngle);
    stdot*=(sign(theAngle)*frac_a);

    //float s,sdot;
    //float L=(T_d.translation()-T_init.translation()).norm();
    //bang_bang(L,t,vc_max,ac_max,s,sdot);

    if(L>0){
        x_des=x_init+s/L*(x_d-x_init);
        v_des=sdot/L*(x_d-x_init);
    }else{
        x_des=x_d;
        v_des.setZero(3);
    }
    p_error = x_des - x_e;
    i_error+=p_error*delta_t;
    v_r=v_des+K_p*p_error+K_i*i_error;

    //float st,stdot;
    //aa=AngleAxisf(T_init.linear().transpose()*T_d.linear());
    //bang_bang(abs(theAngle),t,va_max,aa_max,st,stdot);

    R_des=T_init.linear()*AngleAxisf(st,aa.axis());
    angv_des=T_init.linear()*stdot*aa.axis();
    aa_err=AngleAxisf(R_des*R_e.transpose());
    ang_err=sin(aa_err.angle())*aa_err.axis();
    matL=getL(R_des,R_e);
    angv_r=matL.colPivHouseholderQr().solve(matL.transpose()*angv_des+K_o*ang_err);
    vo_r << v_r, angv_r;

    computeFullJacobian();
#ifdef USE_DLS_PINV
    computeDampedPseudoInverse();
    cmdVel=Jacobian_pinv*vo_r;
#else
    cmdVel=Jacobian.jacobiSvd(ComputeThinU | ComputeThinV).solve(vo_r);
#endif
    scale(cmdVel);
  }

  void RSIInterface::setDesired(){

      if(controlMode==JOINT_CONTROL||controlMode==JOINT_PLANNING){
            //q_d(0)=PI/2; q_d(1)=PI/2; q_d(2)=0.f; q_d(3)=0.f; q_d(4)=0.f; q_d(5)=0.f;
            //q_d(0)=PI/2; q_d(1)=PI/2; q_d(2)=0.f; q_d(3)=0.f; q_d(4)=0.f; q_d(5)=PI/2;
            //q_d(0)=0; q_d(1)=PI/2; q_d(2)=PI/2; q_d(3)=0.f; q_d(4)=-PI/2; q_d(5)=0;
            q_d(0)=0; q_d(1)=PI/2; q_d(2)=PI/2; q_d(3)=0.f; q_d(4)=PI/2; q_d(5)=0;
            return;
      }

      //x_d << 0.f , 0.45f+END_EFF, 1.515f;
      //x_d << 0.45f+END_EFF, 0.f, 1.515f;
      //x_d = x_init;
      //x_d << -0.1850f , 0, 1.63f+END_EFF;
      x_d << 0.f, 0.4902f, 1.3948f;
      

      if(controlMode==POSITION_CONTROL||controlMode==POSITION_PLANNING)
        return;

      T_d.translation() = x_d;
      //T_d.linear() = (Matrix3f() << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f).finished()*T_init.linear();
      //T_d.linear() = (Matrix3f() << 0.f, 0.f, 1.f, 1.f, 0.f, 0.f, 0.f, 1.f, 0.f).finished()*T_init.linear();
      //T_d.linear() << 0.f, 0.f, -1.f, 0.f, -1.f, 0.f, -1.f, 0.f, 0.f;
      ////T_d.linear() << 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, -1.f, 0.f;
      float sq=sqrt(0.5);
      T_d.linear() = (Matrix3f() << -sq, sq, 0.f, 0.5f, 0.5f, sq, 0.5f, 0.5f, -sq).finished();
      
      cout << T_init.matrix() << '\n' << endl;
      cout << T_d.matrix() << '\n' << endl;
      
      if(controlMode==AXISANGLE_PLANNING)
        return;

      zyz_d = tr2eul(T_d.linear());
  }

}//Namespace RSIInterface


ORO_CREATE_COMPONENT( RSIInterface::RSIInterface )
