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

#ifndef _RSIINTERFACE_HPP_
#define _RSIINTERFACE_HPP_


#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Time.hpp>
#include <rtt/Component.hpp>
#include <ocl/OCL.hpp>

#include <fstream>
#include <sys/stat.h>

#include <Eigen/Dense>
#include <Eigen/SVD>

#define PI 3.14159265359 // the quite well known number Pi

#define EIGEN_RUNTIME_NO_MALLOC
#define CONDITIONAL_NUMBER_LIMIT 1e3

#define JOINT_CONTROL 0
#define JOINT_PLANNING 1
#define POSITION_CONTROL 2
#define POSITION_PLANNING 3
#define ZYZ_CONTROL 4
#define ZYZ_PLANNING 5
#define AXISANGLE_PLANNING 6

#define END_EFF 0.09

#define USE_DLS_PINV

using namespace Eigen;
namespace RSIInterface{

    float a1=0.075, a2=0.27, a3=0.09;
    float d1=0.132, d4=0.295, d6=0.08+END_EFF;
    float h=1.023; //altezza base

    /// RSIInterface class
    /**
    This class provides a control interface to the KR5 using RSI.
    */

  class RSIInterface : public RTT::TaskContext{
  public:
    RSIInterface(const std::string& name);
    ~RSIInterface();

    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();
  private:

  protected:
    
    /*********
    DATAPORTS
    *********/
    //! Measured Cartesian Position (INTERNAL USE ONLY)
    RTT::InputPort<std::vector<double> > RIstPort;
    //! Measured Joint Position A (INTERNAL USE ONLY)
    RTT::InputPort<std::vector<double> > AIPosPort;
    //! Measured Joint Current (INTERNAL USE ONLY)
    RTT::InputPort<std::vector<double> > MACurPort;
    //! Commanded Joint Velocity ([rad/s])
    RTT::InputPort<std::vector<double> > JntVelInputPort;
    //! Commanded Cartesian Velocity (X,Y,Z,A,B,C; [m/s], [rad/s])
    RTT::InputPort<std::vector<double> > CartVelInputPort;

    //! Commanded Cartesian Correction (INTERNAL USE ONLY)
    RTT::OutputPort<std::vector<double> > RKorrPort;
    //! Commanded Joint Correction A (INTERNAL USE ONLY)
    RTT::OutputPort<std::vector<double> > AKorrPort;
    //! Measured Joint Position
    RTT::OutputPort<std::vector<double> > JntPosOutputPort;
    //! Measured Cartesian Position (X,Y,Z,A,B,C; [m],[rad])
    RTT::OutputPort<std::vector<double> > CartPosOutputPort;

    
    /*********
    PROPERTIES
    **********/
    //! Control type: 0 for joint control, 1 for cartesian control (do not use)
    bool      controltype;

    template<class Derived>
    static Matrix<float,3,3> skew(const MatrixBase<Derived>& v){
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);
        return (Matrix<float,3,3>() <<    0, -v[2],  v[1],
                                       v[2],     0, -v[0],
                                      -v[1],  v[0],     0).finished();

    }

    static Matrix<float,3,3> getL(const Ref<Matrix3f>& R_des,const Ref<Matrix3f>& R){
        return -0.5*(skew(R.col(0))*skew(R_des.col(0))+skew(R.col(1))*skew(R_des.col(1))+skew(R.col(2))*skew(R_des.col(2)));
    }
    
    template<class Derived>
    static Vector3f tr2eul(const MatrixBase<Derived>& T){
        float phi,psi;
        float theta=atan2(sqrt(T(0,2)*T(0,2)+T(1,2)*T(1,2)),T(2,2));
        if(theta!=0){
            phi=atan2(T(1,2),T(0,2));
            psi=atan2(T(2,1),-T(2,0));
        }else{
            phi=0;
            psi=atan2(T(0,0),T(1,0));
        }
        return (Vector3f() << phi, theta, psi).finished();
    }
    
    template<class Derived>
    static Matrix3f eul2jac(const MatrixBase<Derived>& eul){
        float sp=sin(eul(0)), cp=cos(eul(0)), st=sin(eul(1)), ct=cos(eul(1));
        return (Matrix3f() << 
                    0, -sp, cp*st,
                    0,  cp, sp*st,
                    1,   0,    ct).finished();
    }
    
    static void wrapToPi(float& f){
        f-=floor(f/(2*PI))*2*PI;
        if(f>PI)
            f-=(2*PI);
    }
   
    static void wrapToPi(Vector3f& v){
        for(int k=0;k<v.size();++k)
            wrapToPi(v[k]);
    }
    
    static float sign(float f){
        return f<0.f?-1.f:1.f;
    }
    
  private:
    //communication
    std::vector<double> RIst,AIPos,MACur,JntPosInput,JntVelInput,CartPosInput,CartVelInput,RKorr,AKorr,JntPosOutput,CartPosOutput;

    void bang_bang(float L,float t,float sdot_max,float sddot_max, float &s, float &sdot);
    void bang_bang_init(float L,float sdot_max,float sddot_max,float& T,float& T_s);
    void bang_bang_calc(float L,float t,float sddot_max,float T,float T_s, float& s,float&sdot);

    int controlMode;

    int ticks;

    //direct, differential and inverse differential kinematics methods
    //needed for pos and axisangle
    void computeKinematic();
    void computeJacobian();
    void computeFullKinematic();
    void computeFullJacobian();
    void computeDampedPseudoInverse();
    //void pseudoInverse(const MatrixXf& A, float tol=0.f);

    //method to set desired stuff
    void setDesired();

    //control methods
    void jointControl();
    void jointPlanning();
    void positionControl();
    void positionPlanning();
    void zyzControl();
    void zyzPlanning();
    void axisAnglePlanning();

    void scale(VectorXf& v);
    void saturate(VectorXf& v);

    //joint velocity command
    VectorXf cmdVel;

    //jacobian and (damped) (pseudo) inversion variables.
    //needed for pos and axisangle
    MatrixXf Jacobian;
    MatrixXf Jacobian_pinv;
    MatrixXf Winv;
    MatrixXf ut;
    MatrixXf vtt;
    VectorXf sing;
    JacobiSVD<MatrixXf> svd;

    //current joint config, desired final config and config err (needed for plain jnt and jnt)
    VectorXf q_e;
    VectorXf q_d;
    VectorXf q_err;

    //initial, current desired, current desired derivative config (needed for jnt)
    VectorXf q_init;
    VectorXf q_des;
    VectorXf qdot_des;

    //current pos, desired final pos, proportional and integral err (needed for pos & axisangle)
    VectorXf x_e;
    VectorXf x_d;
    VectorXf p_error;
    VectorXf i_error;

    //Proportional and Integral position gain (K_p also for joints)
    float K_p, K_i;

    //initial, current desired, current desired derivative pos & Cartesian velocity command (needed for pos & axisangle)
    VectorXf x_init;
    VectorXf x_des;
    VectorXf v_des;
    VectorXf v_r;

    //initial, current and final desired pose (needed 4 axisangle)
    Isometry3f T_init;
    Isometry3f T_e;
    Isometry3f T_d;

    //proportional Orientation gain
    float K_o;

    //axis-angle difference (needed 4 axisangle)
    AngleAxisf aa;
    //current desired and actual orientation (needed 4 axisangle)
    Matrix3f R_des;
    Matrix3f R_e;
    //current desired angular velocity, axis-angle error, angular error (needed 4 axisangle)
    Vector3f angv_des;
    AngleAxisf aa_err;
    Vector3f ang_err;
    //matrix used for  orientarioon control (needed 4 axisangle)
    Matrix3f matL;
    //reference angular velocity and generalized Cartesian velocity (needed 4 axisangle)
    Vector3f angv_r;
    Matrix<float,6,1> vo_r;

    Vector3f zyz_e;
    Vector3f zyz_d;
    Vector3f zyz_des;
    Vector3f zyz_init;

    float vc_max;   //maximum Cartesian velocity
    float ac_max;   //maximum Cartesian acceleration
    float va_max;   //maximum angular velocity
    float aa_max;   //maximum angular acceleration

    VectorXf v_max;  // maximum velocities for the axes [rad/s]
    VectorXf a_max;  // maximum accelerations for the axes [rad/s^2]
    std::ofstream logfile;  //log file
  };

}//namespace RSIInterface

#endif //_RSIINTERFACE_HPP_
