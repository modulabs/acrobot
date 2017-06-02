#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <string.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/JointState.h>
#define pi 3.141592

class Link{
public:
    float m;
    float l;
    float lc;
    float Ic;
    float b;
public:
    Link(float _m, float _l, float _lc, float _Ic, float _b){
     m = _m;
     l = _l;
     lc = _lc;
     Ic = _Ic;
     b = _b;
    }

};

class Acrobot{
private:
    Link link1;
    Link link2;

public:
    Eigen::Matrix2f H, C;
    Eigen::RowVector2f G, B;
    Eigen::Vector4f X;

public:
    Acrobot(const Link& _link1, const Link& _link2)
        :link1(_link1)
        ,link2(_link2)
    {}

    void manipulator_dynamics(float q1, float q2, float q1_dot, float q2_dot){

     float g = 9.81;
     float m1 = link1.m;
     float m2 = link2.m;
     float l1 = link1.l;
     float l2 = link2.l;
     float lc1 = link1.lc;
     float lc2 = link2.lc;
     float I1 = 1/2*m1*l1*l1 + link1.Ic;
     float I2 = 1/2*m2*l2*l2 + link2.Ic;
     float b1 =  link1.b;
     float b2 = link2.b;

////   H(q)q_ddot + C(q,q_dot)q_dot + G(q) = Bu

     float m2l1lc2 = m2*l1*lc2;
     Eigen::Vector2f c,s;
     float s12;

     c << cos(q1), cos(q2);
     s << sin(q1), sin(q2);
     s12 = sin(q1+q2);

     float h12 = I2 + m2l1lc2*c(1);


     H << I1 + I2 + m2*l1*l1 + 2*m2l1lc2*c(1),   h12,
          h12                                ,   I2;

     C << -2*m2l1lc2*s(1)*q2_dot,   -m2l1lc2*s(1)*q2_dot,
          m2l1lc2*s(1)*q1_dot   ,              0;

     G << (m1*lc1*s(0) + m2*(l1*s(0) + lc2*s12)*g),
                        m2*lc2*s12 * g ;
     B << 0,
          1;


    }

    void listen_joint_state(const sensor_msgs::JointState & state){

        X(0) = state.position[0];
        X(1) = state.position[1];
        X(2) = state.velocity[0];
        X(3) = state.velocity[1];

    }

    };


int main(int argc, char **argv)
{

  ros::init(argc, argv, "acrobot_control");
  ros::NodeHandle n;
  ros::Rate r(1200);

  Link link1(1,1,0.5,0.083,0.1);
  Link link2(2,2,1,0.33,0.1);
  Acrobot acrobot(link1,link2);

  float q1, q2, q1_dot,q2_dot,q1_ddot,q2_ddot;
  float des_q2=0; float des_q2_dot=0; float des_q2_ddot = 0;
  float des_u = 0; float u = 0; float v2 = 0;

//  q1 = 45*pi/180;
//  q2 = 0*pi/180;
//  q1_dot = 0*pi/180;
//  q2_dot = 0*pi/180;

  float alpha = 22;
  float kp = 50;
  float kd = 50;

  Eigen::Matrix2f H, C;
  Eigen::Vector2f G, B, P;
  Eigen::Vector2f q_dot;
  Eigen::Vector4f X_dot; // state variable, q1 q2 q1_dot q2_dot
  Eigen::RowVector4f K_lqr;
//  X << q1, q2, q1_dot, q2_dot;


  ros::Subscriber sub = n.subscribe("/rrbot/joint_states", 1000, &Acrobot::listen_joint_state, &acrobot);
//  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, boost::bind(&Acrobot::listen_joint_state, this, _1));
  ros::Publisher pub = n.advertise<std_msgs::Float64>("/rrbot/joint2_torque_controller/command",1000);


  while(ros::ok()){
  ros::spinOnce();
  std_msgs::Float64 u_;

  q1 = acrobot.X(0);
  q2 = acrobot.X(1);
  q1_dot = acrobot.X(2);
  q2_dot = acrobot.X(3);

  acrobot.manipulator_dynamics(q1, q2, q1_dot, q2_dot);    //(q1, q2, q1_dot, q2_dot)

  q_dot << q1_dot, q2_dot;

  //   H(q)q_ddot + C(q,q_dot)q_dot + G(q) = Bu
  H = acrobot.H;
  C = acrobot.C;
  G = acrobot.G;
  B = acrobot.B;

  des_q2 = 2*alpha/pi*atan(q1_dot);
  v2 = des_q2_ddot + kd*(des_q2_dot - q2_dot) + kp*(des_q2 - q1);


  P = B*u-C*q_dot-G;

//  ROS_INFO("H : %f %f %f %f", H(0,0), H(0,1), H(1,0), H(1,1));
//  ROS_INFO("C : %f %f %f %f", C(0,0), C(0,1), C(1,0), C(1,1));
//  ROS_INFO("G : %f %f %f %f", G(0),G(1));
//  ROS_INFO("B : %f %f %f %f", B(0),B(1));


//    ROS_INFO("this is output value : %f %f %f %f ", B(0,0), C(1), q1_ddot, u);


  q1_ddot = (P(0)-H(0,1)*v2)/H(0,0);

//    Swing-up input
  u = H(1,0)*q1_ddot + H(1,1)*v2+C(1,0)*q1_dot + C(1,1)*q2_dot+G(1);

    //  LQR input
  K_lqr << -650.4009, -289.0746, -287.1833, -140.0615;
  u = -K_lqr * acrobot.X;

  u_.data = u;
  ROS_INFO("this is output value : %f %f %f %f ", P(0), P(1), q1_ddot, u);
  pub.publish(u_);

  }


  return 0;
}
