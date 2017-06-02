/* Dynamixel Pro Basic Example
 
 Write 2 goal positions to ID 1 dynamixel pro
 turn left and right repeatly.
 Dynamixel pro use DXL protocol 2.0
 You can also find all information about DYNAMIXEL PRO and protocol 2.0
 http://support.robotis.com/
 
               Compatibility
 CM900                  O
 OpenCM9.04             O
 
                 Dynamixel check  the movement
               AX    MX      RX    XL-320    Pro
 CM900          X      X      X        X      O
 OpenCM9.04     X      X      X        X      O
 **** OpenCM9.04 MX-Series and Pro-Series in order to drive should be 
 OpenCM 485EXP board ****
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
/* Dynamixel Pro Basic Example
 
 Write 2 goal positions to ID 1 dynamixel pro
 turn left and right repeatly.
 Dynamixel pro use DXL protocol 2.0
 You can also find all information about DYNAMIXEL PRO and protocol 2.0
 http://support.robotis.com/
 
               Compatibility
 CM900                  O
 OpenCM9.04             O
 
                 Dynamixel check  the movement
               AX    MX      RX    XL-320    Pro
 CM900          X      X      X        X      O
 OpenCM9.04     X      X      X        X      O
 **** OpenCM9.04 MX-Series and Pro-Series in order to drive should be 
 OpenCM 485EXP board ****
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define ID_NUM1 1
#define ID_NUM2 2
#define LED_GREEN 65
#define GOAL_POSITION 116
#define TORQUE_ENABLE 64

Dynamixel Dxl(DXL_BUS_SERIAL3);

float m1=10.0;
float m2=10.0;
float l1=10.0;
float l2=10.0;

float g=9.81;
float dt=0.001;
float theta1=-45*(PI/180);
float theta2=0*(PI/180);
float D_theta1=0*(PI/180);
float D_theta2=0*(PI/180);
float D2_theta1;
float D2_theta2;
float theta_real1;
float theta_real2;


float alpha = 200.0;
float des_d2q2 = 0.0;
float des_d1q2 = 0.0;
float  des_q2 = 0.0;

float kp = 10.0;
float kd = 10.0;
float M[2][2] = {{0,0}, 
                {0,0}} ;
float H[2] = {0,
                 0};
float P[2] = {0,
                 0};  
float inv = 0.0;           
float M22_bar;
float h2_bar;
float pi2_bar;
float v2;
float T1;
float T2;
float T[2] = {0, 0};
float D[2] = {0, 0};


void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  //Toque on to move dynamixel pro
  Dxl.writeByte(ID_NUM1,TORQUE_ENABLE,1);
  Dxl.writeByte(ID_NUM2,TORQUE_ENABLE,1);

}

void loop() {
  
  M[0][0]= (1/3)*m1*l1*l1+m2*l1*l1+(1/3)*m2*l2*l2+m2*l1*l2*cos(theta2);
  M[0][1]= (1/3)*m2*l2*l2+0.5*m2*l1*l2*cos(theta2);
  M[1][0]= (1/3)*m2*l2*l2+0.5*m2*l1*l2*cos(theta2);
  M[1][1]= (1/3)*m2*l2*l2;
  H[0] = (-0.5)*m2*l1*l2*sin(theta2)*D_theta1*D_theta1 + (-m2)*l1*l2*(sin(theta2))*D_theta1*D_theta2 ;
  H[1] = 0.5*m2*l1*l2*sin(theta2)*D_theta1*D_theta1;
  P[0] = ((0.5*m1)+m2)*g*l1*cos(theta1)+0.5*m2*g*l2*cos(theta1+theta2);
  P[1] = 0.5*m2*g*l2*cos(theta1+theta2);

  inv = M[1][1]/(M[0][0]*M[1][1]-M[0][1]*M[1][0]);
  
  M22_bar = M[1][1] - M[1][0]*inv*M[0][1];
  h2_bar = H[1] - M[1][0]*inv*H[0];
  pi2_bar = P[1] -M[1][0]*inv*P[0];
  
  des_q2 = 2*alpha/PI*atan(D_theta1*PI/180);
  v2 = des_d2q2 + kd*(des_d1q2-D_theta2) + kp*(des_q2-theta2);
  T1 = 0;
  T2 = M22_bar*v2 + h2_bar + pi2_bar;
  
  T[0] = T1;
  T[1] = T2;
  
  D[0] =(M[1][1]*(T[0]-H[0]-P[0]))/(M[0][0]*M[1][1]-M[0][1]*M[1][0]) - (M[0][1]*(T[1]-H[1]-P[1]))/(M[0][0]*M[1][1]-M[0][1]*M[1][0]);
  D[1] =(M[0][0]*(T[1]-H[1]-P[1]))/(M[0][0]*M[1][1]-M[0][1]*M[1][0]) - (M[1][0]*(T[0]-H[0]-P[0]))/(M[0][0]*M[1][1]-M[0][1]*M[1][0]);
  D2_theta1 = D[0];
  D2_theta2 = D[1];
  
  D_theta1 = D_theta1 + dt*D2_theta1;
  theta1 = theta1 + dt*D_theta1;
  theta_real1 = theta1/0.088;

  D_theta2 = D_theta2 + dt*D2_theta2;
  theta2 = theta2 + dt*D_theta2;
  theta_real2 = theta2/0.088;

  
  
  //Turn on green LED in DXL PRO
  Dxl.writeByte(ID_NUM1,LED_GREEN,1);
  Dxl.writeByte(ID_NUM2,LED_GREEN,1);
  //Move to goal position 151875 refer to position limit
  Dxl.writeDword(ID_NUM1,GOAL_POSITION,theta_real1);
  Dxl.writeDword(ID_NUM2,GOAL_POSITION,theta_real2);
  delay(1000);

  //Read DXL internal temperature
  SerialUSB.print(" DXL1 present position = ");
  SerialUSB.println(Dxl.readByte(ID_NUM1,132));
  SerialUSB.print(" DXL2 present position = ");
  SerialUSB.println(Dxl.readByte(ID_NUM2,132));
}


