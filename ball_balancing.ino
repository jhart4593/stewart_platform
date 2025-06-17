/* Go to the Serial Settings tab in the Maestro Control Center and apply these settings:

   Serial mode: UART, fixed baud rate
   Baud rate: 9600
   CRC disabled

*/

//Headers and objects--------------------------------------------------------------------------------------------------------------
#include <PololuMaestro.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Pixy2SPI_SS.h>
#include "math.h"
#include <BasicLinearAlgebra.h>

SoftwareSerial maestroSerial(8,9);
MicroMaestro maestro(maestroSerial);
Pixy2SPI_SS pixy;
using namespace BLA;

//Constants------------------------------------------------------------------------------------------------------------------------
//-------------
float abs_0 = 4000;   //us (microsecond) position of absolute 0 degrees is 1000us, maestro uses units of 0.25us so 4x us position
float abs_90 = 6667;  //us (microsecond) position of absolute 90 degrees is 1667us, maestro uses units of 0.25us so 4x us position

//User defined--------------------------------------------------------------------------------------------------------------------

//dimensions of platform
float s = 10.824; //length of operating leg (in)
float a = 2.0; //length of servo operating arm (in)
float alpha_0 = 0.1639; //servo arm angle home position (rad)
float h_0 = 10.7936; //platform height at home location (in)

//servomotor rotation points coordinates in base frame
BLA::Matrix<3,1> b1 = {-2.43, -4.84, 0};
BLA::Matrix<3,1> b2 = {2.43, -4.84, 0};
BLA::Matrix<3,1> b3 = {5.03, -.33, 0};
BLA::Matrix<3,1> b4 = {2.6, 3.876, 0};
BLA::Matrix<3,1> b5 = {-2.6, 3.876, 0};
BLA::Matrix<3,1> b6 = {-5.03, -.33, 0};
//BLA::Matrix<3,6> b = {b1, b2, b3, b4, b5, b6};

//platform joints home coordinates, platform frame
BLA::Matrix<3,1> p01 = {-2.1875, -2.49, 0};
BLA::Matrix<3,1> p02 = {2.1875, -2.49, 0};
BLA::Matrix<3,1> p03 = {3.0625, -.974, 0};
BLA::Matrix<3,1> p04 = {.875, 2.815, 0};
BLA::Matrix<3,1> p05 = {-.875, 2.815, 0};
BLA::Matrix<3,1> p06 = {-3.0625, -.974, 0};
//float p0[6][3] = {p01, p02, p03, p04, p05, p06};

//angle of servo arm plane relative to x-axis, rad 
float beta1 = 0.0;
float beta2 = PI;
float beta3 = 2.0944;
float beta4 = 5.2360;
float beta5 = 4.1888;
float beta6 = 1.0472;
//float beta[6] = {beta1, beta2, beta3, beta4, beta5, beta6};

// angle range for each servo going CCW
// each servo can move from absolute 0 to absolute 270 degrees
// this defines a different range 
float range[6][2] = { { -35, 55 }, { 35, -55 }, // 1, 2
                      { -35, 55 }, { 35, -55 }, // 3, 4
                      { -35, 55 }, { 35, -55 }}; // 5, 6 

//max and min angle values for each servo
float limits[6][2] = { { -10, 30 }, { -30, 10 }, // 1, 2
                      { -10, 30 }, { -30, 10 }, // 3, 4
                      { -10, 30 }, { -30, 10 }}; // 5, 6

// angle offset value for each servo 
// redefines the position of the lower range of each servo
// Example: a servo with a range [0, 90] and an offset value of 5 would have its new 0 degree position where the old 5 degree position was
float offset[6] = {10.0, -10.0, // channel #0 and channel #1
                   10.0, -10.0, // channel #2 and channel #3
                   10.0, -10.0}; // channel #4 and channel #5

//pixy2 cam
float origin[2] = {152, 100};  // X and Y co-ords of the origin
float r_platform = 133;          // the distance from the center of the platform to the corner of the platform seen from the pixy2 cam
float ball[2];                   // X and Y co-ords of the ball

//controller
float alpha_cmd[6];  //array for calculated servo angles
float err[2];       // error of the ball
float err_prev[2];  // previous error value used to calculate derivative. Derivative = (error-previous_error)/(change in time)
float d_err[2];       // derivative of the error
float kp_p = 0.0002;      // proportional constant position
float kd_p = 0.20;      // derivative constant position
float kp_o = 0.0012;      // proportional constant orientation
float kd_o = 0.67;      // derivative constant orientation
float cmd[2];         // output values (commanded position and orientation of platform)
float time_i;         // initial time
float time_f;         // final time
float dt;             //change in time
float pos_max = 0.15; // max x/y position of platform origin allowed
float or_max = 0.52; // max roll/pitch of platform origin allowed

//Arduino Loops-----------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  maestroSerial.begin(9600);
  pixy.init();
  pixy.setLamp(1, 1);
}

void loop() {
  findBall();  // finds the location of the ball
  if (ball[0] == 4004 && ball[1] == 4004) {
    // sees if ball position (x and y) is 4004, if so then the ball is not detected and platform should be in home position
    for (int i = 0; i < 6; i++) {
      alpha_cmd[i] = offset[i];
    }      
    moveServos(20, 20);
  } else {
    controller();  // calculates the proportional and derivative terms and outputs them to the platform
  }
}

// Functions -------------------------------------------------------------------------------------------------------------
void stop() {                   //ends the program and stops all of the servos
  for (int i = 0; i < 6; i++) {
    maestro.setTarget(i, 0);   //stops servo i
  }
  while (1) {}
}

float magnitude(BLA::Matrix<3,1> array) {  //finds the magnitude of an array of size 3
  float mag = 0;
  for (int i = 0; i < 3; i++) {
    mag = mag + pow(array(i,0), 2);  //adds component i of array squared
  }
  mag = sqrt(mag);
  return mag;
}

float dot(float array1[], float array2[]) {  //calculates the dot product of two arrays
  return array1[0] * array2[0] + array1[1] * array2[1] + array1[2] * array2[2];
}

float deg2rad(float angle_deg) {
  return angle_deg * PI / 180;
}

float rad2deg(float angle_rad) {
  return angle_rad * 180 / PI;
}

void moveServo(int i, float pos, int spd, int acc) {        //moves servo i to an input position at a certain speed and acceleration value
  pos = pos + offset[i];                                    //adds offset amount to the input position
  pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to ms position
  maestro.setSpeed(i, spd);                                 //sets input speed to servo i
  maestro.setAcceleration(i, spd);                          //sets input acceleration to servo i
  maestro.setTarget(i, pos);                                //drives motor to calculated position
}

void moveServos(int spd, int acc) {  //moves servos to their calculated positions at a certain speed and acceleration value
  float pos;
  for (int i = 0; i < 6; i++) {
    maestro.setSpeed(i, spd);                                 //sets input speed to servo i
    maestro.setAcceleration(i, acc);                          //sets input acceleration to servo i
    pos = alpha_cmd[i];                                       //calculated servo angle
    pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to ms position
    maestro.setTarget(i, pos);                                //drives motor to calculated position
  }
}

void findBall() {  // find the location of the ball using the pixy2 cam
       
  pixy.ccc.getBlocks();

  // If there is 1 ball detected then collect and print the data
  if (pixy.ccc.numBlocks == 1) {
    //sets current X and Y co-ords
    ball[0] = pixy.ccc.blocks[0].m_x;  // absolute X location of the ball
    ball[1] = pixy.ccc.blocks[0].m_y;  // absolute Y location of the ball
  }
  // If there are multiple balls detected, then print so
  else if (pixy.ccc.numBlocks > 1) {
    Serial.println("MULTIPLE BALLS DETECTED");
    ball[0] = 4004;  // X component of the ball
    ball[1] = 4004;  // Y component of the ball
  }
  // If there is no ball detected, then print so
  else {
    Serial.println("NO BALL DETECTED");
    ball[0] = 4004;  // X component of the ball
    ball[1] = 4004;  // Y component of the ball
  }
}

void IK(float pose_des[]){  //calculates servo arm angles given desired pose of the platform 

  //calculate rotational matrix for platform relative to the base
  float phi = pose_des[3];
  float theta = pose_des[4];
  float psi = pose_des[5];
  BLA::Matrix<3,3> Rz = {cos(psi), -sin(psi), 0, sin(psi), cos(psi), 0, 0, 0, 1};
  BLA::Matrix<3,3> Ry = {cos(theta), 0, sin(theta),0, 1, 0, -sin(theta), 0, cos(theta)};
  BLA::Matrix<3,3> Rx = {1, 0, 0, 0, cos(phi), -sin(phi), 0, sin(phi), cos(phi)};
  BLA::Matrix<3,3> R_PB = Rz*Ry*Rx;

  //calculate effective leg lengths
  //translation vector of platform frame wrt base frame
  BLA::Matrix<3,1> T_0 = {0,0,h_0};
  BLA::Matrix<3,1> T_1 = R_PB*T_0;
  BLA::Matrix<3,1> T = {-(T_1(0)),-(T_1(1)),T_1(2)};

  BLA::Matrix<3,1> l1 = T+R_PB*p01-b1;
  BLA::Matrix<3,1> l2 = T+R_PB*p02-b2;
  BLA::Matrix<3,1> l3 = T+R_PB*p03-b3;
  BLA::Matrix<3,1> l4 = T+R_PB*p04-b4;
  BLA::Matrix<3,1> l5 = T+R_PB*p05-b5;
  BLA::Matrix<3,1> l6 = T+R_PB*p06-b6;
  //l = [l1 l2 l3 l4 l5 l6];

  // servo 1
  BLA::Matrix<3,1> q1 = T + R_PB * p01;  
  float ll = magnitude(l1);
  float LL = pow(ll,2) - (pow(s,2) - pow(a,2));
  float MM = 2*a*(q1(2)-b1(2));
  float NN = 2*a*(cos(beta1)*(q1(0)-b1(0))+sin(beta1)*(q1(1)-b1(1)));  
  float al = asin(LL/sqrt(pow(MM,2)+pow(NN,2)))-atan2(NN,MM);
  //Serial.print("1:"); Serial.println(al); 
  alpha_cmd[0] = rad2deg(al); 

  // servo 2
  BLA::Matrix<3,1> q2 = T + R_PB * p02;  
  ll = magnitude(l2);
  LL = pow(ll,2) - (pow(s,2) - pow(a,2));
  MM = 2*a*(q2(2)-b2(2));
  NN = 2*a*(cos(beta2)*(q2(0)-b2(0))+sin(beta2)*(q2(1)-b2(1)));  
  al = asin(LL/sqrt(pow(MM,2)+pow(NN,2)))-atan2(NN,MM); 
  //Serial.print("2:"); Serial.println(al);
  alpha_cmd[1] = -(rad2deg(al));

  // servo 3
  BLA::Matrix<3,1> q3 = T + R_PB * p03;  
  ll = magnitude(l3);
  LL = pow(ll,2) - (pow(s,2) - pow(a,2));
  MM = 2*a*(q3(2)-b3(2));
  NN = 2*a*(cos(beta3)*(q3(0)-b3(0))+sin(beta3)*(q3(1)-b3(1)));  
  al = asin(LL/sqrt(pow(MM,2)+pow(NN,2)))-atan2(NN,MM); 
  //Serial.print("3:"); Serial.println(al);
  alpha_cmd[2] = rad2deg(al);

  // servo 4
  BLA::Matrix<3,1> q4 = T + R_PB * p04;  
  ll = magnitude(l4);
  LL = pow(ll,2) - (pow(s,2) - pow(a,2));
  MM = 2*a*(q4(2)-b4(2));
  NN = 2*a*(cos(beta4)*(q4(0)-b4(0))+sin(beta4)*(q4(1)-b4(1)));  
  al = asin(LL/sqrt(pow(MM,2)+pow(NN,2)))-atan2(NN,MM); 
  //Serial.print("4:"); Serial.println(al);
  alpha_cmd[3] = -(rad2deg(al));

  // servo 5
  BLA::Matrix<3,1> q5 = T + R_PB * p05;  
  ll = magnitude(l5);
  LL = pow(ll,2) - (pow(s,2) - pow(a,2));
  MM = 2*a*(q5(2)-b5(2));
  NN = 2*a*(cos(beta5)*(q5(0)-b5(0))+sin(beta5)*(q5(1)-b5(1)));  
  al = asin(LL/sqrt(pow(MM,2)+pow(NN,2)))-atan2(NN,MM); 
  //Serial.print("5:"); Serial.println(al);
  Serial.println(al);
  alpha_cmd[4] = rad2deg(al);
  Serial.println(alpha_cmd[4]);

  // servo 6
  BLA::Matrix<3,1> q6 = T + R_PB * p06;  
  ll = magnitude(l6);
  LL = pow(ll,2) - (pow(s,2) - pow(a,2));
  MM = 2*a*(q6(2)-b6(2));
  NN = 2*a*(cos(beta6)*(q6(0)-b6(0))+sin(beta6)*(q6(1)-b6(1)));  
  al = asin(LL/sqrt(pow(MM,2)+pow(NN,2)))-atan2(NN,MM); 
  //Serial.print("6:"); Serial.println(al);
  Serial.println(al);
  alpha_cmd[5] = -(rad2deg(al));
  Serial.println(alpha_cmd[5]);

  //Serial.println((String)"Alpha_cmd pre: [" + alpha_cmd[0]+", "+alpha_cmd[1]+", "+alpha_cmd[2]+", "+alpha_cmd[3]+", "+alpha_cmd[4]+", "+alpha_cmd[5]+"]");

  for (int i = 0; i < 6; i++) {
    if (alpha_cmd[i] > limits[i][1]){
      alpha_cmd[i] = limits[i][1];
      Serial.println("IK OVER LIMIT");
    }      
    if (alpha_cmd[i] < limits[i][0]){
      alpha_cmd[i] = limits[i][0];
      Serial.println("IK UNDER LIMIT");
    }   
    if (isnan(alpha_cmd[i])) {
      Serial.println("ERROR: CURRENT VALUES CANNOT PHYSICALLY BE EXECUTED");
      stop();
    }
  }
  //Serial.println((String)"Alpha_cmd post: [" + alpha_cmd[0]+", "+alpha_cmd[1]+", "+alpha_cmd[2]+", "+alpha_cmd[3]+", "+alpha_cmd[4]+", "+alpha_cmd[5]+"]");
}

void controller(){// calculates the PD controller values and moves the servos
  // calculates the error of the ball
  err[0] = origin[0] - ball[0];  // x component of error
  err[1] = ball[1] - origin[1];  // y component of error (opposite because pixy2 y frame is opposite from platform frame)

  time_f = millis();       // sets final time
  dt = time_f - time_i;  // change in time
  Serial.println((String)"dt: "+dt+".");
  time_i = millis();    // sets initial time
  d_err[0] = (err[0] - err_prev[0]) / dt;  // x component of derivative
  d_err[1] = (err[1] - err_prev[1]) / dt;  // y component of derivative

  // checks if derivative is NaN or INF. If so, set to zero
  if (isnan(d_err[0]) || isinf(d_err[0])) {  // x component of derivative
    d_err[0] = 0;
  }
  if (isnan(d_err[1]) || isinf(d_err[1])) {  // y component of derivative
    d_err[1] = 0;
  }

  // sets previous error to current error
  err_prev[0] = err[0];  // x component of previous error
  err_prev[1] = err[1];  // x component of previous error

  float r_ball = sqrt(pow(err[0], 2) + pow(err[1], 2));  // calculates the distance from the center of the platfrom to the ball

  if (r_ball > r_platform) {
    // checks to see if the platform should be moved to the home position
    // checks if the ball is on the platform by comparing r_ball and r_platform. If r_ball is greater than r_platform, the ball is off the platform and the platform should be in the home position
    for (int i = 0; i < 6; i++) {
      alpha_cmd[i] = offset[i];
    }   
    moveServos(20, 20);
  }

  else {  //if the ball should not be in the home position then calculate PD outputs
    // output values
    //cmd[0] = (err[0] * kp_p) + (d_err[0] * kd_p);  // calculates commanded platform x position
    //cmd[1] = (err[1] * kp_p) + (d_err[1] * kd_p);  // calculates commanded platform y position
    cmd[0] = (err[0] * kp_o) + (d_err[0] * kd_o);  // calculates commanded platform roll
    cmd[1] = (err[1] * kp_o) + (d_err[1] * kd_o);  // calculates commanded platform pitch

    //Serial.println((String)"init cmd: "+cmd[0]+","+cmd[1]+","+cmd[2]+","+cmd[3]+".");
    Serial.println((String)"init cmd: "+cmd[0]+","+cmd[1]+".");

    // //error prevention
    // for (int i = 0; i < 2; i++) {// clips position if over or under max/min platform origin position 
    //   if (cmd[i] > pos_max){
    //     cmd[i] = pos_max;
    //     Serial.println("OVER POSITION LIMIT");
    //   }
    //   if (cmd[i] < -pos_max){
    //     cmd[i] = -pos_max;
    //     Serial.println("UNDER POSITION LIMIT");
    //   }
    // }   

    //for (int i = 2; i < 4; i++) {// clips orientation if over or under max/min platform origin orientation
    for (int i = 0; i < 2; i++) {
      if (cmd[i] > or_max){
        cmd[i] = or_max;
        Serial.println("OVER ORIENTATION LIMIT");
      }
      if (cmd[i] < -or_max){
        cmd[i] = -or_max;
        Serial.println("UNDER ORIENTATION LIMIT");
      }
    } 

    //Serial.println((String)"clipped cmd: "+cmd[0]+","+cmd[1]+","+cmd[2]+","+cmd[3]+".");
    Serial.println((String)"clipped cmd: "+cmd[0]+","+cmd[1]+".");

    // desired pose of platform based on PD controller output
    //float pose_des[6] = {cmd[0],cmd[1],0,-cmd[2],cmd[3],0};
    //float pose_des[6] = {cmd[0],cmd[1],0,0,0,0};
    //float pose_des[6] = {0,0,0,-(cmd[0]),-(cmd[1]),0};
    float pose_des[6] = {0,0,0,cmd[0],cmd[1],0};

    Serial.println((String)"sign final cmd: "+pose_des[0]+","+pose_des[1]+","+pose_des[3]+","+pose_des[4]+".");

    //move platform
    IK(pose_des);
    moveServos(70, 70);
  }
}  
