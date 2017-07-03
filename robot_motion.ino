/*
  vmax = 95 pulses per 200 ms = 23.75 pulses per 50ms
  vmax = 2*pi*R*(23/420) cm per 50ms
  2*pi*R = 31, L = 35
  vmax = 31*(23.75/420) = 1.75 cm per 50ms
  v = (R/2)(vr+vl)
  w = (R/L)(vr-vl)
  wheel_radius = R = 31/(2*3.1418);
  v_max = (1/2)(vr+vl)
  w_max = (1/L)(vr-vl) = (1/L)((1.75 per 50ms) - 0)
  = (1.75 per 50ms)/35 = 0.05
*/

//old robot
/* L=8  R=6.5/2
  2*pi*R= 20.4*/

//new robot
//L=30 R=7.5  2*pi*R=47.1
double v_max = 0;
double w_max = 0;

#include <math.h>
#include <Timer.h>

Timer t;

//Define Variables we'll be connecting to
double motor_output_R;  //PWM right
double motor_output_L; //pwm left


#define motorR 10
#define motorL 11

void get_x_y_new_direct();
void go_to_goal();
void unicycle_to_diff_drive();
void output_shaping();

//The sample code for driving one way motor encoder
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 4;//B pin -> the digital pin 4
byte encoder0PinALast;
int duration;//the number of the pulses
boolean Direction;//the rotation direction
//The sample code for driving one way motor encoder
const byte encoder1pinA = 3;//A pin -> the interrupt pin 0
const byte encoder1pinB = 5;//B pin -> the digital pin 4
byte encoder1PinALast;
int duration2;//the number of the pulses
boolean Direction2;//the rotation direction
void EncoderInit();
void wheelSpeed();
void wheelSpeed2();
void print_every_500();
volatile long encoderR = 0, encoderL = 0;
unsigned long encoderR_old = 0, encoderL_old = 0;
int encoderR_dif = 0, encoderL_dif = 0;

char incomingByte; // for incoming serial data

double diff_x, diff_y, distance_error, theta_goal, theta_error;
double Prop_error, Int_error, Dif_error, PID_output;
double omega, velocity, vel_r, vel_l, wheel_radius;
double vel_r_in_pulses, vel_l_in_pulses, vel_rl_max, vel_rl_min;
double vel_Kp, vel_PID_output;

int get_x_y_new_Event;
int go_to_goal_Event;
int myprint;

void EncoderInit()
{
  Direction = true;//default -> Forward
  Direction2 = true;//default -> Forward
  pinMode(encoder0pinB, INPUT);
  attachInterrupt(0, wheelSpeed, CHANGE);
  pinMode(encoder1pinB, INPUT);
  attachInterrupt(1, wheelSpeed2, CHANGE);
}

void setup() {
  motor_output_R = 0;
  motor_output_L = 0;
  Serial.begin(57600);//Initialize the serial port
  EncoderInit();//Initialize the module

  pinMode(motorR, OUTPUT);
  analogWrite(motorR, motor_output_R);

  pinMode(motorL, OUTPUT);
  analogWrite(motorL, motor_output_L);

  //Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
}


double DR = 0, DL = 0;
double DC, delta_theta, radius, prev_theta, new_theta;
double x, y, x_new, y_new, x_goal, y_goal;
double Kp, Ki, Kd, prev_err, accum_err;
boolean has_reached_goal = false;
int flag = 1;
int pwm_max = 0;

void loop() {
  t.update();
  if (flag)
  {
    get_x_y_new_Event = t.every(50, get_x_y_new_direct);
    x_goal = 100;
    y_goal = 100;
    /*Kp = 0.05;
      Ki = 0.0001;
      Kd = 0.001;*/

    Kp = 0.44;
    v_max = 3.23;
    w_max = 0.69;
    velocity = 24.1;
    pwm_max = 50;

    Ki = 0;
    Kd = 0;
    prev_err = 0;
    accum_err = 0;
    has_reached_goal = false;
    go_to_goal_Event = t.every(250, go_to_goal);

//    analogWrite(motorR, 0);
//    analogWrite(motorL, 0);
    digitalWrite(13, HIGH);

    myprint = t.every(500, print_every_500);

    flag = 0;
  }
}

void go_to_goal()
{
  // every 250 ms
  diff_x = x_goal - x;
  diff_y = y_goal - y;
  distance_error = sqrt((diff_x * diff_x) + (diff_y * diff_y));
  theta_goal = atan2(diff_y , diff_x); //0.785
  theta_error = theta_goal - prev_theta;
  theta_error = atan2(sin(theta_error), cos(theta_error));
  /* Serial.print("theta_error= ");
    Serial.print(theta_error);
    Serial.print("   theta_goal= ");
    Serial.println(theta_goal);*/
  //Kp = 0.05; Ki = 0.0001; Kd = 0.001;
  Prop_error = theta_error;
  Int_error = accum_err + theta_error;
  Dif_error = theta_error - prev_err;
  PID_output = Kp * Prop_error + Ki * Int_error + Kd * Dif_error;

  accum_err = Int_error;
  prev_err = theta_error;

  omega = PID_output;
  /*Serial.print("   omega= ");
    Serial.println(omega);*/
  if (omega > w_max)
    omega = w_max;
  else if (omega < -w_max)
    omega = -w_max;

  unicycle_to_diff_drive();
  output_shaping();

  motor_output_R = map(vel_r * 100, 0, v_max * 100, 0, pwm_max);
  motor_output_L = map(vel_l * 100, 0, v_max * 100, 0, pwm_max);
  /*Serial.print("motor_output_R= ");
    Serial.print(motor_output_R);
    Serial.print("   motor_output_L= ");
    Serial.println(motor_output_L);*/
  analogWrite(motorR, motor_output_R);
  analogWrite(motorL, motor_output_L);

  if ((abs(diff_x) < 15) && (abs(diff_y) < 15))
  {
    // stop
    analogWrite(motorR, 0);
    analogWrite(motorL, 0);
    t.stop(go_to_goal_Event);
    t.stop(get_x_y_new_Event);
  }
}

//  v = (R/2)(vr+vl)
//  w = (R/L)(vr-vl)

//    kp = 0.44;
//    v_max = 3.23;
//    w_max = 0.69;
//    velocity = 24.1;
//    pwm_max = 60;

//  at pwm==60 && sample==50ms
//  encoderR_dif = 42 from 600  >>  3.297 cm in 50 ms
//  encoderL_dif = 64 from 960  >>  3.14 cm in 50 ms
//  v_max = 3.2285*R = 24.13875
//  w_max = (7.5/34)*(3.14-0) = 0.69
//  vel_r, vel_l = (2 * 24.1 + 0)/(2*7.5) = 3.21 at W = zero
//  vel_r = (2 * 24.1 + 0.69*34)/(2*7.5) = 4.8 at W = w_max
//  vel_l = (2 * 24.1 - 0.69*34)/(2*7.5) = 1.65 at W = w_max
//  shaping
//  vel_rl_max = 4.8
//  vel_rl_min = 1.65
//  v_max = 3.23
//  vel_r = 4.8 - (4.8 - 3.23) = 3.23
//  vel_l = 1.65 - (4.8 - 3.23) = 0.08
//  kp = w_max/(pi/2) = 0.44

//with output shaping
void unicycle_to_diff_drive()
{
  // 2*pi*R=31
  wheel_radius = 47.1 / (2 * 3.1418); //31/(2*3.1418);
  vel_r = (2 * velocity + omega * 34) / (2 * wheel_radius); //vel_r = (2*velocity+omega*35)/(2*wheel_radius);
  vel_l = (2 * velocity - omega * 34) / (2 * wheel_radius); //vel_l = (2*velocity-omega*35)/(2*wheel_radius);
  /* Serial.print("   omega= ");
    Serial.println(o
    mega);
    Serial.print("vel_r= ");
    Serial.print(vel_r);
    Serial.print("   vel_l= ");
    Serial.println(vel_l);*/
}

void output_shaping()
{
  vel_rl_max = max(vel_r, vel_l);
  vel_rl_min = min(vel_r, vel_l);
  if (vel_rl_max > v_max)
  {
    vel_r = vel_r - (vel_rl_max - v_max);
    vel_l = vel_l - (vel_rl_max - v_max);
  }
  if (vel_rl_min < 0)
  {
    vel_r = vel_r + (0 - vel_rl_min);
    vel_l = vel_l + (0 - vel_rl_min);
  }
  else
  {
    vel_r = vel_r;
    vel_l = vel_l;
  }
}

void get_x_y_new_direct()
{
  // every 50 ms
  encoderR_dif = encoderR - encoderR_old;
  encoderR_old = encoderR;
  encoderL_dif = encoderL - encoderL_old;
  encoderL_old = encoderL;

  DR = 47.1 * encoderR_dif / 600;   // DR = 31.0 * encoderR_dif/420.0;
  DL = 47.1 * encoderL_dif / 960;    //DL = 31.0 * encoderL_dif/420.0;
  /*Serial.print("DR= ");
    Serial.print(DR);
    Serial.print("   DL= ");
    Serial.println(DL);*/
  DC = (DR + DL) / 2.0;
  delta_theta = (DR - DL) / 34; ///(DR-DL)
  /*Serial.print("   DC= ");
    Serial.println(DC);
    Serial.print("delta_theta= ");
    Serial.print(delta_theta);
    Serial.print("   prev_theta= ");
    Serial.println(prev_theta);*/
  x_new = x + DC * cos(prev_theta);
  y_new = y + DC * sin(prev_theta);
  new_theta = prev_theta + delta_theta;
  new_theta = atan2(sin(new_theta), cos(new_theta));
  //  Serial.print("x_new= ");
  //  Serial.print(x_new);
  //  Serial.print("   y_new= ");
  //  Serial.println(y_new);
  x = x_new;
  y = y_new;
  prev_theta = new_theta;
}

//void wheelSpeed()
//{
//  int Lstate = digitalRead(encoder0pinA);
//  if((encoder0PinALast == LOW) && Lstate==HIGH)
//  {
//    int val = digitalRead(encoder0pinB);
//    if(val == LOW && Direction)
//    {
//      Direction = false; //Reverse
//    }
//    else if(val == HIGH && !Direction)
//    {
//      Direction = true;  //Forward
//    }
//  }
//  encoder0PinALast = Lstate;
//
//  if(!Direction)  encoderR++;
//  else  encoderR--;
//}
//void wheelSpeed2()
//{
//  int Lstate = digitalRead(encoder1pinA);
//  if((encoder1PinALast == LOW) && Lstate==HIGH)
//  {
//    int val = digitalRead(encoder1pinB);
//    if(val == LOW && Direction2)
//    {
//      Direction2 = false; //Reverse
//    }
//    else if(val == HIGH && !Direction2)
//    {
//      Direction = true;  //Forward
//    }
//  }
//  encoder1PinALast = Lstate;
//
//  if(!Direction2)  encoderL++;
//  else  encoderL--;
//}
void print_every_500()
{
  //  Serial.print("encoderR: ");
  //  Serial.println(encoderR);
  //  Serial.print("encoderL: ");
  //  Serial.println(encoderL);

  //  Serial.print("encoderR_dif: ");
  //  Serial.print(encoderR_dif);
  //  Serial.print("   encoderL_dif: ");
  //  Serial.println(encoderL_dif);

  Serial.print("x_new: ");
  Serial.print(x_new);
  Serial.print("   y_new: ");
  Serial.print(y_new);
  Serial.print("   theta_new: ");
  Serial.println(new_theta * 180 / 3.1418);

//  Serial.print("theta_error: ");
//  Serial.print(theta_error);
//  Serial.print("   PID_output: ");
//  Serial.print(PID_output);
//  Serial.print("   omega: ");
//  Serial.print(omega);
//  Serial.print("   vel_r: ");
//  Serial.print(vel_r);
//  Serial.print("   vel_l: ");
//  Serial.println(vel_l);
}

void wheelSpeed()
{
  if (digitalRead(encoder0pinA) == digitalRead(encoder0pinB)) {
    encoderR--;
  }
  else {
    encoderR++;
  }
}

void wheelSpeed2()
{
  if (digitalRead(encoder1pinA) == digitalRead(encoder1pinB)) {
    encoderL--;
  }
  else {
    encoderL++;
  }
}
