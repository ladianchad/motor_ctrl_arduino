#include <m_pid.h>
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define M1_pwm 12
#define M1_dir_1 34
#define M1_dir_2 35

#define M2_pwm 8
#define M2_dir_1 36
#define M2_dir_2 37

#define EN_L 18
#define EN_R 19


void m1_move(int pwm,bool dir);
void m2_move(int pwm,bool dir);
double m1_speed();
double m2_speed();
void todo_1();
void todo_2();
void messageCb( const std_msgs::Float32MultiArray& msg);

ros::NodeHandle  nh;
m_pid motor_contrl(2,10);               // motor contrl scheduler setting, (how many motor , state nubmer)  ** max motor number is 10 , max state number is 10

std_msgs::Float32 sp_msg;
ros::Publisher pub1("L_speed", &sp_msg),pub2("R_speed", &sp_msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub("speed_set", &messageCb );

long ctrl_ratio = 10000;
int l_en =0;
int r_en =0;
int l_dir;
int r_dir;
void messageCb( const std_msgs::Float32MultiArray& msg){
  motor_contrl.set_target(0,msg.data[0]);
  motor_contrl.set_target(1,msg.data[1]);
  if(msg.data[0]>=0)
    l_dir = 1;
  else
    l_dir = -1;
  if(msg.data[1]>=0)
    r_dir = 1;
  else
    r_dir = -1;
  motor_contrl.pid_set(0,msg.data[2],msg.data[3],msg.data[4]);
  motor_contrl.pid_set(1,msg.data[5],msg.data[6],msg.data[7]);
}


void setup(){
  pinMode(M1_pwm,OUTPUT);
  pinMode(M1_dir_1,OUTPUT);
  pinMode(M1_dir_2,OUTPUT);
  pinMode(M2_pwm,OUTPUT);
  pinMode(M2_dir_1,OUTPUT);
  pinMode(M2_dir_2,OUTPUT);
  pinMode(EN_L,INPUT);
  pinMode(EN_R,INPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub1);
  nh.advertise(pub2);
  attachInterrupt(digitalPinToInterrupt(EN_L),L_encoder,CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN_R),R_encoder,CHANGE);
  Timer5.initialize(ctrl_ratio);
  Timer5.attachInterrupt(timer_hander);
  Timer5.start();
  motor_contrl.motor_move(0,m1_move);   // setting funtion to move motor
  motor_contrl.motor_move(1,m2_move);

  
  motor_contrl.get_speed(0,m1_speed);   // setting funtion to get motor speed
  motor_contrl.get_speed(1,m2_speed);

  
  motor_contrl.pid_set(0,20,0,0);     // setting motor P , I , D value   (motor index , P value, I value , D value)
  motor_contrl.pid_set(1,20,0,0);


  motor_contrl.push_todo(3,todo_1);     //  setting to do funtion (todo index , todo funtion )  ** funtion must be return void, and argument is void 
                                        //    ** 0,1 index using motor contrl , so you have to todo index over 1
}

void loop() {
  motor_contrl.ctrl_start();
}
void timer_hander(){
  motor_contrl.timer_flag = true;
  motor_contrl.state = (motor_contrl.state + 1)%motor_contrl.max_state;
}

void todo_1(){
  nh.spinOnce();
}
void m1_move(int pwm,bool dir){ 
  digitalWrite(M1_dir_1,dir);
  digitalWrite(M1_dir_2,!dir);
  analogWrite(M1_pwm,pwm);
}
void m2_move(int pwm,bool dir){
  digitalWrite(M2_dir_1,dir);
  digitalWrite(M2_dir_2,!dir);
  analogWrite(M2_pwm,pwm);
}
double m1_speed(){
  double speed = (l_en *ctrl_ratio)/90/22/60;
  sp_msg.data =speed*l_dir;
  pub1.publish(&sp_msg);
  l_en = 0;
  return speed;
}

double m2_speed(){
  double speed = (r_en *ctrl_ratio)/90/22/60;
  sp_msg.data =speed*r_dir;
  pub2.publish(&sp_msg);
  r_en = 0;

  return speed;
}

void L_encoder(){
    l_en++;
}

void R_encoder(){
    r_en++;
}
