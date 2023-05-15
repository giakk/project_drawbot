
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <drawbot/Custom.h>
#include <drawbot/Data_arduino.h> //data_arduino

// Takes from the node math.py through the topic "send_motor_velocities" the reference speed for the two motors (in rpm).

ros::NodeHandle nh; //ROS node handler setup


//------------ PIN ASSIGNEMENT ------------
// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 102.0 //Each 102 triggers the wheel makes a complete rotation (.0 since we need a float)

// Encoder output to Arduino Interrupt pin (careful only 2interupt pins on arduino 3 and 2)
#define ENC_IN_r 3
#define ENC_IN_l 2

// Encoder phase B
//#define ENC_B_r  5
//#define ENC_B_l  4

//LEFT MOTOR --> M2
// MD10C PWM connected to pin 10
#define PWM_l 6
// MD10C DIR connected to pin 12
#define DIR_l 7

//RIGHT MOTOR --> M1
// MD10C PWM connected to pin 10
#define PWM_r 5
// MD10C DIR connected to pin 12
#define DIR_r 4


//-------------- VARIABLE ASSIGNEMENT ------------------
// Pulse count from encoder
volatile long encoderValue_r = 0;
volatile long encoderValue_l = 0;

// One-second interval for measurements
   int interval = 100; //sampling time 0.1s
   int interval2 = 70; //sampling time 0.05s

// Counters for milliseconds during interval
long previousMillis = 0;
long previousInternalMillis = 0;
long currentMillis = 0;
long internalMillis = 0;

// Variable for RPM measuerment
int rpm_l = 0;
int rpm_r = 0;
double speed_encoder_l = 0;
double speed_encoder_r = 0;
double reference_speed_l = 0; //reference speed in rpm
double reference_speed_r = 0;
double motor_pwm[2]={0,0};
int motor_inputs_pwm[2]={0,0};

double dir_r;
double dir_l;

long currentTime = 0;
long previousTime = 0;
long StartRecordTime = 0;
double dt=0.0;

//end variable definition section



void messageCb( const drawbot::Custom& toggle_msg){ //just one function to take all the data from the topic Custom
  reference_speed_l = toggle_msg.motor_l; //rpm vel
  reference_speed_r = toggle_msg.motor_r; //rpm vel
}


//Ros subscribers

ros::Subscriber<drawbot::Custom> sub("send_motor_velocities", &messageCb);

//Ros publishers
drawbot::Custom rpm_values;
drawbot::Data_arduino data;
ros::Publisher pub_encoder("pub_encoder", &rpm_values); //publisher which publishes the encoder values
ros::Publisher data_arduino("data_arduino", &data); //publisher named data_arduino which publishes data through the topic Data_Arduino


void setup()
{
  //----------------ARDUINO
  // Setup Serial Monitor
  Serial.begin(9600);

  // Set encoder as input with internal pullup
  pinMode(ENC_IN_r, INPUT_PULLUP);
  pinMode(ENC_IN_l, INPUT_PULLUP);

  // Set encoder phase B as input with internal pullup
  //pinMode(ENC_B_r, INPUT_PULLUP);
  //pinMode(ENC_B_l, INPUT_PULLUP);

  // Set PWM and DIR connections as outputs
  pinMode(PWM_l, OUTPUT);
  pinMode(DIR_l, OUTPUT);

  pinMode(PWM_r, OUTPUT);
  pinMode(DIR_r, OUTPUT);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(ENC_IN_r), updateEncoder_r, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_l), updateEncoder_l, RISING);


  // Setup initial values for timer
  previousMillis = millis();
  StartRecordTime = millis();


  //---------------ROS
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_encoder); //setting the Arduino as a publisher named "pub_encoder"
  nh.advertise(data_arduino); //setting the Arduino as a publisher named "data_arduino"

}

void loop()
{

  // Update RPM value every second
  currentMillis = millis();
  internalMillis = millis();


  if (currentMillis - previousMillis > interval) { // count 0.1 second
    previousMillis = currentMillis;

    speed_encoder_r = (float)(encoderValue_r*600.0/(ENC_COUNT_REV)); //it gives you the rotations per minutes for the right wheel
    speed_encoder_l = (float)(encoderValue_l*600.0/(ENC_COUNT_REV)); //it gives you the rotations per minutes for the left wheel

    rpm_values.motor_l = speed_encoder_l;
    rpm_values.motor_r = speed_encoder_r;

    //use this to show in output the direction read by the encoders instead of the rpm value
    //rpm_values.motor_l = dir_l;
    //rpm_values.motor_r = dir_r;

    //Debug encoder value
    pub_encoder.publish( &rpm_values );   //Publishes the encoders values
    encoderValue_r = 0;
    encoderValue_l = 0;
  }

  double * motor_inputs = computeMotorInput(speed_encoder_r, reference_speed_r, speed_encoder_l, reference_speed_l);

  //direction
  if(motor_inputs[1] >= 0){
    digitalWrite(DIR_l, LOW);
  }
  else{
    digitalWrite(DIR_l, HIGH);
  }


  if(motor_inputs[0]>=0){
    digitalWrite(DIR_r, LOW);
  }else{
    digitalWrite(DIR_r, HIGH);
  }

  motor_inputs_pwm[0]=abs(motor_inputs[0])*255/1200; //conversion of the speed command from rpm to pwm
  motor_inputs_pwm[1]=abs(motor_inputs[1])*255/1200; //conversion of the speed command from rpm to pwm


  //boundary check for the motor PWM values
  boundaryCheck(motor_inputs_pwm[0], PWM_r);
  boundaryCheck(motor_inputs_pwm[1], PWM_l);

  nh.spinOnce();
  delay(5);
}

 

// FUNCTIONS

void updateEncoder_r()
{
  // Increment value for each pulse from encoder
  encoderValue_r++;
  //if(digitalRead(ENC_B_r) == HIGH){   //1 is
  //  dir_r=1;
  //}
  //else{
  //  dir_r=0;
  //}
}

void updateEncoder_l()
{
  // Increment value for each pulse from encoder
  encoderValue_l++;
 // if(digitalRead(ENC_B_l) == HIGH){   //1 is
 //   dir_l=1;
 // }
 // else{
 //   dir_l=0;
 // }
}

double * computeMotorInput(double measured_speed_r, double ref_speed_r, double measured_speed_l, double ref_speed_l) //computes rpm value needed for the motor
{

	motor_pwm[0]=ref_speed_r;
	motor_pwm[1]=ref_speed_l;

// Setting variables to send through the topic "data"
	data.err_speed_r=0;
	data.err_speed_l=0;
	//data.err_speed_r=dir_r;
	//data.err_speed_l=dir_l;
	data.err_sum_speed_r=0;
	data.err_sum_speed_l=0;
	data.ref_speed_r=ref_speed_r;
	data.ref_speed_l=ref_speed_l;
	data.controller_speed_r=motor_pwm[0];
	data.controller_speed_l=motor_pwm[1];

  //since we do not have the second phase from the encoder we have to set the sign of the velocities in this way
  if (ref_speed_r<0 && measured_speed_r>0 ){
    measured_speed_r=-measured_speed_r;

  }
  if (ref_speed_l<0 && measured_speed_l>0 ){
    measured_speed_l=-measured_speed_l;
  }
	data.mesu_speed_r=measured_speed_r;
	data.mesu_speed_l=measured_speed_l;


  if (internalMillis - previousInternalMillis > interval2){
    previousInternalMillis = internalMillis;
    data_arduino.publish( &data );

   }


	return motor_pwm;
}

void boundaryCheck(double mot_pwm, int PWM)
{
 if(mot_pwm<255 & mot_pwm>0){
	  analogWrite(PWM, mot_pwm);
 }
 else{
	  if(mot_pwm>255){
		  analogWrite(PWM, 255);
	  }
	  else{
		  analogWrite(PWM, 0);
	  }
 }
}
