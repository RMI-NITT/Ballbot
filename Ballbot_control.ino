#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[1024]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3],yprnew[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#define ENCODER_A1 2 
#define ENCODER_B1 3
#define ENCODER_A2 48 
#define ENCODER_B2 19
#define ENCODER_A3 49 
#define ENCODER_B3 18
#define MOTOR_A1 10
#define DIRECTION1 9
#define MOTOR_A2 12
#define DIRECTION2 11
#define MOTOR_A3 7
#define DIRECTION3 13
//A1 - HIGH A2 - LOW ---> count  ++
//A1 - LOW A2 - HIGH ---> count  --
// variables to store the number of encoder pulses
// for each motor
volatile long Count1 = 0,Count2 = 0,Count3 = 0;
float Kpm1=7,kim1=0.5,kdm1=0;
float Kpm2=8,kim2=0.1,kdm2=0;
float Kpm3=4,kim3=0.1,kdm3=0;
int reqpwm1,reqpwm2,reqpwm3,prevpwm1=0;
float presentvel1=0,targetvel1,error1,sumerror1=0,preverror1=0,presentvel2=0,targetvel2,error2,sumerror2=0,preverror2=0,presentvel3=0,targetvel3,error3,sumerror3=0,preverror3=0;
volatile long timer1=0,timeelapsed,timer2,timer3,timer4=0,timer5,timer6,timer7,timer8=0,timer9,timer10,timer11,timer12=0;
volatile long t1=0,t2=0;
int flag = 0;
float pitcherror = 0, rollerror = 0;
float set_p = 0, set_r = 0, sum_pitch = 0, sum_roll = 0, prev_pitch = 0, prev_roll = 0;
float bias=0;
int kp_bb=0, kd_bb=0, ki_bb=0;
float conv;
float dynamicpitch=0,dynamicroll=0;
int flag2=0;

void setup() {
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(DIRECTION1, OUTPUT);
  
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(DIRECTION2, OUTPUT);
  
  pinMode(ENCODER_A3, INPUT);
  pinMode(ENCODER_B3, INPUT);
  pinMode(MOTOR_A3, OUTPUT);
  pinMode(DIRECTION3, OUTPUT);
  
  attachInterrupt(1, EncoderEvent1, FALLING);
  attachInterrupt(4, EncoderEvent2, FALLING);
  attachInterrupt(5, EncoderEvent3, FALLING);
  
  Serial.begin(9600);

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        //mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));}

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    t1=micros();
}

float vx=0;
float vy=0;
float vz=0;

float yaw=0,yaw1=0;
float pitch=0;
float roll=0;



void loop() {
  
  if (!dmpReady) return;
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));
    
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        
        
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("yrp\t");
            /*
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
            yprnew[0]=ypr[0] * 180/M_PI + 180;
            yprnew[1]=ypr[1] * 180/M_PI + 180;
            yprnew[2]=ypr[2] * 180/M_PI + 180;

            

            
            
            //Serial.write(uint8_t(yprnew[0]));
        #endif


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

  t2 = micros();
  if ((t2-t1)>60000000 && flag==0)
  {
    bias = yprnew[0];
    flag=1;
    Serial.println("BIAS SET" + String(bias));
    t1=0;
  }

  else if (flag!=1 || (t2-t1)<60000000)
  {
    bias = yprnew[0];
  }
   //Serial.print(yprnew[0]-bias);
    //Serial.print("\t");
    //Serial.print(yprnew[1]);
    //Serial.print("\t");
    //Serial.println(yprnew[2]);
  //if((yprnew[0]-bias)>0)
    yaw1 = (yprnew[0]-bias);
    yaw = (yprnew[0]-bias)*M_PI/180;
  //else
    //yaw = (360 + (yprnew[0]-bias))*M_PI/180;
  pitch = (yprnew[2]);
  roll = (yprnew[1]);
  //Serial.println(yprnew[1]);
  if (flag==1)
  {
  if(flag2==0)
  {
    dynamicpitch=pitch;
    dynamicroll=roll;
    flag2=1;
  }
  if(abs(dynamicpitch-0)<=abs(dynamicpitch-360))
  {     if (pitch<=90 && pitch>dynamicpitch)
          {
          set_p=dynamicpitch;
          pitcherror = pitch-set_p;
          }
      
        else if ((pitch<=dynamicpitch && pitch>=0))
          {set_p = dynamicpitch;
          pitcherror = set_p-pitch;}
          
        else if (pitch<=360 && pitch>=270)
          {set_p=360 + dynamicpitch;
          pitcherror = pitch-set_p;}
  }

  if(abs(dynamicpitch-0)>abs(dynamicpitch-360))
  {     if (pitch<=90 && pitch>=0)
          {set_p=dynamicpitch-360;
          pitcherror = pitch-set_p;}
      
        else if ((pitch<=360 && pitch>=dynamicpitch))
          {set_p = dynamicpitch;
          pitcherror = pitch - set_p;}
          
        else if (pitch<dynamicpitch && pitch>=270)
          {set_p=dynamicpitch;
          pitcherror = pitch - set_p;}
  }

  if(abs(dynamicroll-0)<=abs(dynamicroll-360))
  {     if (roll<=90 && roll>=dynamicroll)
          {set_r=dynamicroll;
          rollerror = set_r - roll;}
      
        else if ((roll<dynamicroll && roll>=0))
          {set_r = dynamicroll;
          rollerror = set_r-roll;}
          
        else if (roll<=360 && roll>=270)
          {set_r=dynamicroll;
          rollerror = 360 - roll + set_r;}
  }

  if(abs(dynamicroll-0)>abs(dynamicroll-360))
  {     if (roll<=90 && roll>=0)
          {set_r=dynamicroll-360;
          rollerror = set_r - roll;}
      
        else if ((roll<=360 && roll>=dynamicroll))
          {set_r = dynamicroll;
          rollerror = set_r - roll;}
          
        else if (roll<dynamicroll && roll>=270)
          {set_r=dynamicroll;
          rollerror = set_r - roll;}
  }

  vx = kp_bb*(pitcherror) + ki_bb*(sum_pitch) + kd_bb*(pitcherror-prev_pitch);
  vy = kp_bb*(rollerror) + ki_bb*(sum_roll) + kd_bb*(rollerror-prev_roll);

  
  targetvel3= (0.333)*(vz + (1.414*((vx*cos(yaw))-(vy*sin(yaw)))));
  targetvel2= (0.333)*(vz+ ((1/1.414)*((sin(yaw)*((-1.732*vx)+vy))-(cos(yaw)*(vx+(1.732*vy))))));
  targetvel1= (0.333)*(vz+ ((1/1.414)*((sin(yaw)*((1.732*vx)+vy))+(cos(yaw)*(-vx+(1.732*vy))))));
  }

  /*Serial.print(pitcherror);
  Serial.print('\t');
  Serial.print(rollerror);
  Serial.print('\t');
  Serial.print(pitch);
  Serial.print('\t');
  Serial.println(roll);
  */
  //targetvel3=20;
  //targetvel2 =20;
  //targetvel1= 20;s
  //Serial.print(targetvel1);
  //Serial.print(targetvel2);
  //Serial.println(targetvel3);
  if(targetvel1>0)
  {
  error1 = targetvel1-presentvel1;
  digitalWrite(DIRECTION1,LOW);
  }
  else if(targetvel1<0)
  {
  error1 = -targetvel1-presentvel1;
  digitalWrite(DIRECTION1,HIGH);
  }

  reqpwm1 = (Kpm1 * error1)+(kim1 * sumerror1)+(kdm1 * (error1 - preverror1)) ;
  if(reqpwm1>255)
  reqpwm1=255;
  if (reqpwm1<0)
  reqpwm1=0;
  analogWrite(MOTOR_A1,reqpwm1);
  

  if(targetvel2>0)
  {
  error2 = targetvel2-presentvel2;
  digitalWrite(DIRECTION2,LOW);
  }
  else if(targetvel2<0)
  {
  error2 = -targetvel2-presentvel2;
  digitalWrite(DIRECTION2,HIGH);
  }

  reqpwm2 = (Kpm2 * error2)+(kim2 * sumerror2)+(kdm2 * (error2 - preverror2)) ;
  if(reqpwm2>255)
  reqpwm2=255;
  if (reqpwm2<0)
  reqpwm2=0;
  analogWrite(MOTOR_A2,reqpwm2);

  
  
  if(targetvel3>0)
  {
  error3 = targetvel3-presentvel3;
  digitalWrite(DIRECTION3,LOW);
  }
  else if(targetvel3<0)
  {
  error3 = -targetvel3-presentvel3;
  digitalWrite(DIRECTION3,HIGH);
  }

  reqpwm3 = (Kpm3 * error3)+(kim3 * sumerror3)+(kdm3 * (error3 - preverror3)) ;
  if(reqpwm3>255)
  reqpwm3=255;
  if (reqpwm3<0)
  reqpwm3=0;
  analogWrite(MOTOR_A3,reqpwm3);


  if (presentvel3==0||presentvel2==0||presentvel1==0)
  delay(15);
  presentvel1 = (1/float(timer4))*60000000*10/14760;
  sumerror1+=error1;
  preverror1=error1;
  
  presentvel2 = (1/float(timer8))*60000000*10/14760;
  sumerror2+=error2;
  preverror2=error2;
  
  presentvel3 = (1/float(timer12))*60000000*10/14760;
  sumerror3+=error3;
  preverror3=error3;

  if(timer4==0)
    presentvel1=0;
  if(timer8==0)
    presentvel2=0;
  if(timer12==0)
    presentvel3=0;  


  sum_pitch+=pitcherror;
  prev_pitch=pitcherror;

  sum_roll+=rollerror;
  prev_roll=rollerror;
  
  //Serial.println(targetvel3);
  //Serial.println(presentvel3);

}
void EncoderEvent1() {
  if(Count1==0)
  timer1=micros();
  
  
  if (digitalRead(ENCODER_A1) == HIGH) 
  {
      Count1++;
  }
  else if (digitalRead(ENCODER_A1) == LOW) {
      Count1--;
  }
  if(Count1==5)
  { timer2=micros();
    timer4=timer2-timer1;
    Count1 =0;
  }
  else if(Count1==-5)
  {
    timer3=micros();
    timer4=timer3-timer1;
    Count1=0;
  }
}

void EncoderEvent2() {
  if(Count2==0)
  timer5=micros();
  
  
  if (digitalRead(ENCODER_A2) == HIGH) 
      Count2++;
  else if (digitalRead(ENCODER_A2) == LOW) 
      Count2--;
  if(Count2==5)
  { timer6=micros();
    timer8=timer6-timer5;
    Count2 =0;
  }
  else if(Count2==-5)
  {
    timer7=micros();
    timer8=timer7-timer5;
    Count2=0;
  }
}
void EncoderEvent3() {
  if(Count3==0)
  timer9=micros();
  
  
  if (digitalRead(ENCODER_A3) == HIGH) 
    Count3++;
    else if (digitalRead(ENCODER_A3) == LOW) {
      Count3--;
  }
  if(Count3==5)
  { timer10=micros();
    timer12=timer10-timer9;
    Count3 =0;
  }
  else if(Count3==-5)
  {
    timer11=micros();
    timer12=timer11-timer9;
    Count3=0;
  }
}
