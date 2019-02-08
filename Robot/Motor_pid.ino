#define ENCODER_A1 2 
#define ENCODER_B1 3
#define ENCODER_A2 21 
#define ENCODER_B2 20
#define ENCODER_A3 19 
#define ENCODER_B3 18
#define MOTOR_A1 10
#define DIRECTION1 9
#define MOTOR_A2 12
#define DIRECTION2 11
#define MOTOR_A3 14
#define DIRECTION3 13
//A1 - HIGH A2 - LOW ---> count  ++
//A1 - LOW A2 - HIGH ---> count  --
// variables to store the number of encoder pulses
// for each motor
volatile long Count1 = 0,Count2 = 0,Count3 = 0;
float Kpm1=7,kim1=0.5,kdm1=0;
float Kpm2=8,kim2=0.1,kdm2=0;
float Kpm3=50,kim3=0.1,kdm3=0;
int reqpwm1,reqpwm2,reqpwm3,prevpwm1=0;
float presentvel1=0,targetvel1,error1,sumerror1=0,preverror1=0,presentvel2=0,targetvel2,error2,sumerror2=0,preverror2=0,presentvel3=0,targetvel3,error3,sumerror3=0,preverror3=0;
volatile long timer1=0,timeelapsed,timer2,timer3,timer4=0,timer5,timer6,timer7,timer8=0,timer9,timer10,timer11,timer12=0;

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
  
  targetvel1=-0;
  targetvel2=-0;
  targetvel3=0;
  
  attachInterrupt(1, EncoderEvent1, FALLING);
  attachInterrupt(3, EncoderEvent2, FALLING);
  attachInterrupt(5, EncoderEvent3, FALLING);
  
  Serial.begin(9600);
}


void loop() {
  
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
