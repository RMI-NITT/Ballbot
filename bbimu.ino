#include "MPU9250.h"

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

#define QuaternionOutput   false
#define RawOutput          false
#define AccOutput          false
#define GyroOutput         false
#define MagOutput          false
#define YawPitchRollOutput true

#define CalibrateMag true

int myLed = 13;

MPU9250 myIMU;

float GyroMeasError = PI * (40.0f / 180.0f);  

float beta = sqrt(3.0f / 4.0f) * GyroMeasError;

#define Kp 2.0f * 5.0f 
#define Ki 0.0f

float deltat = 0.0f;  // integration interval for both filter schemes

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float yawstart,pitchstart,rollstart;
int ch=0;
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
  
  Wire.begin();
  Serial.begin(38400);
   
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  //Serial.println("Waiting for MPU9250...");
  while(myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) != 0x71){ yield(); }
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  //Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  //Serial.print(" I should be "); Serial.println(0x71, HEX);

  // Initialize the MPU9250
  InitializeMPU9250();
}

float vx=0;
float vy=0;
float vz=0;

float yaw=0;
float pitch=0;
float roll=0;

void InitializeMPU9250 ()
{
  myIMU.MPU9250SelfTest(myIMU.SelfTest);
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();

  if(CalibrateMag)
  {
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    delay(100);
  }  
}
void loop() {

  
  myIMU.updateTime();
  
  MadgwickQuaternionUpdate(myIMU.ay, myIMU.az, myIMU.ax, myIMU.gy*PI/180.0f, myIMU.gz*PI/180.0f, myIMU.gx*PI/180.0f, myIMU.mx, -myIMU.mz, myIMU.my);
  
  myIMU.delt_t = millis() - myIMU.count;
  
  if (myIMU.delt_t > 30)
  {
    
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
      
      myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
      myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
      myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];
      
      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      
      myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
      
      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
      
      myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
      myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
      myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
      
      }
    
    /*if(QuaternionOutput)
    {
      // Print Quaternion value
      Serial.print(q[0], 5);
      Serial.print(",");
      Serial.print(q[1], 5);
      Serial.print(",");
      Serial.print(q[2], 5);
      Serial.print(",");
      Serial.println(q[3], 5);
    }

    if(AccOutput)
    {
      // Print acceleration values in g
      Serial.print(myIMU.ax, 5);
      Serial.print(",");
      Serial.print(myIMU.ay, 5);
      Serial.print(",");
      Serial.println(myIMU.az, 5);
    }

    if(GyroOutput)
    {
      // Print gyro values in dps
      Serial.print(myIMU.gx, 5);
      Serial.print(",");
      Serial.print(myIMU.gy, 5);
      Serial.print(",");
      Serial.println(myIMU.gz, 5);
    }

    if(MagOutput)
    {
      // Print mag values in ga
      Serial.print(myIMU.mx, 5);
      Serial.print(",");
      Serial.print(myIMU.my, 5);
      Serial.print(",");
      Serial.println(myIMU.mz, 5);
    }
    */
    if(YawPitchRollOutput)
    {
      float yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      float pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      float roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      //yaw   -= 15.0; // Declination at Hobart, Tasmania is about 15 degrees in March 2017
      roll  *= 180.0f / PI;

      //if(ch!=0)
      //{
      yaw=yaw+180.0f;
      pitch=pitch+180.0f;
      roll=roll+180.0f;
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.println(roll, 2);
      //}
      /*if(ch==0)
      {
        yawstart = yaw+180;
        pitchstart = pitch + 180;
        rollstart= roll + 180;
        ch++;
      }*/
    }
    
    /*if(RawOutput)
    {
      // Print acceleration values in g
      Serial.print(myIMU.ax, 5);
      Serial.print(",");
      Serial.print(myIMU.ay, 5);
      Serial.print(",");
      Serial.print(myIMU.az, 5);
      Serial.print(",");
      
      // Print gyro values in dps
      Serial.print(myIMU.gx, 5);
      Serial.print(",");
      Serial.print(myIMU.gy, 5);
      Serial.print(",");
      Serial.print(myIMU.gz, 5);
      Serial.print(",");
      
      // Print mag values in ga
      Serial.print(myIMU.mx, 5);
      Serial.print(",");
      Serial.print(myIMU.my, 5);
      Serial.print(",");
      Serial.println(myIMU.mz, 5);
    }*/

    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  }
  targetvel3= (0.333)*(vz + (1.414*((vx*cos(yaw))-(vy*sin(yaw)))));
  targetvel2= (0.333)*(vz+ ((1/1.414)*((sin(yaw)*((-1.732*vx)+vy))-(cos(yaw)*(vx+(1.732*vy))))));
  targetvel1= (0.333)*(vz+ ((1/1.414)*((sin(yaw)*((1.732*vx)+vy))+(cos(yaw)*(-vx+(1.732*vy))))));
  //targetvel3=0;
  //targetvel2 =0;
  //targetvel1= 0;
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
  //Serial.println(presentvel2);

}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * myIMU.deltat;
            q2 += qDot2 * myIMU.deltat;
            q3 += qDot3 * myIMU.deltat;
            q4 += qDot4 * myIMU.deltat;
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }
  
  
  
  
            void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrtf((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
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
