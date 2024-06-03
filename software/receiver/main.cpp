#include <Arduino.h>
#include <RF24.h>
#include <RF24Network.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>

RF24 radio(30, 31);  //CE and CSN pins
const byte address[6] = "00001";


#define PWM_LF  7     //左前PWM引脚
#define PWM_LB  8     //左后PWM引脚
#define PWM_RF  9     //右前PWM引脚
#define PWM_RB  10    //右后PWM引脚
#define INA_LF  26           
#define INB_LF  27    
#define INA_LB  28           
#define INB_LB  29   
#define INA_RF  22           
#define INB_RF  23   
#define INA_RB  24           
#define INB_RB  25

int pwma = 0;

//声明舵机对象
Servo servo_arm;
Servo servo_wrist;
Servo servo_base;
Servo servo_rotation;
Servo servo_actuator;

int i = 0;

double servo_actuator_value = 0;

const int base_height = 102;    //底座高度
const int arm_length = 140;     //大臂长度
const int wrist_length = 150;   //小臂长度
double x = 0;
double y = -30;
double z = 102;
struct robot_arm 
{
    double base_value;
    double arm_value;
    double wrist_value;
};
robot_arm data_1;

int joyMiddle = 130;           //摇杆中心位置
int joyDeadzone = 20;          //摇杆死区

struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
  byte roll;
  byte pitch;
};
Data_Package data;

void forward() 
{
  digitalWrite(INA_LF, LOW );
  digitalWrite(INB_LF, HIGH);
  analogWrite (PWM_LF, pwma);

  digitalWrite(INA_LB, LOW );
  digitalWrite(INB_LB, HIGH);
  analogWrite (PWM_LB, pwma);

  digitalWrite(INA_RF, LOW );
  digitalWrite(INB_RF, HIGH);
  analogWrite (PWM_RF, pwma);

  digitalWrite(INA_RB, LOW );
  digitalWrite(INB_RB, HIGH);
  analogWrite (PWM_RB, pwma);
}

void back() 
{
  digitalWrite(INA_LF, HIGH);
  digitalWrite(INB_LF, LOW );
  analogWrite (PWM_LF, pwma);

  digitalWrite(INA_LB, HIGH);
  digitalWrite(INB_LB, LOW );
  analogWrite (PWM_LB, pwma);

  digitalWrite(INA_RF, HIGH);
  digitalWrite(INB_RF, LOW );
  analogWrite (PWM_RF, pwma);

  digitalWrite(INA_RB, HIGH);
  digitalWrite(INB_RB, LOW );
  analogWrite (PWM_RB, pwma);
}

void left() 
{
  digitalWrite(INA_LF, HIGH);
  digitalWrite(INB_LF, LOW );
  analogWrite (PWM_LF, pwma);

  digitalWrite(INA_LB, HIGH);
  digitalWrite(INB_LB, LOW );
  analogWrite (PWM_LB, pwma);

  digitalWrite(INA_RF, LOW );
  digitalWrite(INB_RF, HIGH);
  analogWrite (PWM_RF, pwma);

  digitalWrite(INA_RB, LOW );
  digitalWrite(INB_RB, HIGH);
  analogWrite (PWM_RB, pwma);
}

void right() 
{
  digitalWrite(INA_LF, LOW );
  digitalWrite(INB_LF, HIGH);
  analogWrite (PWM_LF, pwma);

  digitalWrite(INA_LB, LOW );
  digitalWrite(INB_LB, HIGH);
  analogWrite (PWM_LB, pwma);

  digitalWrite(INA_RF, HIGH);
  digitalWrite(INB_RF, LOW );
  analogWrite (PWM_RF, pwma);

  digitalWrite(INA_RB, HIGH);
  digitalWrite(INB_RB, LOW );
  analogWrite (PWM_RB, pwma);
}

void stop() 
{
  digitalWrite(INA_LF, LOW );
  digitalWrite(INB_LF, LOW );
  analogWrite (PWM_LF, pwma);

  digitalWrite(INA_LB, LOW );
  digitalWrite(INB_LB, LOW );
  analogWrite (PWM_LB, pwma);

  digitalWrite(INA_RF, LOW );
  digitalWrite(INB_RF, LOW );
  analogWrite (PWM_RF, pwma);

  digitalWrite(INA_RB, LOW );
  digitalWrite(INB_RB, LOW );
  analogWrite (PWM_RB, pwma);
}

// void servo_initial()
// {
//   servo_base.write(pos_bottom);
//   servo_arm.write(pos_left);
//   servo_wrist.write(pos_right);
//   servo_base_value = pos_bottom;
//   servo_arm_value = pos_left;
//   servo_wrist_value = pos_right;
// }

void calculateAngles() {
    //底座角度
    double base_angle = atan2(x, y);

    //大臂角
    double vertical_projection = sqrt(pow(x, 2) + pow(y, 2));
    double short_edge = abs(base_height - z);
    double hypotenuse = sqrt(pow(short_edge, 2) + pow(vertical_projection, 2));
    double arm_angle1 = acos((pow(arm_length, 2) + pow(hypotenuse, 2) - pow(wrist_length, 2)) / (2 * arm_length * hypotenuse));
    double arm_angle2;

    if (base_height == z) {
        arm_angle2 = 0; // + PI / 4;
    } else if (base_height > z) {
        arm_angle2 = atan2(vertical_projection, short_edge ) - PI / 2;
    }
     else if (base_height < z) {
        arm_angle2 = atan2(short_edge,vertical_projection);
    }
    double arm_angle = (arm_angle1 + arm_angle2) * 180 /PI;

    //小臂角
    double wrist_angle = acosf((pow(arm_length, 2) + pow(wrist_length, 2) - pow(hypotenuse, 2)) / (2.0 * arm_length * wrist_length)) * 180 /PI + arm_angle - 90;
    double base_value = base_angle * 180 / PI;

    data_1.base_value = base_value - 45;
    data_1.arm_value = arm_angle + 15;//left   
    data_1.wrist_value =60-wrist_angle;//right 
}

void setup()
{
  SPI.begin();
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver

  servo_base.attach(2);
  servo_arm.attach(3);
  servo_wrist.attach(4);
  servo_rotation.attach(5);
  servo_actuator.attach(6);

  // servo_initial();
  
  pinMode(PWM_LF, OUTPUT);
  pinMode(INA_LF, OUTPUT);
  pinMode(INB_LF, OUTPUT);

  pinMode(PWM_LB, OUTPUT);
  pinMode(INA_LB, OUTPUT);
  pinMode(INB_LB, OUTPUT);

  pinMode(PWM_RF, OUTPUT);
  pinMode(INA_RF, OUTPUT);
  pinMode(INB_RF, OUTPUT);

  pinMode(PWM_RB, OUTPUT);
  pinMode(INA_RB, OUTPUT);
  pinMode(INB_RB, OUTPUT);
}

void loop() 
{
  while ( radio.available() ) // Is there any incoming data?
  {    
    radio.read(&data, sizeof(Data_Package)); // Read the incoming data
    if(data.tSwitch1 == 0)      //小车运动部分
    {
      if(data.j1PotY > joyMiddle + joyDeadzone)
       {
        pwma = (data.j1PotY - joyMiddle + joyDeadzone);
        if(pwma > 252)
        {
          pwma = 252;
        }
        forward();
        Serial.println("前进");
       }
       else if(data.j1PotY < joyMiddle - joyDeadzone)
       {
        pwma = (joyMiddle - joyDeadzone - data.j1PotY);
        if(pwma > 252)
        {
          pwma = 252;
        }
        back();
        Serial.println("后退");
       }
       else if(data.j1PotX > joyMiddle + joyDeadzone)
       {
        pwma = (data.j1PotX - joyMiddle + joyDeadzone);
        if(pwma > 252)
        {
          pwma = 252;
        }
        left();
        Serial.println("左转");
       }
       else if(data.j1PotX < joyMiddle - joyDeadzone)
       {
        pwma = (joyMiddle - joyDeadzone - data.j1PotX);
        if(pwma > 252)
        {
          pwma = 252;
        }
        right();
        Serial.println("右转");
       }
       else if(data.button1 == 0)
       {
       pwma = 20;
       forward();
       delay(1000);
       stop();
       Serial.println("前进延时");
       }
       else
       {
        pwma=0;
        stop();
        Serial.println(pwma);
       }

    Serial.println(pwma);
    }
    else if(data.tSwitch1 == 1)    //机械臂部分
    {
      if(data.tSwitch2 == 0)           
      {
        if(data.j1PotY < joyMiddle + joyDeadzone)
        {
          data_1.arm_value -= 7;
        }
        else if(data.j1PotY > joyMiddle - joyDeadzone)
        {
          data_1.arm_value += 7;
        }
        else if(data.j1PotX > joyMiddle + joyDeadzone)
        {
          data_1.base_value -= 7;
        }
        else if(data.j1PotX < joyMiddle - joyDeadzone)
        {
          data_1.base_value += 7;
        }
        else if(data.j2PotY > joyMiddle + joyDeadzone)
        {
          data_1.wrist_value -= 7;
        }
        else if(data.j2PotY < joyMiddle - joyDeadzone)
        {
          data_1.wrist_value += 7;
        }
        else if(data.button2 == 0)
        {
          servo_actuator_value = 20;
          Serial.println("末端执行器一");
        }
        else if(data.button3 == 0)
        {
          servo_actuator_value = 110;
          Serial.println("末端执行器二");
        }
        else if(data.button4 == 0)
        {
          servo_actuator_value = 270;
          Serial.println("末端执行器三");
        }
        else if(data.j1Button == 0 && data.j2Button == 1)
        {
          servo_rotation.write(0);
          Serial.println("末端执行器正转");
        }
        else if(data.j2Button == 0 && data.j1Button == 1)
        {
          servo_rotation.write(180);
          Serial.println("末端执行器反转");
        }
        else if(data.j1Button == 1 && data.j2Button == 1)
        {
          servo_rotation.write(90);
          // Serial.println("停止");
        }
        
        // else if (data.button2 == 0)
        // {
        //   servo_actuator_value = 20;
        //   Serial.println(data.button2);
        // }
        // else if (data.button3 == 0)
        // {
        //   servo_actuator_value = 110;
        // }
        // else if (data.button4 == 0)
        // {
        //   servo_actuator_value = 270;
        // }
        
      }
      else if(data.tSwitch2 == 1)
      {
        if(data.j1PotY > joyMiddle + joyDeadzone)
        {
          x += 1;
        }
        else if(data.j1PotY < joyMiddle - joyDeadzone)
        {
          x -= 1;       
        } 
        else if(data.j1PotX > joyMiddle + joyDeadzone)
        {
          y -= 1;        
        }
        else if(data.j1PotX < joyMiddle - joyDeadzone)
        {
          y += 1;       
        }
        else if(data.j2PotY > joyMiddle + joyDeadzone)
        {
          z += 1;       
        }
        else if(data.j2PotY < joyMiddle - joyDeadzone)
        {
          z -= 1;       
        }
        else if(data.button1 == 0)
        {
          x = 25;
          y = -15;
          z = 102;
          delay(100);
          x = 50;
          y = 0;
          z = 102;
          Serial.println("复位");
        }
        else if(data.button2 == 0)
        {
          servo_actuator_value = 20;
          Serial.println("末端执行器一");
        }
        else if(data.button3 == 0)
        {
          servo_actuator_value = 110;
          Serial.println("末端执行器二");
        }
        else if(data.button4 == 0)
        {
          servo_actuator_value = 270;
          Serial.println("末端执行器三");
        }
        else if(data.j1Button == 0 && data.j2Button == 1)
        {
          servo_rotation.write(0);
          Serial.println("末端执行器正转");
        }
        else if(data.j2Button == 0 && data.j1Button == 1)
        {
          servo_rotation.write(180);
          Serial.println("末端执行器反转");
        }
        else if(data.j1Button == 1 && data.j2Button == 1)
        {
          servo_rotation.write(90);
          // Serial.println("停止");
        }
        calculateAngles();

        Serial.print("arm=");  
        Serial.print(data_1.arm_value);          
        Serial.print("wrist=");
        Serial.print(data_1.wrist_value);
        Serial.print("base=");
        Serial.println(data_1.base_value);
        Serial.print("x=");  
        Serial.print(x); 
        Serial.print("y=");
        Serial.print(y); 
        Serial.print("z=");
        Serial.println(z);
      }
    }
      if(servo_actuator_value > 270)
      {
        servo_actuator_value = 0;
      }
      if(data_1.arm_value > 125)
      {
        data_1.arm_value = 125;
      }
      else if(data_1.arm_value < 45)
      {
        data_1.arm_value = 45;
      }

      if(data_1.wrist_value > 100)
      {
        data_1.wrist_value = 100;
      }
      else if(data_1.wrist_value < 0)
      {
        data_1.wrist_value = 0;
      }

      if(data_1.base_value > 180)
      {
        data_1.base_value = 100;
      }
      else if(data_1.base_value < 0)
      {
        data_1.base_value = 0;
      }

      // Serial.println(data.button3);
      servo_actuator.write(servo_actuator_value);
      // Serial.println(i);
      servo_base.write(data_1.base_value);
      // Serial.println(servo_base_value);
      servo_arm.write(data_1.arm_value);
      // Serial.println(servo_arm_value);      
      servo_wrist.write(data_1.wrist_value);
      // Serial.println(servo_wrist_value);  
      // Serial.print("arm=");  
      // Serial.print(servo_arm_value);          
      // Serial.print("wrist=");
      // Serial.print(servo_wrist_value);
      // Serial.print("base=");
      // Serial.println(servo_base_value);
      // Serial.print("x=");  
      // Serial.print(x); 
      // Serial.print("y=");
      // Serial.print(y); 
      // Serial.print("z=");
      // Serial.println(z);       
    }
  }
