#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define M_PI 3.14159265

//
// System settings
//

//set HUMAN_READABLE_SERIAL to 1 for csv serial output
#define HUMAN_READABLE_SERIAL 0

#define ENABLE_MOTOR1 1
#define ENABLE_MOTOR2 0

#define ENABLE_DISPLAY 1

#define MOTOR_UPDATE_FREQ_HZ 1000
#define SERIAL_UPDATE_FACTOR 10
#define SCREEN_UPDATE_FACTOR 5

#define TIMING_OVERHEAD_us 2
//
// Definitions for STM32 pin equivalents
//

#define STM_B7  2
#define STM_B6  3
#define STM_B5  4
#define STM_B4  5
#define STM_B3  6
#define STM_A15 7
#define STM_A12 8
#define STM_A11 9
#define STM_A10 10
#define STM_A9  11
#define STM_A8  12
#define STM_B15 13
#define STM_B14 14
#define STM_B13 15

#define STM_C13 17
#define STM_C14 18
#define STM_C15 19
#define STM_A0  20
#define STM_A1  21
#define STM_A2  22
#define STM_A3  23
#define STM_A4  24
#define STM_A5  25
#define STM_A6  26
#define STM_A7  27
#define STM_B0  28
#define STM_B1  29
#define STM_B10 30
#define STM_B11 31

//
// Definitions for Inverted Pendulum control board pins
//

#define PIN_OLED_SCL 4
#define PIN_OLED_SDA 5
#define PIN_OLED_RES 6
#define PIN_OLED_DC 7
#define OLED_WIDTH_PX 128
#define OLED_HEIGHT_PX 64

#define PIN_BUTTON_PLUS 9
#define PIN_BUTTON_MINUS 8
#define PIN_BUTTON_X 22
#define PIN_BUTTON_M 27
#define PIN_BUTTON_USER 25

#define PIN_LED_USER 24

#define PIN_MOTOR1_ENCA 2
#define PIN_MOTOR1_ENCB 3
#define PIN_MOTOR1_IN1 16
#define PIN_MOTOR1_IN2 15
#define PIN_MOTOR1_PWM 29

#define PIN_MOTOR2_ENCA 21
#define PIN_MOTOR2_ENCB 20
#define PIN_MOTOR2_IN1 13
#define PIN_MOTOR2_IN2 12
#define PIN_MOTOR2_PWM 28

#define PIN_ANGLE_POT 23

//
// Initialize Globals
//
bool system_enabled = true;
const int loop_delay_us = 1000000/MOTOR_UPDATE_FREQ_HZ;
uint32_t loop_start_us = 0;
uint32_t loop_time = 0;

float time_s = 0;

int32_t angle_pot = 0;

#define MODE_BUTTON_CONTROL 0
#define MODE_CALIBRATE 1
#define MODE_UART_CONTROL 2
#define MODE_COSINE_CONTROL 3
#define MODE_PID_ANGLE_SPEED_CONTROL 4
#define MODE_PID_ANGLE_POS_CONTROL 5

#if ENABLE_MOTOR1 == 1
byte motor1_control_mode = 3;

volatile int32_t motor1_count = 0;
int32_t motor1_last_count = 0;
int32_t motor1_count_delta = 0;
int32_t motor1_cps = 0;
#define MOTOR1_HIST_LEN 30
int32_t motor1_cps_history[MOTOR1_HIST_LEN];

int32_t motor1_error = 0;
int32_t motor1_error_total = 0;
int32_t motor1_setpoint = 0;
int32_t motor1_command = 0;

float motor1_kP = 10.00;
float motor1_kI = 0.00;
float motor1_kD = 0.00;

#define MAX_COSINES 8
float motor1_cos_mag[MAX_COSINES] =     {0.5, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
float motor1_cos_freq_Hz[MAX_COSINES] = {0.5, 0.25, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
float motor1_cos_phase_s[MAX_COSINES] = {2.5, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0};
#endif

#if ENABLE_MOTOR2 == 1
volatile int32_t motor2_count = 0;
#endif

#if ENABLE_DISPLAY == 1
Adafruit_SSD1306 display(OLED_WIDTH_PX, OLED_HEIGHT_PX, PIN_OLED_SDA, PIN_OLED_SCL, PIN_OLED_DC, PIN_OLED_RES, 10);
#endif

//
// Init Subroutines
//

void init_pinmodes()
{
  pinMode(PIN_BUTTON_PLUS, INPUT);
  pinMode(PIN_BUTTON_MINUS, INPUT);
  pinMode(PIN_BUTTON_X, INPUT);
  pinMode(PIN_BUTTON_M, INPUT);
  pinMode(PIN_BUTTON_USER, INPUT);
  pinMode(PIN_LED_USER, OUTPUT);

#if ENABLE_MOTOR1 == 1
  pinMode(PIN_MOTOR1_ENCA, INPUT);
  pinMode(PIN_MOTOR1_ENCB, INPUT);
  pinMode(PIN_MOTOR1_IN1, OUTPUT);
  pinMode(PIN_MOTOR1_IN2, OUTPUT);
  pinMode(PIN_MOTOR1_PWM, OUTPUT);
#endif

#if ENABLE_MOTOR2 == 1
  pinMode(PIN_MOTOR2_ENCA, INPUT);
  pinMode(PIN_MOTOR2_ENCB, INPUT);
  pinMode(PIN_MOTOR2_IN1, OUTPUT);
  pinMode(PIN_MOTOR2_IN2, OUTPUT);
  pinMode(PIN_MOTOR2_PWM, OUTPUT);
#endif
}

void init_display()
{
#if ENABLE_DISPLAY == 1
  display.begin(SSD1306_SWITCHCAPVCC);
  display.display();
  delay(100);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.clearDisplay();
#endif
}

void init_serial()
{
  Serial.begin(115200);
  
#if HUMAN_READABLE_SERIAL == 1
  Serial.println("\n");
  Serial.println("timestamp_ms, angle_pot, motor1_counts, count_delta_per_s");
#endif
}

void init_enc_isr()
{
#if ENABLE_MOTOR1 == 1
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR1_ENCA), motor1_encA_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR1_ENCB), motor1_encB_change, CHANGE);
#endif

#if ENABLE_MOTOR2 == 1
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR2_ENCA), motor2_encA_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_MOTOR2_ENCB), motor2_encB_change, CHANGE);
#endif
}

void setup() {
  init_pinmodes();
  init_enc_isr();
  init_display();
  init_serial();

  loop_start_us = micros();
}

//
// Main Update Loop
//

unsigned long loop_count = 0;

void loop() {
  loop_time = (micros()-loop_start_us);
  loop_start_us = micros();

  time_s = loop_start_us/1000000.0;
  
  motor1_count_delta = motor1_count - motor1_last_count;
  motor1_last_count = motor1_count;

  motor1_cps_history[loop_count % MOTOR1_HIST_LEN] = (motor1_count_delta*MOTOR_UPDATE_FREQ_HZ);
  motor1_cps = 0;
  for(int i = 0; i < MOTOR1_HIST_LEN; i++)
      motor1_cps += motor1_cps_history[i];
  motor1_cps /= MOTOR1_HIST_LEN;
  
  angle_pot = analogRead(PIN_ANGLE_POT)-512;

  if( digitalRead(PIN_BUTTON_X)==0 ){
    system_enabled = false;    
  } else if( digitalRead(PIN_BUTTON_M)==0 ){
    system_enabled = true;
  }

  if (system_enabled)
    update_motor_control();
  else
    motor1_set_stop();

  if(loop_count % SERIAL_UPDATE_FACTOR == 0)
    serial_write();
  else if(loop_count % SERIAL_UPDATE_FACTOR == 1) //offset by 1 so updates occur on seperate loops
    serial_read();
    
#if ENABLE_DISPLAY == 1
  if(loop_count % SCREEN_UPDATE_FACTOR == 2) 
    update_display();
#endif

  loop_count++;
  delayMicroseconds(loop_delay_us - (micros()-loop_start_us) - TIMING_OVERHEAD_us);
}

//
// Update Subroutines
// 

void update_motor_control()
{
  switch(motor1_control_mode)
  {
    case MODE_BUTTON_CONTROL:
      if( digitalRead(PIN_BUTTON_PLUS)==0 ){
        motor1_set_ccw();
      } else if( digitalRead(PIN_BUTTON_MINUS)==0 ){
        motor1_set_cw();
      } else {
        motor1_set_stop();
      }
      
      analogWrite(PIN_MOTOR1_PWM, angle_pot/4);
      return;
    
    case MODE_UART_CONTROL:
      return;
    
    case MODE_COSINE_CONTROL:
      float command;
      //float time_s;
      
      command = 0;
      //time_s = (float)(micros()/1000000);
      for( int i = 0; i < MAX_COSINES; i++)
      {
        command += motor1_cos_mag[i] * cos(2*M_PI*motor1_cos_freq_Hz[i]*(time_s+motor1_cos_phase_s[i]));
      }
      motor1_command = (int)(command*10000);
      motor1_set(motor1_command);
      return;
           
    case MODE_PID_ANGLE_SPEED_CONTROL:
      motor1_setpoint = angle_pot*10;
      motor1_error = motor1_cps - motor1_setpoint;
      break;
      
    case MODE_PID_ANGLE_POS_CONTROL:
      motor1_setpoint = angle_pot;
      motor1_error = motor1_count - motor1_setpoint;
      break;
    }
    
    motor1_error_total += motor1_error;

    motor1_command = (motor1_kP * motor1_error);
                     //+ (motor1_kI * (float)motor1_error_total)
                     //+ (motor1_kD * (float)motor1_cps));
                     
    motor1_set(motor1_command);
}

#if ENABLE_DISPLAY == 1
void update_display()
{
  display.clearDisplay();
  display.setCursor(0,0);
  
  display.print(angle_pot);
  display.print(" ");
  display.println(motor1_count);
  display.println("+ - X M U A B");
  display.print(digitalRead(PIN_BUTTON_PLUS));
  display.print(" ");
  display.print(digitalRead(PIN_BUTTON_MINUS));
  display.print(" ");
  display.print(digitalRead(PIN_BUTTON_X));
  display.print(" ");
  display.print(digitalRead(PIN_BUTTON_M));
  display.print(" ");
  display.print(digitalRead(PIN_BUTTON_USER));
  display.print(" ");
  display.print(digitalRead(PIN_MOTOR1_ENCA));
  display.print(" ");
  display.print(digitalRead(PIN_MOTOR1_ENCB));
  display.print("\n");

  display.println(motor1_command);
  display.println(motor1_cps);
  display.println(loop_time);
  display.println(time_s);

  display.display();
}
#endif

typedef union
{
 float value;
 uint8_t bytes[4];
} FLOATUNION_t;

typedef union
{
 int32_t value;
 uint8_t bytes[4];
} INT32UNION_t;

INT32UNION_t writeint32;
void serial_write()
{
#if HUMAN_READABLE_SERIAL == 1

  Serial.print(millis());
  Serial.print(", ");
  Serial.print(angle_pot);
  Serial.print(", ");
  Serial.print(motor1_count);
  Serial.print(", ");
  Serial.println(motor1_cps);
  
#else
  int32_t printtime = millis();

  byte serialbuffer[20];

  serialbuffer[0] = (byte) 'A';
  serialbuffer[1] = (byte) 'B';
  serialbuffer[2] = (byte) 'C';
  
  writeint32.value = printtime;
  serialbuffer[3] = writeint32.bytes[0];
  serialbuffer[4] = writeint32.bytes[1];
  serialbuffer[5] = writeint32.bytes[2];
  serialbuffer[6] = writeint32.bytes[3];
  
  writeint32.value = angle_pot;
  serialbuffer[7] = writeint32.bytes[0];
  serialbuffer[8] = writeint32.bytes[1];
  serialbuffer[9] = writeint32.bytes[2];
  serialbuffer[10] = writeint32.bytes[3];
  
  writeint32.value = motor1_count;
  serialbuffer[11] = writeint32.bytes[0];
  serialbuffer[12] = writeint32.bytes[1];
  serialbuffer[13] = writeint32.bytes[2];
  serialbuffer[14] = writeint32.bytes[3];
  
  writeint32.value = motor1_cps;
  serialbuffer[15] = writeint32.bytes[0];
  serialbuffer[16] = writeint32.bytes[1];
  serialbuffer[17] = writeint32.bytes[2];
  serialbuffer[18] = writeint32.bytes[3];

  serialbuffer[19] = (byte) '\n';
  Serial.write(serialbuffer, 20);
  
#endif
}

void serial_read()
{
  // DEF Message start
  // 1    Manual Control mode
  // \n   Message end
  
  // DEF Message start
  // 1    UART Control mode
  // 4    (signed int) motor command
  // \n   Message end
  
  // DEF Message start
  // 1    Cosine Control mode
  // 96   (float) Cosine array bytes
  // \n   Message end

  // DEF Message start
  // 1    PID Control mode
  // 3*4  (float) Gains 
  // \n   Message end
  
  byte readbuffer[12];
            
  while(Serial.available() > 4)
  {
    if(Serial.read()=='D')
    {
      if(Serial.read()=='E' && Serial.read()=='F')
      {
        Serial.print("Recieved command ");
        switch(Serial.read()){
          case MODE_BUTTON_CONTROL:
            motor1_control_mode = MODE_BUTTON_CONTROL;
            break;
              
          case MODE_CALIBRATE:
            Serial.println("CALIBRATE");
            delay(500);
            motor1_control_mode = MODE_CALIBRATE;
            Serial.println("READY");
            break;
            
          case MODE_UART_CONTROL:
            for(int i = 0; i < 4; i++)
              readbuffer[i] = Serial.read();
            motor1_command = ((signed int*)readbuffer)[0];
            break;
            
          case MODE_COSINE_CONTROL:
            Serial.print("COSINE control ");
            motor1_control_mode = MODE_COSINE_CONTROL;
            
            for(int cosine = 0; cosine < MAX_COSINES; cosine++)
            {
              for(int i = 0; i < 12; i++)
                readbuffer[i] = Serial.read();
              motor1_cos_mag[cosine] = ((float*)readbuffer)[0];
              motor1_cos_freq_Hz[cosine] = ((float*)readbuffer)[1];
              motor1_cos_phase_s[cosine] = ((float*)readbuffer)[2];
              
              Serial.print(motor1_cos_mag[cosine]);
              Serial.print(" ");
              Serial.print(motor1_cos_freq_Hz[cosine]);
              Serial.print(" ");
              Serial.println(motor1_cos_phase_s[cosine]);
              
              if(Serial.peek()=='\n') //Full the rest with 0s if the message is ended
                for(int i = cosine+1; i < MAX_COSINES; i++)
                {
                  motor1_cos_mag[i] = 0.0;
                  motor1_cos_freq_Hz[i] = 0.0;
                  motor1_cos_phase_s[i] = 0.0;
                }
                break;
            }
            
            for(int cosine = 0; cosine < MAX_COSINES; cosine++)
            {
              Serial.print(motor1_cos_mag[cosine]);
              Serial.print(" ");
              Serial.print(motor1_cos_freq_Hz[cosine]);
              Serial.print(" ");
              Serial.println(motor1_cos_phase_s[cosine]);
            }
            break;
            
          case MODE_PID_ANGLE_SPEED_CONTROL:
            motor1_control_mode = MODE_PID_ANGLE_SPEED_CONTROL;
            for(int i = 0; i < 12; i++)
              readbuffer[i] = Serial.read();
            motor1_kP = ((float *)readbuffer)[0];
            motor1_kI = ((float *)readbuffer)[1];
            motor1_kD = ((float *)readbuffer)[2];
            break;
            
          case MODE_PID_ANGLE_POS_CONTROL:
            motor1_control_mode = MODE_PID_ANGLE_POS_CONTROL;
            for(int i = 0; i < 12; i++)
              readbuffer[i] = Serial.read();
            motor1_kP = ((float *)readbuffer)[0];
            motor1_kI = ((float *)readbuffer)[1];
            motor1_kD = ((float *)readbuffer)[2];
            break;
        }

        while(Serial.available() && Serial.read()!='\n');
      }
      else
        break;
    }
    else
      break;
  }
}

//
// Motor Subroutines
//
#if ENABLE_MOTOR1 == 1
void motor1_set(signed int command)
{
  if(command == 0){
    motor1_set_stop();
  } else if (command >0) {
    motor1_set_ccw();
  } else {
    motor1_set_cw();            
  }
  analogWrite(PIN_MOTOR1_PWM, desired_cps_to_motor1(abs(command)));
}

void motor1_set_stop()
{
  digitalWrite(PIN_MOTOR1_IN1, LOW);
  digitalWrite(PIN_MOTOR1_IN2, LOW);
}

void motor1_set_cw()
{
  digitalWrite(PIN_MOTOR1_IN1, HIGH);
  digitalWrite(PIN_MOTOR1_IN2, LOW);
}

void motor1_set_ccw()
{
  digitalWrite(PIN_MOTOR1_IN1, LOW);
  digitalWrite(PIN_MOTOR1_IN2, HIGH);
}

//motor max speed 10000 cps
int desired_cps_to_motor1(int desired_cps)
{
  return int(0.0256 * desired_cps) + 5;
}
#endif

#if ENABLE_MOTOR2 == 1
void motor2_set_stop()
{
  digitalWrite(PIN_MOTOR2_IN1, LOW);
  digitalWrite(PIN_MOTOR2_IN2, LOW);
}

void motor2_set_cw()
{
  digitalWrite(PIN_MOTOR2_IN1, HIGH);
  digitalWrite(PIN_MOTOR2_IN2, LOW);
}

void motor2_set_ccw()
{
  digitalWrite(PIN_MOTOR2_IN1, LOW);
  digitalWrite(PIN_MOTOR2_IN2, HIGH);
}
#endif

//
// Interrupt Service Routines
// 

#if ENABLE_MOTOR1 == 1
//Positive direction is defined as the A phase lead direction
void motor1_encA_change()
{
  if (digitalRead(PIN_MOTOR1_ENCA)){
    if(digitalRead(PIN_MOTOR1_ENCB))
      motor1_count--;
    else
      motor1_count++;
  } else {
    if(digitalRead(PIN_MOTOR1_ENCB))
      motor1_count++;
    else
      motor1_count--;
  }
}

void motor1_encB_change()
{
  if (digitalRead(PIN_MOTOR1_ENCB)){
    if(digitalRead(PIN_MOTOR1_ENCA))
      motor1_count++;
    else
      motor1_count--;
  } else {
    if(digitalRead(PIN_MOTOR1_ENCA))
      motor1_count--;
    else
      motor1_count++;
  }
}
#endif

#if ENABLE_MOTOR2 == 1
void motor2_encA_change()
{
  if (digitalRead(PIN_MOTOR2_ENCA)){
    if(digitalRead(PIN_MOTOR2_ENCB))
      motor2_count--;
    else
      motor2_count++;
  } else {
    if(digitalRead(PIN_MOTOR2_ENCB))
      motor2_count++;
    else
      motor2_count--;
  }
}

void motor2_encB_change()
{
  if (digitalRead(PIN_MOTOR2_ENCB)){
    if(digitalRead(PIN_MOTOR2_ENCA))
      motor2_count++;
    else
      motor2_count--;
  } else {
    if(digitalRead(PIN_MOTOR2_ENCA))
      motor2_count--;
    else
      motor2_count++;
  }
}
#endif
