#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <MPU6050.h>
#include<Queue.h>
#include <SoftwareSerial.h>

MPU6050 mpu;

//float pinReadArray[6]={0,0,0,0,0,0};     -------------   FYP
float pinReadArray[3]={0,0,0};


//****** DATA TRANSFER ********

SoftwareSerial mySerial(10, 11); // RX, TX for UART Communication

/*
float current_data = 0;
float voltage_data = 0;
float temp_data = 0;
*/

//******* FOR CURRENT/VOLTAGE SENSOR INITIALIZATION ****
const int analogIn = A0;
double mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
double RawValue= 0;
double ACSoffset = -200; 
double Voltage = 0;
double Amps = 0;



////******* FOR MOTOR CONTROL ********


int In1 = 7;
int In2 = 12;
int ENA = 5; 

int SPEED = 500;
int pin_in = 2;
int motor_stat = 0;
int current_out = A3;


//******** TASKS *****

void data(void *pvParameters);
void send_data(void *pvParameters);
void motor_control(void *pvParameters);


QueueHandle_t queue_s;

void setup() 
{
  Serial.begin(9600);
  queue_s = xQueueCreate(20, sizeof(float));

  // setting up PWM motor control

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(pin_in, INPUT);
 

  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  
  if (queue_s != NULL)
  {

  xTaskCreate(data, "data", 100, NULL,1, NULL );
  xTaskCreate(send_data, "transmit", 100, NULL,2, NULL );
  xTaskCreate(motor_control, "Motor_control", 100, NULL, 1, NULL);


  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
   while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G));          // setting up MPU 6050
  }
 mySerial.begin(9600);
  
   //vTaskStartScheduler();
  
  
}

void loop()
{}

// ---------------------------------- MPU 6050 SETUP FOR FYP 

/*
void checkSettings()
{
  
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     break; //Serial.println("Stops the clock and keeps the timing generator in reset"); 
   
    case MPU6050_CLOCK_EXTERNAL_19MHZ:  break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ:  break;
    case MPU6050_CLOCK_PLL_ZGYRO:       break; 
    case MPU6050_CLOCK_PLL_YGYRO:       break; 
    case MPU6050_CLOCK_PLL_XGYRO:       break; 
    case MPU6050_CLOCK_INTERNAL_8MHZ:   break;
  }
  
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:       break;      
    case MPU6050_RANGE_8G:        break;     
    case MPU6050_RANGE_4G:        break;     
    case MPU6050_RANGE_2G:        break;     
  }  

}*/

void data (void *pvParameters)
{
  (void) pvParameters;

  // checkSettings();             *------------------- FYP **
  
  for(;;){
  
  float temp = mpu.readTemperature();
 // Vector normAccel = mpu.readNormalizeAccel();
  RawValue = analogRead(analogIn);
  Voltage = (RawValue / 1024.0) * 15; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);
  
  
 /* pinReadArray[0] = normAccel.XAxis;    *--------------- FYP ***
  pinReadArray[1] = normAccel.YAxis;
  pinReadArray[2] = normAccel.ZAxis; */
  pinReadArray[0] = temp;
  pinReadArray[1] = Voltage;
  pinReadArray[2] = Amps;


   xQueueSend(queue_s, &pinReadArray, portMAX_DELAY); // sends data from sensors to sending task which will transmit it over wifi to STM32
   //vTaskDelay(1);
  
}
}


void send_data (void *pvParameters)
{
   (void) pvParameters;
   
   for (;;){
     if(xQueueReceive(queue_s, &pinReadArray, portMAX_DELAY) == pdPASS )
    {
      /*Serial.print("X-AXIS: ");
      Serial.print(pinReadArray[0]);
      Serial.print("    Y-AXIS: ");
      Serial.print(pinReadArray[1]);
      Serial.print("    Z-AXIS: ");
      Serial.print(pinReadArray[2]);*/
      
      Serial.print("    TEMP: ");
      Serial.print(pinReadArray[0]);
      
      Serial.print("    Voltage: ");
      Serial.print(pinReadArray[1]);
      
      Serial.print("    Current: ");
      Serial.println(pinReadArray[2]);


    if (Serial.available()) {
    mySerial.write(pinReadArray[0]);
    Serial.println("TEMP Sent");
    mySerial.write(pinReadArray[1]);
    Serial.println("VOLTAGE Sent");
   // mySerial.write(pinReadArray[2]);
    Serial.println("CURRENT Sent");
    
  }
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    }
  
  }

  void motor_control(void *pvParameters){

    (void) pvParameters;

    for(;;){
      motor_stat = digitalRead(pin_in);
    if (motor_stat == LOW)
    {
      analogWrite(ENA, SPEED);
    }else
    {
      analogWrite(ENA, 0);
    }
    }
  }
