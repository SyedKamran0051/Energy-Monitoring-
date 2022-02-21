#include <STM32FreeRTOS.h>
#include <SD.h>
#include <SoftwareSerial.h>

// Software Serial library used for UART communication between stm32 and arduino uno
SoftwareSerial mySerial(PA9, PA10); // RX, TX

// Binary Semaphore used for handling of shared resources of Data
SemaphoreHandle_t xSerialSemaphore;

// Float variables declared to store incoming sensor data
float Temprature, Voltage;
float Current = 1.12;
float Power;


// Flag created to control PWM Controller on arduino and stop motor when fault diagnosed
int PWM_Flag = 0;


// define 4 tasks for AnalogRead, Computation, Display and PWM Control
void TaskAnalogRead( void *pvParameters );
void TaskCompute( void *pvParameters );
void TaskDisplay( void *pvParameters );
void TaskPWM( void *pvParameters );




// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // PIN 7 configured as output to Control PWM contoller on arduino
  pinMode(PA7, OUTPUT);


  
  // Binary Semaphore/Mutex initialized to have secure data sharing between tasks
  if ( xSerialSemaphore == NULL ){
     xSerialSemaphore = xSemaphoreCreateMutex();  
     if ( ( xSerialSemaphore ) != NULL )
       xSemaphoreGive( ( xSerialSemaphore ) );  
    }

    
  while (!Serial) {
    ; // wait for serial port to connect.
  }

    
    // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);



  // Create 4 tasks to run using FREERTOS
  xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *)"AnalogRead"   // Char name used for debugging purposes
    ,  128  // Stack Size
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
    
  xTaskCreate(
    TaskCompute
    ,  (const portCHAR *)"Compute"   // Char name used for debugging purposes
    ,  128  // Stack Size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  xTaskCreate(
    TaskDisplay
    ,  (const portCHAR *)"Display"   // Char name used for debugging purposes
    ,  128  // Stack Size
    ,  NULL
    ,  3  // Priority
    ,  NULL );
    
  xTaskCreate(
    TaskPWM
    ,  (const portCHAR *) "PWMControl" // Char name used for debugging purposes
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}



void loop()
{
  // Empty. Things are done in Tasks.
}


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  float TempratureData;
  float CurrentData;
  float VoltageData;
  
  for (;;)
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      if (mySerial.available()) {
        TempratureData = (mySerial.read());      
      }
      
      if (mySerial.available()) {
        VoltageData = (mySerial.read());
      }
      
      
      Temprature = TempratureData;
      Voltage = VoltageData;
      
    xSemaphoreGive( xSerialSemaphore ); 
    }
  
  vTaskDelay(pdMS_TO_TICKS(50));  
  }
}



void TaskCompute(void *pvParameters)  
{
  (void) pvParameters;
  
  for (;;) 
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
    
      Power = Current * Voltage;      
      
      if (Temprature > 25 || Voltage >= 8 || Current > 1.5){
        PWM_Flag = 1;
      }
      else {
        PWM_Flag = 0;
      }
      
      xSemaphoreGive( xSerialSemaphore ); 
    }
  vTaskDelay(pdMS_TO_TICKS(200));
  }
}



void TaskDisplay(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  for (;;) 
  {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      if (PWM_Flag != 1){   
        Serial.println("---------------");
        Serial.print("Temprature:");
        Serial.println(Temprature);
        
        Serial.print("Voltage:");
        Serial.println(Voltage);
        
        Serial.print("Current:");
        Serial.println(Current);
        
        Serial.print("Power:");
        Serial.println(Power);
        Serial.println("---------------");
        }
      else {
        Serial.println("Error Diagnosed!");
      }
      
      
      xSemaphoreGive( xSerialSemaphore ); 
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}



void TaskPWM(void *pvParameters) 
{
  (void) pvParameters;
  
  for (;;) 
  {
    if (PWM_Flag == 1){
      digitalWrite(PA7, HIGH);
    }
    else{
      digitalWrite(PA7, LOW);
    }
  }
}
