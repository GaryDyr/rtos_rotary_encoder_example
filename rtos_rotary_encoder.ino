
/*
  Test for how to implement a rotary encorder in RTOS.
  It uses the ESPRotary and Button2 library, even though it likely also
  uses interrupts to determine changes. The advantage is that it has
  built in debounce coding.
  An operational change is signified by a button click. Operational changes are:
  start taking data
  stop taking data
  Non operational changes are anything set to a rotor change. The change is made, but
  no operation is committed until the button is pressed. Of course it is possible to
  change this so that a rotor change or position directly institutes a change. This
  would be indicated by pushing to a specific queue.
*/

#include "Button2.h" //  https://github.com/LennartHennigs/Button2
#include "ESPRotary.h"

//Define Rotary Encoder pins
#define ROTARY_ENCODER_CLK_PIN 32 //called A in library
#define ROTARY_ENCODER_DT_PIN 35  //called B in library
#define ROTARY_ENCODER_BUTTON_PIN 34

//depending on your encoder - try 1,2 or 4 to get expected behaviour
#define CLICKS_PER_STEP 4 //this is really a divisor; if 1 showed 4 steps.
#define ONBOARD_LED  2

ESPRotary r;
Button2 b;

int taskCore;
int previousMillis;
int currentMillis;
int count;

//Structure to hold time/data pairs
struct Data_t {
  float stime = 0;
  float wt = 0;
} RX_Data_t;

//Structure to monitor rotary encoder status
struct Encoder_s {
  int8_t rotar_state; // = 0;   //not currently used.will show -255 to +255.
  int8_t rotar_direction;       // = 0; //only used for testing.
  int8_t btn_click; //= 0;      //0 for no btn click; 1 for btn click
  int8_t process_state; // = 0; //with rotor state controls opertions; see onButtonClick
} Encoder_state;

//smple way to set struct to defaults... 
Encoder_s defaults = { 0, 0, 0, 0 };
Encoder_s params   = defaults; //then use parans = defaults as needd to reset individually

UBaseType_t uxHighWaterMark;  // Use for debug and reviewing stack height limits

//------------------------------------------------------------------------------
// define tasks for Sensor Data and SD Write
void EncoderTask( void *pvParamaters );
void DataTask( void *pvParameters );
TaskHandle_t Encoder_Handle;
TaskHandle_t DataTask_Handle;
TaskHandle_t DataEaterTask_Handle;
//------------------------------------------------------------------------------

//Declare Queue data type for FreeRTOS
QueueHandle_t DataQueue; //
QueueHandle_t EncoderQueue;

void rotary_onButtonClick() {
  //Controls scale functioning; driven by ESPRotary click() defined after loop().
  //Encoder_state.process_state conditions:
  //  0 - idle only watch for interrupts; changes Encoder_state.process_state to 1
  //  1 - take data on first click; stop and send/print data on next click
  // -1 - calibrate scale
  //static unsigned long lastTimePressed = 0;
  if (Encoder_state.rotar_direction == 255) { //i.e., CCW
    Encoder_state.process_state = -1;
  }
  Serial.print("rotary state: " );
  Serial.print(Encoder_state.rotar_state);
  Serial.print("; rotar change: ");
  Serial.print(Encoder_state.rotar_direction);
  Serial.print(" Process state ");
  Serial.print(Encoder_state.process_state);
  Serial.print(" rotar_state: ");
  Serial.println(Encoder_state.rotar_state);
  // to get here a button was clicked, signaling a change operation
  // on return to loop will start reading scale
  switch (Encoder_state.process_state) {
    case 0: //data acquition is suspended. start new acquistion
      //for example, get load cell tare
      //scale.tare(10);
      Encoder_state.process_state = 1;

      //reset btn state, so next click will stop data acquistion
      count = 0;
      //wts = "";
      //t1 = "";
      //turn on led
      digitalWrite(ONBOARD_LED, HIGH);
      previousMillis = millis();
      //at start up the sensor read and send tasks are suspended; see setup()
      //reactivate the tasks
      vTaskResume(DataEaterTask_Handle);
      vTaskDelay(5);
      vTaskResume(DataTask_Handle);
      vTaskDelay(5);
      break;
    Serial.println("Data capture initiated");
    case 1:  //was collecting data; stop data acquisition
      //send data out; return to idle
      // DECISION POINT, ASSUMING DATA HAS BEEN TEMPORARILY OR
      //PERMANENTLY STORED IN FILE. CURRENTLY, DATA IS JUST PRINTED
      //BUT A LONGER TERM STORAGE SOLUTION WOULD GO HERE.
      //reset all
      Serial.println("Data stop initiated");
      Encoder_state = params;
      count = 1;
      vTaskSuspend(DataTask_Handle);
      vTaskSuspend(DataEaterTask_Handle);
      //blink to acknowledge call to output;
      digitalWrite(ONBOARD_LED, LOW);
      //indicate via blinking LED data sending initiated
      for (int i = 0; i < 5; i++) {
        digitalWrite(ONBOARD_LED, LOW);
        vTaskDelay(300);
        digitalWrite(ONBOARD_LED, HIGH);
        vTaskDelay(200);
      }
      digitalWrite(ONBOARD_LED, HIGH);
      Serial.println("Data Out Initiated.");
      vTaskDelay(50);
      /*
        //data out to cloud; these modules not here
        //see scale_streaming.ino in other repository Google Sheet output
        if (output_type == PRINT_TO_SERIAL) {
        printData();
        } else {
         Serial.println("Data to WiFi");
         sendData();
        }
      */

      //turn off LED
      digitalWrite(ONBOARD_LED, LOW);
      vTaskDelay(20);
      Encoder_state = params;
      //reset wt string;
      //wts = "";
      //re-zero the encoder position
      r.resetPosition();
      break;
    case -1:
      vTaskSuspend(DataTask_Handle);
      vTaskSuspend(DataEaterTask_Handle);
      Serial.println("Doing something like a calibration");
      vTaskDelay(1000);
      //calibrate mode
      //Calibrate_scale();
      //reset states to zero
      Encoder_state = params;
      //wts = "";
      r.resetPosition();
      break;
      digitalWrite(ONBOARD_LED, LOW);
  }
  //reset the btn trigger
  Encoder_state.btn_click = 0; 
}

//task to handle encoder changes
void EncoderTask( void * pvParameters ) {
  // String taskMessage = "Task running on core ";
  // taskMessage = taskMessage + xPortGetCoreID();
  // Serial.println(taskMessage);
  while (true) {
    //these two ESPEncoder library fcns use the callback fcns found after
    //loop() to make changes to the encoder struct. and drive the
    //other tasks
    r.loop(); //rotor handling [this loop() is defined in ESPRotary
    b.loop(); //button handlin
    //Serial.println(Encoder_state.btn_click);
    vTaskDelay(20);
  }

}
void DataTask(void * pvParameters) {
  //How many pts to take depends where the data is stored, In memory, might be able to get to
  //several thousand, but on disk limit would be up to 4 GB file limit/file
  //technically two arrays of floating pt should take up, 4 bytes/value, with some overhead for a struct.
  //
  struct Data_t TX_Data_t;
  while (true) {
    //e.g., load cell example should be aver. of ~ 3 pts/s
    //TX_Data_t.wt = scale.get_units(3);
    //dummy test wt
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //Serial.println(uxHighWaterMark);
    TX_Data_t.wt = random(2, 12)/1.00;
    //for output to internet need to convert to string; this
    //can lead to memory issues.
    // awt = String(wt_value,2); //convert wt to string
    // wts += awt;
    // wts += ",";
     currentMillis = millis();
    TX_Data_t.stime = (currentMillis - previousMillis) / 1000.;
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    //Serial.println(uxHighWaterMark);
    //for testing and time to string
    //Serial.print(TX_Data_t.stime);
    //Serial.print(" ");
    //Serial.println(TX_Data_t.wt);
    //vTaskDelay(2);
    //TX_Data_t.stime = String((currentMillis - previousMillis)/1000.,2);
    //For checking free space:
    //uint16_t FreeSpace = uxQueueSpacesAvailable( DataQueue ); //Use if need to evaluate SD write and queue availability
    //Serial.println(FreeSpace);
    if(xQueueSend( DataQueue, &TX_Data_t, portMAX_DELAY ) != pdPASS ) {  //portMAX_DELAY 
      Serial.println("xQueueSend is not working"); 
    }
    //sets a limit on maximum number of points to take; a limit is important
    //if dumping data to strings, because of ESP heap memory limits  or 
    //to the 4GB limit for SD file. 100 is just a small test, takes 1.23 s
    if(count > 100){
      vTaskSuspend(DataTask_Handle);
      vTaskSuspend(DataEaterTask_Handle);  
    }
    else{
      count++;
    }
  }
}

void DataEaterTask( void * pvParameters) {
  struct Data_t receiveData;
  char buff[1*1024]; //careful with this. allows only up to 999.99
  for (;;) {
   if (xQueueReceive(DataQueue, &receiveData, portMAX_DELAY) != pdPASS) {
     Serial.println("xQueueRecieve is not working");
    }
    //THIS IS WHERE OUTPUT TO SD WOULD BE CALLED TO AN ALREADY OPEN FILE
    //FILE NEEDS TO BE OPENED (AND CLOSED) IN onButtonnClick
    //ESP allows use of sprintf, but watch buffer size.
    sprintf(buff, "%.3f; %.2f\n", receiveData.stime, receiveData.wt);
    Serial.println(buff);
    vTaskDelay(1);
  }
}

void setup() {
  pinMode(ONBOARD_LED, OUTPUT);
  Serial.begin(115200);
  //Reset all Encoder struct elements to 0
  Encoder_state = params;
  
  // setup ESPRotary Encoder
  r.begin(ROTARY_ENCODER_DT_PIN, ROTARY_ENCODER_CLK_PIN, CLICKS_PER_STEP);
  r.setChangedHandler(rotate);
  r.setLeftRotationHandler(showDirection);
  r.setRightRotationHandler(showDirection);
  //initiate rotary button part from separate library
  b.begin(ROTARY_ENCODER_BUTTON_PIN);
  b.setTapHandler(click);
  b.setLongClickHandler(resetPosition);

  //Queue Setup
  //Set queue to handle sensor input and output struct
  DataQueue = xQueueCreate(2, sizeof(Data_t));
  if (DataQueue == NULL) {
    Serial.println("Error Creating the Data Queue");
  }
  //Set queue to handle rotary encoder struct
  EncoderQueue = xQueueCreate(1, sizeof(Encoder_s));
  if (EncoderQueue == NULL) {
    Serial.println("Error Creating the Encoder Queue");
  }
  //UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  //Serial.println(uxHighWaterMark);
  //Set up tasks
  Serial.println("Setting up tasks.");
  //Upon setup this is the only task running; suspend all others.
  taskCore = 0; //use primary core ro monitor rotary encoder
  xTaskCreatePinnedToCore(
    EncoderTask,   /* Function to implement the task */
    "EncoderTask",   /* Name of the task */
    15000,           /* Stack size in words; not necessarily optimized*/
    NULL,            /* Task input parameter */
    2,               /* Priority of the task */
    &Encoder_Handle, /* Handle for start, stopping, resuming task*/
    taskCore);       /* Core where the task should run */
  Serial.println("Encoder task started.");
  //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  //Serial.println(uxHighWaterMark);  
  taskCore = 1;
  xTaskCreatePinnedToCore(
    DataTask,         /* Function to implement the task */
    "DataTaak",       /* Name of the task */
    15000,            /* Stack size in words; not necessarily optimized.*/
    NULL,             /* Task input parameter */
    1,                /* Priority of the task */
    &DataTask_Handle, /* Task handle. */
    taskCore);        /* Core where the task should run */

  vTaskSuspend(DataTask_Handle);
  Serial.println("Data task engaged and suspended");
  //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  //Serial.println(uxHighWaterMark); 
  taskCore = 1;
  xTaskCreatePinnedToCore(
    DataEaterTask,         /* Function to implement the task */
    "DataEaterTask",       /* Name of the task */
    15000,                 /* Stack size in words; not necessarily optimized */
    NULL,                  /* Task input parameter */
    3,                     /* Priority of the task */
    &DataEaterTask_Handle, /* Task handle. */
    taskCore);             /* Core where the task should run */

  //uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  //Serial.println(uxHighWaterMark); 
 vTaskSuspend(DataEaterTask_Handle);
  Serial.println("Data Eater task engaged and suspended");
}

void loop() {
vTaskDelete(NULL); 
}

// on change of encoder, these are the 4 callbacks defined in ESPRotary lib.
void rotate(ESPRotary& r) {
  Serial.print("rotar position now: ");
  Serial.println(r.getPosition());
  Encoder_state.rotar_state = r.getPosition();
}

// on left or right rotation
void showDirection(ESPRotary& r) {
  Serial.println(r.directionToString(r.getDirection())); //converts to "LEFT" or "RIGHT"
  Encoder_state.rotar_direction = r.getDirection(); //  1 if CW; 255 if CCW
}

// single click
void click(Button2& btn) {
  Serial.println("Registered a Btn Click!");
  Encoder_state.btn_click = 1;
  rotary_onButtonClick();
}

//long click
void resetPosition(Button2& btn) {
  Serial.println("Long Click!");
  r.resetPosition();
  Encoder_state.rotar_state = 0;
  Encoder_state.rotar_direction = 0;
  Serial.println("Reset!");
}
