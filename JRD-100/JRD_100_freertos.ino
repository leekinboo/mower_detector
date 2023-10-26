#include <Arduino_FreeRTOS.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include "UNIT_UHF_RFID.h"


Unit_UHF_RFID RFID(&Serial3);
bool DEBUG = true;

int ledAPin = 26;   // choose the pin for the LED
int inputAPin = 5;  // choose the input pin (for PIR sensor)


int ledBPin = 22;   // choose the pin for the LED
int inputBPin = 4;  // choose the input pin (for PIR sensor)


int pirAState = LOW;  // we start, assuming no motion detected
int pirBState = LOW;  // we start, assuming no motion detected

int valA = 0;  // variable for reading the pin status
int valB = 0;  // variable for reading the pin status

TaskHandle_t Handle_aTask;
TaskHandle_t Handle_bTask;
TaskHandle_t Handle_cTask;

void sendCMD(uint8_t* data, size_t size) {
  // Serial.write(data, size);
}

void setup() {
  Serial.begin(9600);
  Serial3.begin(115200);

  Serial.println("UHF Reader");

  //Serial.println(F("In Setup function"));
  pinMode(ledAPin, OUTPUT);   // declare LED as output
  pinMode(inputAPin, INPUT);  // declare sensor as input


  pinMode(ledBPin, OUTPUT);   // declare LED as output
  pinMode(inputBPin, INPUT);  // declare sensor as input

  //delay(100);  // prevents usb driver crash on startup, do not omit this
  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
  xTaskCreate(Thread_PIR_ReadA, "Task PIRA", 256, NULL, 2, NULL);

  xTaskCreate(Thread_PIR_ReadB, "Task PIRB", 256, NULL, 2, NULL);
  xTaskCreate(Thread_PIR_Read_Rfid, "Task RFID", 512, NULL, 1, NULL);
  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();
  Serial.println(F("In Setup function"));
}


void Thread_PIR_ReadA(void* pvParameters) {
  (void)pvParameters;
  // Serial1.println(F("TaskA"));

  while (1) {
    //Serial.println(F("TaskA"));
    valA = digitalRead(inputAPin);  // read input value
    if (valA == HIGH) {             // check if the input is HIGH
      digitalWrite(ledAPin, HIGH);  // turn LED ON
      if (pirAState == LOW) {
        // we have just turned on
        // We only want to print on the output change, not state
        pirAState = HIGH;
 
       }
 
    } else {
      digitalWrite(ledAPin, LOW);  // turn LED OFF
      if (pirAState == HIGH) {
        // we have just turned of
        // We only want to print on the output change, not state
        pirAState = LOW;
      }
     }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}

void Thread_PIR_ReadB(void* pvParameters) {
  (void)pvParameters;


  while (1) {
    //  Serial.println(F("Task1"));
    valB = digitalRead(inputBPin);  // read input value
    if (valB == HIGH) {             // check if the input is HIGH
      digitalWrite(ledBPin, HIGH);  // turn LED ON
      if (pirBState == LOW) {
        // we have just turned on
        // We only want to print on the output change, not state
        pirBState = HIGH;
       }
 
    } else {
      digitalWrite(ledBPin, LOW);  // turn LED OFF
      if (pirBState == HIGH) {
        // we have just turned of
        // We only want to print on the output change, not state
        pirBState = LOW;
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}
void Thread_PIR_Read_Rfid(void* pvParameters) {
  (void)pvParameters;
  while (1) {

    if (RFID.pollingOnce_have_card()) {
      //led on
     // Serial.println("card found");
      digitalWrite(LED_BUILTIN, HIGH);
      delay (2000);
      digitalWrite(LED_BUILTIN, LOW);
    }
    else{
    digitalWrite(LED_BUILTIN, LOW);

    }
   // vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

static void Thread_Control(void* pvParameters) {
  //if()
}
void loop() {
}
