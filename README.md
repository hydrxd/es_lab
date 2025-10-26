🔹 Q1. Arduino + FreeRTOS: Simultaneous Digital and Analog Reading with Semaphore
Question:
Write an Arduino program using FreeRTOS to perform simultaneous digital and analog reading
tasks with mutual exclusion using a semaphore.
Requirements:
Task 1: Reads the state of a digital push button and prints it on the serial monitor.
Task 2: Reads analog input values from a sensor and prints them on the serial monitor.

Use a binary semaphore (mutex) to prevent concurrent access to the serial port.
Execute both tasks concurrently using FreeRTOS scheduling.
Implement appropriate delays in each task.

✅ Answer:
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#define BUTTON_PIN 2
#define ANALOG_PIN A0

SemaphoreHandle_t xSerialSemaphore;

void TaskDigitalRead(void *pvParameters);
void TaskAnalogRead(void *pvParameters);

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  xSerialSemaphore = xSemaphoreCreateMutex();

  xTaskCreate(TaskDigitalRead, "Digital", 128, NULL, 1, NULL);
  xTaskCreate(TaskAnalogRead, "Analog", 128, NULL, 1, NULL);
}

void loop() {}

void TaskDigitalRead(void *pvParameters) {
  for (;;) {
    int buttonState = digitalRead(BUTTON_PIN);
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) {
      Serial.print("Button State: ");
      Serial.println(buttonState);
      xSemaphoreGive(xSerialSemaphore);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void TaskAnalogRead(void *pvParameters) {
  for (;;) {
    int sensorValue = analogRead(ANALOG_PIN);
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) {
      Serial.print("Analog Value: ");
      Serial.println(sensorValue);
      xSemaphoreGive(xSerialSemaphore);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

Algorithm:
Initialize Serial communication and configure input/output pins.
Create a binary semaphore for mutual exclusion on Serial printing.
Create two tasks:
Task 1: Read digital push button state.
Task 2: Read analog sensor value.
In each task, take the semaphore before printing to Serial.
Print the respective value.
Release the semaphore.
Delay task for a specific interval and repeat indefinitely.

Explanation:
 Two FreeRTOS tasks run concurrently — each prints data safely to the Serial Monitor using a mutex semaphore.

🔹 Q2. Inter-Task Communication Using Global Variable (Touch Sensor)
Question:
Demonstrate the Inter task communication using the concept of global variable.
Task 1: Read the value of Touch sensor.
Task 2: Get the value of the Touch sensor. If it is High LED must glow.

✅ Answer:
#include <Arduino_FreeRTOS.h>

#define TOUCH_PIN 4
#define LED_PIN 13

volatile int touchState = 0;

void TaskReadTouch(void *pvParameters);
void TaskLEDControl(void *pvParameters);

void setup() {
  pinMode(TOUCH_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  xTaskCreate(TaskReadTouch, "ReadTouch", 128, NULL, 1, NULL);
  xTaskCreate(TaskLEDControl, "LEDControl", 128, NULL, 1, NULL);
}

void loop() {}

void TaskReadTouch(void *pvParameters) {
  for (;;) {
    touchState = digitalRead(TOUCH_PIN);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void TaskLEDControl(void *pvParameters) {
  for (;;) {
    if (touchState == HIGH)
      digitalWrite(LED_PIN, HIGH);
    else
      digitalWrite(LED_PIN, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

Algorithm:
Initialize input (touch sensor) and output (LED) pins.
Define a global variable for touch state.
Create two tasks:
Task 1: Read touch sensor and update global variable.
Task 2: Read global variable and control LED accordingly.
Continuously update and read the global variable in each task.
Delay each task to allow periodic execution.
Explanation:
 The tasks share a global variable touchState for inter-task communication.

🔹 Q3. FreeRTOS Queue: Producer–Consumer
Question:
Write an Arduino program using FreeRTOS to demonstrate inter-task communication using a
Queue.
Create a producer task that generates a sequence of integer values and sends them to a queue.
Create a consumer task that receives the integers from the queue and displays them on the serial monitor.
Use appropriate FreeRTOS functions such as xQueueCreate(), xQueueSend(), and xQueueReceive().
Display suitable messages for both sending and receiving operations.

✅ Answer:
#include <Arduino_FreeRTOS.h>
#include <queue.h>

QueueHandle_t xQueue;

void ProducerTask(void *pvParameters);
void ConsumerTask(void *pvParameters);

void setup() {
  Serial.begin(9600);
  xQueue = xQueueCreate(5, sizeof(int));

  xTaskCreate(ProducerTask, "Producer", 128, NULL, 1, NULL);
  xTaskCreate(ConsumerTask, "Consumer", 128, NULL, 1, NULL);
}

void loop() {}

void ProducerTask(void *pvParameters) {
  int count = 0;
  for (;;) {
    if (xQueueSend(xQueue, &count, portMAX_DELAY)) {
      Serial.print("Sent: ");
      Serial.println(count);
      count++;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ConsumerTask(void *pvParameters) {
  int received;
  for (;;) {
    if (xQueueReceive(xQueue, &received, portMAX_DELAY)) {
      Serial.print("Received: ");
      Serial.println(received);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

Algorithm:
Initialize Serial communication.
Create a queue to hold integer values.
Create two tasks:
Producer: Generate an integer and send it to the queue.
Consumer: Receive integer from the queue and print it.
In each task, use queue send/receive functions with proper blocking.
Delay each task for periodic execution.
Explanation:
 Producer sends integers through a FreeRTOS queue; consumer receives and prints them.

🔹 Q4. Two Independent Tasks – 2-bit and 3-bit Counters
Write an arduino sketch to perform the following:
Create 2 independent tasks.
Task 1 – implement 2 bit counter. (Use 2 LED to display the result)
Task 2 – implement 3 bit counter. Display the result on the serial monitor.
✅ Answer:
#include <Arduino_FreeRTOS.h>

#define LED1 8
#define LED2 9

void Task2BitCounter(void *pvParameters);
void Task3BitCounter(void *pvParameters);

void setup() {
  Serial.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  xTaskCreate(Task2BitCounter, "2Bit", 128, NULL, 1, NULL);
  xTaskCreate(Task3BitCounter, "3Bit", 128, NULL, 1, NULL);
}

void loop() {}

void Task2BitCounter(void *pvParameters) {
  for (int i = 0; ; i = (i + 1) % 4) {
    digitalWrite(LED1, bitRead(i, 0));
    digitalWrite(LED2, bitRead(i, 1));
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void Task3BitCounter(void *pvParameters) {
  for (int i = 0; ; i = (i + 1) % 8) {
    Serial.print("3-bit Counter: ");
    Serial.println(i);
    vTaskDelay(700 / portTICK_PERIOD_MS);
  }
}

Algorithm:
Initialize Serial and configure LED pins as outputs.
Create two tasks:
Task 1: Implement a 2-bit counter using LEDs.
Task 2: Implement a 3-bit counter and print value to Serial.
Update LEDs and Serial with current counter values.
Delay each task for periodic execution.
Repeat counting indefinitely.

🔹 Q5. I2C Master–Slave Communication
Write an Arduino program using the I2C (Inter-Integrated Circuit) protocol to demonstrate
communication between a master and a slave device.
The master Arduino should send a string message ("x is ") followed by an incrementing integer value to the slave Arduino at regular intervals.
The slave Arduino should receive the transmitted data and display it on the serial monitor.
Use appropriate I2C functions such as Wire.begin(), Wire.beginTransmission(), Wire.write(), Wire.endTransmission(), and Wire.onReceive().
Master Code:
#include <Wire.h>

int counter = 0;

void setup() {
  Wire.begin(); 
}

void loop() {
  Wire.beginTransmission(8);
  Wire.write("x is ");
  Wire.write(counter);
  Wire.endTransmission();
  counter++;
  delay(1000);
}


Slave Code:
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
}

void loop() {}

void receiveEvent(int howMany) {
  Serial.print("Received: ");
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println();
}

Algorithm:
 Master:
Initialize I2C as Master.
In a loop, send data (counter or message) to Slave.
Increment counter after each transmission.
Delay before next transmission.
Slave:
Initialize I2C as Slave with a specific address.
Set up an event handler to receive data.
In the event handler, read incoming data and print to Serial.
Wait for the next transmission.


🔹 Q6. I2C RGB LED Blink (Slave controls RGB LED)
Perform Arduino to Arduino communication via I2C to demonstrate RGB LED blink. (RGB LED - Slave).
Master:
#include <Wire.h>
byte color = 0;

void setup() {
  Wire.begin();
}

void loop() {
  Wire.beginTransmission(8);
  Wire.write(color);
  Wire.endTransmission();
  color = (color + 1) % 3;
  delay(1000);
}


Slave:
#include <Wire.h>
#define RED 9
#define GREEN 10
#define BLUE 11

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
}

void loop() {}

void receiveEvent(int howMany) {
  int color = Wire.read();
  digitalWrite(RED, color == 0);
  digitalWrite(GREEN, color == 1);
  digitalWrite(BLUE, color == 2);
}



Algorithm
Master:
Initialize I2C as Master.
Send color code to Slave repeatedly.
Cycle color code in loop.
Delay between transmissions.
Slave:
Initialize RGB LED pins as outputs.
Initialize I2C as Slave with specific address.
Set up receive event handler to read color code.
Turn on corresponding LED based on color code.



🔹 Q7. I2C – 3-bit Counter LEDs
Write an arduino sketch to perform the following using I2C communication.
Interface three LED’s on the slave side.
A counter of 3 bits must be generated on the master side.
Based on the count value, the LED on the slave side must glow.
Master:
#include <Wire.h>

void setup() {
  Wire.begin();
}

void loop() {
  for (int count = 0; count < 8; count++) {
    Wire.beginTransmission(8);
    Wire.write(count);
    Wire.endTransmission();
    delay(1000);
  }
}

Slave:
#include <Wire.h>

int leds[] = {3, 4, 5};

void setup() {
  for (int i = 0; i < 3; i++) pinMode(leds[i], OUTPUT);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
}

void loop() {}

void receiveEvent(int howMany) {
  int count = Wire.read();
  for (int i = 0; i < 3; i++)
    digitalWrite(leds[i], bitRead(count, i));
}

Algorithm
Master:
Initialize I2C as Master.
Send counter value (0–7) to Slave in loop.
Delay between transmissions.
Slave:
Initialize 3 LED pins as outputs.
Initialize I2C as Slave with specific address.
Set up receive event handler to read counter value.
Display counter value on LEDs using bit operations.

🔹 Q8. UART Calculator
Write an Arduino program using Serial Communication (UART) to design a simple calculator that performs basic arithmetic operations.
The user should enter two numbers and an operator (+, -, *, /) through the serial monitor.
The Arduino should read the input using **UART functions (Serial.read(), Serial.parseInt(), etc.), perform the corresponding arithmetic operation, and display the **result** back on the serial monitor.
Implement proper handling for invalid inputs or division by zero.
✅ Answer:
int num1, num2;
char op;

void setup() {
  Serial.begin(9600);
  Serial.println("Enter: number1 operator number2 (e.g., 5 + 3)");
}

void loop() {
  if (Serial.available()) {
    num1 = Serial.parseInt();
    op = Serial.read();
    num2 = Serial.parseInt();

    float result;
    bool valid = true;

    switch (op) {
      case '+': result = num1 + num2; break;
      case '-': result = num1 - num2; break;
      case '*': result = num1 * num2; break;
      case '/': 
        if (num2 == 0) { Serial.println("Error: Divide by 0"); valid = false; }
        else result = (float)num1 / num2;
        break;
      default:
        Serial.println("Invalid operator!");
        valid = false;
    }

    if (valid) {
      Serial.print("Result: ");
      Serial.println(result);
    }
  }
}

Algorithm
Initialize Serial communication.
Wait for input of two numbers and an operator.
Read first number, operator, and second number.
Perform calculation based on operator (+, −, *, /).
Handle divide-by-zero and invalid operator errors.
Print result to Serial.
Repeat for next input.


🔹 Q9. UART 4-Digit PIN Check
Write an arduino sketch to perform the following Serial Communication (UART):
Provide a 4 digit pin via serial monitor.
If the pin is valid print welcome on the serial monitor.
If the pin is invalid, passive buzzer must beep.
✅ Answer:
#define BUZZER 8
String correctPin = "1234";

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER, OUTPUT);
  Serial.println("Enter 4-digit PIN:");
}

void loop() {
  if (Serial.available()) {
    String pin = Serial.readStringUntil('\n');
    pin.trim();
    if (pin == correctPin) {
      Serial.println("Welcome!");
    } else {
      Serial.println("Invalid PIN!");
      tone(BUZZER, 1000, 500);
    }
  }
}

Algorithm
Initialize Serial and configure buzzer pin as output.
Wait for user to input 4-digit PIN.
Read input from Serial.
Compare input with correct PIN.
If correct, print welcome message.
If incorrect, print error and trigger buzzer.
Repeat indefinitely.

🔹 Q10. UART Number & LED
Write an arduino sketch to perform the following Serial Communication (UART):
Enter a number in the serial monitor.
If the number is greater than 50 the red LED must glow.
If the number is less than 50 the green LED must glow.
If the number is 50 yellow LED must glow.
✅ Answer:
#define RED 9
#define GREEN 10
#define YELLOW 11

void setup() {
  Serial.begin(9600);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  Serial.println("Enter a number:");
}

void loop() {
  if (Serial.available()) {
    int num = Serial.parseInt();
    if (num > 50) {
      digitalWrite(RED, HIGH);
      digitalWrite(GREEN, LOW);
      digitalWrite(YELLOW, LOW);
    } else if (num < 50) {
      digitalWrite(GREEN, HIGH);
      digitalWrite(RED, LOW);
      digitalWrite(YELLOW, LOW);
    } else {
      digitalWrite(YELLOW, HIGH);
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
    }
  }
}

Algorithm
Initialize Serial and configure LED pins as output.
Wait for user to input a number.
Read number from Serial.
Compare number with threshold (e.g., 50).
Turn on corresponding LED based on value:
Greater than threshold → RED
Less than threshold → GREEN
Equal → YELLOW
Repeat for next input.

