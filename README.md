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
  Wire.begin();            // Start I2C as Master
  pinMode(13, OUTPUT);     // Built-in LED
  Serial.begin(9600);
  Serial.println("Master Ready");
}

void loop() {
  Wire.beginTransmission(8);   // Slave address = 8
  Wire.write("x is ");         // Send text
  Wire.write(counter);         // Send number as a byte
  Wire.endTransmission();      // End I2C transmission

  Serial.print("Sent: x is ");
  Serial.println(counter);

  digitalWrite(13, HIGH);      // Blink LED to show sending
  delay(100);
  digitalWrite(13, LOW);

  counter++;
  delay(1000);                 // Wait 1 second
}

Slave Code:
#include <Wire.h>

void setup() {
  Wire.begin(8);               // Join I2C bus with address #8
  Wire.onReceive(receiveEvent); // Register event
  Serial.begin(9600);
  pinMode(13, OUTPUT);         // Built-in LED
  Serial.println("Slave Ready");
}

void loop() {
  // Nothing here - all happens when data arrives
}

void receiveEvent(int howMany) {
  digitalWrite(13, HIGH);      // LED ON when data received
  Serial.print("Received: ");
  while (Wire.available()) {
    char c = Wire.read();      // Read each byte
    Serial.print(c);
  }
  Serial.println();
  digitalWrite(13, LOW);       // Turn LED off
}

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
