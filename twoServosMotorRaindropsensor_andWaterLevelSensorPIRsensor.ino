#include <ESP32Servo.h>

Servo myservo1;          // create first servo object
Servo myservo2;          // create second servo object

int servoPin1 = 18;      // GPIO pin for first servo
int servoPin2 = 5;       // GPIO pin for second servo

int raindrops = 15;      // Analog pin for rain sensor
int waterLevelSensor = 34; // Analog pin for water level sensor
int pinLED = 26;         // LED indicator pin

#define PIR_SENSOR_PIN 23   // PIR sensor pin
#define buzzer 4            // Buzzer pin

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  Serial.println("System initializing...");
  delay(500);

  // Allow allocation of all timers for ESP32 servo control
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Configure both servos
  myservo1.setPeriodHertz(50);  // Standard 50 Hz servo signal
  myservo2.setPeriodHertz(50);

  // Attach both servos to their respective pins
  myservo1.attach(servoPin1, 500, 2400); // min and max pulse width
  myservo2.attach(servoPin2, 500, 2400);

  // Start both servos at 0 degrees
  myservo1.write(0);
  myservo2.write(0);

  // Pin modes configuration
  pinMode(raindrops, INPUT);
  pinMode(waterLevelSensor, INPUT);
  pinMode(pinLED, OUTPUT);
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(buzzer, OUTPUT);

  Serial.println("System ready!");
  Serial.println("------------------------------");
}

void loop() {
  // Read all sensors
  int motionState = digitalRead(PIR_SENSOR_PIN);     // PIR sensor state
  int analogdropsvalue = analogRead(raindrops);      // Rain sensor analog value
  int waterLevelSensorvalue = analogRead(waterLevelSensor); // Water level analog value

  // Display readings on Serial Monitor
  Serial.print("Rain Sensor: ");
  Serial.print(analogdropsvalue);
  Serial.print("  |  Water Level: ");
  Serial.print(waterLevelSensorvalue);
  Serial.print("  |  Motion: ");
  Serial.println(motionState == HIGH ? "Detected" : "None");

  // Condition: rain detected, high water level, or motion
  if (motionState == HIGH || analogdropsvalue < 2500 || waterLevelSensorvalue > 350) {
    Serial.println("⚠️ Condition Triggered! Activating system...");
    digitalWrite(buzzer, HIGH);
    digitalWrite(pinLED, HIGH);
    myservo1.write(180);
    myservo2.write(180);
  } else {
    Serial.println("✅ Normal condition. System idle.");
    digitalWrite(buzzer, LOW);
    digitalWrite(pinLED, LOW);
    myservo1.write(0);
    myservo2.write(0);
  }

  Serial.println("------------------------------");
  delay(500); // Wait half a second before next reading
}
