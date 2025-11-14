notion link : https://budidhabhanuchander.notion.site/tasks-2aa85094ccdc80959665df42a7ac5e46

Task1:  https://wokwi.com/projects/446697471894949889   street light
Task2:  https://wokwi.com/projects/446698333867651073   ultrasonic sensor
Task3:  https://wokwi.com/projects/446699975541728257   soil moisture sensor
Task4:  https://www.tinkercad.com/things/lEpiHz9c3vs/editel?sharecode=0wuNOu2ja1RDr155K5EepEAtbjrjPVOM9Y25LxfURPA  motor
Task5:  https://wokwi.com/projects/447250584681122817   curtail lcd display
Task6:  https://wokwi.com/projects/447147116511380481   acc , gyro ,temp
Task7a: https://creator.kodular.io/#6164201635577856    kodular
Task7b: https://creator.kodular.io/#5207490087092224
Task7c: https://creator.kodular.io/#4722664582152192
Task8:  https://wokwi.com/projects/447151567891194881   ultra using esp
Task9:  https://wokwi.com/projects/446602330153298945   street light esp
Task10: https://wokwi.com/projects/447302364359157761   soil moisture using esp(go and write mam's classroom)
task 11 https://wokwi.com/projects/409312856465230849   (hard)Internet enabled Display of Ambient Parameter
Task12: https://wokwi.com/projects/446601917061437441   home security door


mciot tasks

https://wokwi.com/projects/444674746115127297
ultrasonic firebase task task 8

task 6 
https://wokwi.com/projects/383981664359646209


task 10 
https://wokwi.com/projects/415371687258065921

task 12
https://wokwi.com/projects/383260777330575361

task 9
https://wokwi.com/projects/375833305298579457

task 11
https://wokwi.com/projects/409312856465230849


- task1  street light
    
    ![image.png](attachment:b4b79f74-3530-45a8-b889-238522a38ab5:image.png)
    
    ![image.png](attachment:e9158af2-f9b4-4f21-9796-85950febc96e:image.png)
    
    ```
    // C++ code
    //
    void setup() {
    pinMode(A0,INPUT);
    Serial.begin(9600);
    pinMode(8,OUTPUT);
    }
    void loop() {
    // Serial.print("LDR = ");
    int i=analogRead(A0);
    Serial.println(i);
    if(i>60) digitalWrite(8,1);
    else
    digitalWrite(8,0);
    delay(500);
    }
    ```
    
- task2 ultrasonic
    
    ![image.png](attachment:ce3a735a-78ce-4b65-b6d0-9f40d37a57a8:image.png)
    
    ```jsx
    // Ultrasonic Distance Measurement using HC-SR04 (No external library)
    
    const int trigPin = 12;   // Trigger pin connected to D11
    const int echoPin = 11;   // Echo pin connected to D12
    
    long duration;             // Variable to store pulse duration
    int distance;              // Variable to store calculated distance
    
    void setup() {
      pinMode(trigPin, OUTPUT); // Trigger pin as output
      pinMode(echoPin, INPUT);  // Echo pin as input
      Serial.begin(9600);       // Initialize serial monitor
    }
    
    void loop() {
      // Clear the trigger pin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
    
      // Send a 10µs pulse to trigger the ultrasonic sensor
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
    
      // Measure the time of the reflected pulse
      duration = pulseIn(echoPin, HIGH);
    
      // Calculate the distance in centimeters
      distance = duration * 0.034 / 2;
    
      // Print the result to Serial Monitor
      Serial.print("The distance is: ");
      Serial.print(distance);
      Serial.println(" cm");
    
      delay(1000); // Wait 1 second before next measurement
    }
    
    ```
    
- task3  soil moisture
    
    ![image.png](attachment:cfe901db-6ae5-4e08-b117-707e28d29bdb:image.png)
    
    ```jsx
    // C++ code
    //
    int sensorPin = A0;      // Soil moisture sensor connected to A0
    int ledPin = 13;         // LED connected to D13
    int buzzerPin = 8;       // Buzzer connected to D8
    int sensorValue = 0;     // Variable to store sensor value
    
    void setup() {
      pinMode(ledPin, OUTPUT);
      pinMode(buzzerPin, OUTPUT);
      Serial.begin(9600);    // For monitoring values
    }
    
    void loop() {
      sensorValue = analogRead(sensorPin);   // Read soil sensor
      Serial.print("Soil Moisture Value: ");
      Serial.println(sensorValue);
    
      if (sensorValue < 500) {  // If soil is dry
        digitalWrite(ledPin, HIGH);
        digitalWrite(buzzerPin, HIGH);
      } else {                  // If soil is wet
        digitalWrite(ledPin, LOW);
        digitalWrite(buzzerPin, LOW);
      }
    
      delay(1000);
    }
    
    ```
    
- task4  motor
    
    ![image.png](attachment:55cb829f-90f6-4a8f-bdfb-035ac2447be0:image.png)
    
    ![image.png](attachment:4c8351a5-5a36-4843-9b94-c7bfc0134efa:image.png)
    
    ```jsx
    void setup() {
      pinMode(9, OUTPUT);  // Input 2 of L293D
      pinMode(10, OUTPUT); // Input 1 of L293D
    }
    
    void loop() {
      // Rotate motor clockwise
      digitalWrite(9, HIGH);
      digitalWrite(10, LOW);
      delay(3000); // Rotate for 3 seconds
    
      // Stop motor
      digitalWrite(9, LOW);
      digitalWrite(10, LOW);
      delay(1000); // Stop for 1 second
    
      // Rotate motor counter-clockwise
      digitalWrite(9, LOW);
      digitalWrite(10, HIGH);
      delay(3000); // Rotate for 3 seconds
    
      // Stop motor
      digitalWrite(9, LOW);
      digitalWrite(10, LOW);
      delay(1000); // Stop for 1 second
    }
    
    ```
    
- task5 curtail lcd
    
    ![WhatsApp Image 2025-11-13 at 21.19.55_88c57429.jpg](attachment:b7f93f72-6158-40b4-83c9-5f4a3c48f2bc:WhatsApp_Image_2025-11-13_at_21.19.55_88c57429.jpg)
    
    ```jsx
    #include <LiquidCrystal.h>
    
    // Initialize LCD: (RS, E, D4, D5, D6, D7)
    LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
    
    void setup() {
      lcd.begin(16, 2); // Initialize LCD 16x2
      lcd.print("Hello, World!");
    }
    
    void loop() {
      lcd.setCursor(0, 1); // Move to second line
      lcd.print("Tinkercad Demo");
      delay(1000);
    }
    
    ```
    
- task6  gyro
    
    ![image.png](attachment:8b05d642-6ac9-4cc6-b694-a6b1cefcf1d0:image.png)
    
    ```jsx
    #include<Adafruit_SSD1306.h>
    #include<Adafruit_MPU6050.h>
    #include<Adafruit_Sensor.h>
    #include<Wire.h>
    
    Adafruit_SSD1306 display(128,64,&Wire);
    Adafruit_MPU6050 mpu;
    
    void setup(){
      Serial.begin(115200);
      mpu.begin();
      display.begin(SSD1306_SWITCHCAPVCC,0x3C);
      display.setTextColor(WHITE);
    }
    
    void loop(){
      sensors_event_t a,g,t;
      mpu.getEvent(&a,&g,&t);
      Serial.printf("A: %.1f %.1f %.1f | G: %.1f %.1f %.1f | T:%.1f\n",
      a.acceleration.x,a.acceleration.y,a.acceleration.z,g.gyro.x,g.gyro.y,
      g.gyro.z,t.temperature);
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("A:");
      display.print(a.acceleration.x,1);
      display.print(" ");
      display.print(a.acceleration.y,1);
      display.print(" ");
      display.print(a.acceleration.z,1);
      display.println();
      display.print("G:");
      display.print(g.gyro.x,1);
      display.print(" ");
      display.print(g.gyro.y,1);
      display.print(" ");
      display.print(g.gyro.z,1);
      display.println();
      display.print("T:");
      display.print(t.temperature,1);
      display.println();
      display.display();
      delay(200);
    }
    ```
    
- task8
    
    ![image.png](attachment:debf00db-adc3-495c-b9b8-49ab61fda44d:image.png)
    
    ```jsx
    int trig = 18;
    int echo = 5;
    long duration;
    int distance;
    
    void setup() {
      pinMode(trig, OUTPUT);
      pinMode(echo, INPUT);
      Serial.begin(9600);
    }
    
    void loop() {
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      duration = pulseIn(echo, HIGH);
      distance = duration * 0.034 / 2;
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      delay(500);
    }
    
    ```
    
- task 9
    
    ![image.png](attachment:41247e58-e1f6-4d12-bf1f-981d39c2f7b0:image.png)
    
    ```jsx
    #define LIGHT_SENSOR_PIN 36 // ESP32 pin GIOP36 VP (ADC0)
    
    void setup() {
      // initialize serial communication at 9600 bits per second:
      Serial.begin(9600);
    }
    
    void loop() {
      // reads the input on analog pin (value between 0 and 4095)
      int analogValue = analogRead(LIGHT_SENSOR_PIN);
    
      Serial.print("Analog Value = ");
      Serial.print(analogValue);  
      if (analogValue < 40) {
        Serial.println(" => Dark");
      } else if (analogValue < 800) {
        Serial.println(" => Dim");
      } else if (analogValue < 2000) {
        Serial.println(" => Light");
      } else if (analogValue < 3200) {
        Serial.println(" => Bright");
      } else {
        Serial.println(" => Very bright");
      }
    
      delay(500);
    }
    
    ```
    
- task10(optimised)
    
    ![image.png](attachment:4129dee4-eff9-4556-a062-bbcc64380cf2:image.png)
    
    ```python
    from machine import Pin, I2C, ADC, PWM
    import ssd1306, time
    
    # ---- Pin Setup ----
    buzzer = PWM(Pin(26))
    soil = ADC(Pin(34))
    
    leds = {
        "wet": Pin(32, Pin.OUT),
        "ideal": Pin(33, Pin.OUT),
        "dry": Pin(25, Pin.OUT)
    }
    
    # ---- OLED Setup ----
    i2c = I2C(0, scl=Pin(22), sda=Pin(21))
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
    
    # ---- Helper Functions ----
    def beep(freq=None):
        if freq:
            buzzer.freq(freq)
            buzzer.duty(50)
            time.sleep(0.5)
        buzzer.duty(0)
    
    def show_oled(text, value):
        oled.fill(0)
        oled.text("Soil Moisture", 0, 0)
        oled.text("Percentage: {}%".format(value), 0, 20)
        oled.text("Status: " + text.upper(), 0, 40)
        oled.show()
    
    def set_led(state):
        leds["wet"].value(1 if state=="wet" else 0)
        leds["ideal"].value(1 if state=="ideal" else 0)
        leds["dry"].value(1 if state=="dry" else 0)
    
    # ---- Main Loop ----
    while True:
        # Read & convert sensor value (0–100%)
        raw = soil.read()
        percent = int((raw - 2165) / (3135 - 2165) * 100)
        percent = max(0, min(percent, 100))
    
        # Conditions
        if percent < 40:
            status = "wet"
            beep(400)
        elif percent > 60:
            status = "dry"
            beep(2000)
        else:
            status = "ideal"
            beep(None)
    
        # Update LED + OLED
        set_led(status)
        print(status.upper())
        show_oled(status, percent)
    
        time.sleep(1)
    
    ```
    
- task10(mam)
    
    ```jsx
    # STEP 1 : IMPORT MODULES OR LIBRARY
    from machine import Pin, I2C, ADC, PWM
    import ssd1306
    import time
    
    #STEP 2: DECLARE CONNECTION
    # ESP32 Pin assignment 
    
    buzzer = PWM(Pin(26), Pin.OUT)
    soil_sensor = ADC(Pin(34))  # Analog pin for soil moisture sensor
    orangeled = Pin(32,Pin.OUT)
    yellowled = Pin(33,Pin.OUT)
    greenled = Pin(25,Pin.OUT)
    
    #STEP 2.2 : DECLARE THE CONNECTION OLED
    i2c = I2C(0, scl=Pin(22), sda=Pin(21))
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
    
    #STEP 3 : THE PROCESS
    while True:
       # Read the soil moisture sensor value
        sensorValue = soil_sensor.read()
        
        # Map the sensor value to a percentage (assuming 2165-3135 range)
        humidityPercent = int((sensorValue - 2165) / (3135 - 2165) * 100)
        humidityPercent = max(0, min(humidityPercent, 100))  # Clamp to range 0-100
    
        # Determine the state based on humidity percentage
        if humidityPercent < 40:
            print("WET")
            orangeled.on()                              # Turn on red LED
            yellowled.off()                             # Turn off yellow LED
            greenled.off()                              # Turn off green LED
            buzzer.init( freq = 400, duty = 50)         # Turn on buzzer
            time.sleep(0.5)                             # Keep buzzer on for half a second
            buzzer.init(freq=1, duty=0)                 # Turn off buzzer
    
        elif humidityPercent > 60:
            print("DRY")
            orangeled.off()                            # Turn off red LED
            yellowled.off()                            # Turn off yellow LED
            greenled.on()                              # Turn on green LED
            buzzer.init( freq = 2000, duty = 50)        # Turn on buzzer at higher frequency
            time.sleep(0.5)                            # Keep buzzer on for half a second
            buzzer.init(freq=1, duty=0)                # Turn off buzzer
    
        else:
            print("IDEAL")
            orangeled.off()                   # Turn off red LED
            yellowled.on()                    # Turn on yellow LED
            greenled.off()                    # Turn off green LED
            buzzer.init(freq=1, duty=0)       # Ensure buzzer is off
    
        # Display the status on the OLED display
        oled.fill(0)  # Clear the display
        oled.text("Soil Moisture", 0, 0)
        oled.text("Percentage: {}%".format(humidityPercent), 0, 20)
        oled.show()
    
        time.sleep(1)  # Delay before next reading cycle
    
    ```
    
- task11 (optimised)
    
    ![image.png](attachment:86b2b12a-6d13-426f-9cec-f100218fe986:image.png)
    
    ```jsx
    #include <Wire.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    #include <Adafruit_BMP280.h>
    
    #define WIDTH 128
    #define HEIGHT 64
    #define BMP_SCK 18
    #define BMP_MISO 19
    #define BMP_MOSI 23
    #define BMP_CS 5
    
    Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);
    Adafruit_SSD1306 display(WIDTH, HEIGHT, &Wire, -1);
    
    // ------------- Helper: Print 2 lines of sensor data -------------
    void showOnOLED(float temp, float pressure) {
      display.clearDisplay();
      display.setTextColor(WHITE);
    
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("Temperature:");
    
      display.setTextSize(2);
      display.setCursor(0, 10);
      display.print(temp);
      display.print(" C");
    
      display.setTextSize(1);
      display.setCursor(0, 35);
      display.print("Pressure:");
    
      display.setTextSize(2);
      display.setCursor(0, 45);
      display.print(pressure);
      display.print(" Pa");
    
      display.display();
    }
    
    void setup() {
      Serial.begin(115200);
    
      // -------- BMP280 Setup --------
      if (!bmp.begin()) {
        Serial.println("BMP280 not found! Check wiring.");
        while (1) delay(10);
      }
    
      bmp.setSampling(
          Adafruit_BMP280::MODE_NORMAL,
          Adafruit_BMP280::SAMPLING_X2,
          Adafruit_BMP280::SAMPLING_X16,
          Adafruit_BMP280::FILTER_X16,
          Adafruit_BMP280::STANDBY_MS_500
      );
    
      // -------- OLED Setup --------
      if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 initialization failed!");
        while (1);
      }
    
      display.clearDisplay();
      display.display();
    }
    
    void loop() {
      float temp = bmp.readTemperature();
      float pressure = bmp.readPressure();
    
      // Serial Monitor Output
      Serial.print("Temp: ");
      Serial.print(temp);
      Serial.print(" *C | Pressure: ");
      Serial.print(pressure);
      Serial.println(" Pa");
    
      // OLED Output
      showOnOLED(temp, pressure);
    
      delay(1000); // update every second
    }
    
    ```
    
- task12(abdul)
    
    ![image.png](attachment:f8809004-a44e-4c63-81c1-8fefe5f55d87:image.png)
    
    ```arduino
    // Pin declarations
    #define TRIG 14
    #define ECHO 25
    #define BUZZER 15
    #define YELLOW_LED 5
    #define GREEN_LED 2
    
    long duration;
    float distance;
    
    void setup() {
      Serial.begin(9600);
      
      pinMode(TRIG, OUTPUT);
      pinMode(ECHO, INPUT);
      pinMode(BUZZER, OUTPUT);
      pinMode(YELLOW_LED, OUTPUT);
      pinMode(GREEN_LED, OUTPUT);
    
      Serial.println("Home Safety and Security System Initialized");
    }
    
    void loop() {
      // Trigger the ultrasonic sensor
      digitalWrite(TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);
    
      // Read the echo pulse
      duration = pulseIn(ECHO, HIGH);
      distance = (duration * 0.0343) / 2; // Convert time to distance (cm)
    
      // Print the distance
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    
      // Determine safety level
      if (distance > 200) {
        Serial.println("✅ Safe - No one near the door");
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(YELLOW_LED, LOW);
        noTone(BUZZER);
      }
      else if (distance >= 50 && distance <= 200) {
        Serial.println("⚠️  Someone approaching the door!");
        digitalWrite(YELLOW_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
        tone(BUZZER, 1000);
        delay(200);
        noTone(BUZZER);
      }
      else {
        Serial.println("🚨 Person very close to the door!");
        digitalWrite(YELLOW_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
        tone(BUZZER, 1500);
        delay(300);
        noTone(BUZZER);
      }
    
      delay(1000);
    }
    
    ```
    
- task12(mam)
    
    ![image.png](attachment:0c1b3e64-c0e5-4221-b3f6-eb21c1d12e9e:image.png)
    
    ```python
    import library_ultrasensor
    import Oled_library
    from machine import Pin, SoftI2C, PWM
    from utime import sleep
    
    # ---- Pin Setup ----
    yellow = Pin(5, Pin.OUT)
    green = Pin(2, Pin.OUT)
    TRIG = Pin(14)
    ECHO = Pin(25)
    buzz_pin = Pin(15)
    pir = Pin(13)
    
    oled_i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
    screen = Oled_library.SSD1306_I2C(128, 64, oled_i2c)
    
    # ---- Ultrasonic Sensor Object ----
    usonic = library_ultrasensor.HCSR04(trigger_pin=TRIG, echo_pin=ECHO)
    
    # ---- Helper Functions ----
    def beep(freq=0, time_on=0.2):
        buz = PWM(buzz_pin)
        if freq:
            buz.freq(freq)
            buz.duty(400)
            sleep(time_on)
        buz.duty(0)
    
    def show(msg1, msg2):
        screen.fill(1)
        screen.text(msg1, 10, 20, 0)
        screen.text(msg2, 10, 40, 0)
        screen.show()
    
    def blink(pin, t=0.2, times=3):
        for _ in range(times):
            pin.on()
            sleep(t)
            pin.off()
            sleep(t)
    
    # ---- Main Program ----
    while True:
        # Read distance
        dist = usonic.distance_cm()
        print("Object detected:", dist, "cm")
    
        # DISTANCE CONDITIONS
        if dist > 200:
            print("Someone is coming (range > 200)")
            beep(500, 0.5)
            show("Someone is coming:/", "in range 200")
            blink(yellow, 0.5)
    
        elif 50 <= dist < 200:
            print("Someone is coming (<200 cm)")
            beep(1000, 0.1)
            show("Someone is coming:/", "below range 200")
            blink(yellow, 0.1)
    
        else:
            print("Safe zone")
            beep(1400, 0.01)
            blink(yellow, 0.01, 1)
    
        # PIR MOTION DETECTION
        motion = pir.value()
        if motion:
            print("\nSOMEONE CROSSED THE DOOR")
            show("someone crossed:/", "the door!!!")
            blink(green, 1.5)
            sleep(2)
    
        else:
            print("\nNO ONE APPROACHED THE DOOR")
            show("no one approached:/", "the door")
    
    ```
