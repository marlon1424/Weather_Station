//Implementing Blynk
#define BLYNK_TEMPLATE_ID "TMPLwJTiQ1mH"
#define BLYNK_TEMPLATE_NAME "Weather Station"
#define BLYNK_AUTH_TOKEN "dyuo9gZoKl9NysI_fBlgWNlbhiJgiydQ"

//include libraries
#include <Wire.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_BME280.h>
#include <hp_BH1750.h> 


// Your WiFi credentials.
char ssid[] = "EDA-IOT";
char pass[] = "3aB1J27M";

//Blynk authentication token and API
char auth[] = "dyuo9gZoKl9NysI_fBlgWNlbhiJgiydQ";
const char* server = "api.blynk.com";


//Pin defintions
const int MUX_A = 13; 
const int MUX_B = 12;
const int MUX_C = 11;
const int readPin = A0;

const int pinNums = 8;

//vector value representing ports 0 to 7 of multiplexer
int val[pinNums];

 float wind_Direction_Angle = 0;
 String wind_Direction_Rose;
 
 //Wind vane array
 String windRose[16] = {"N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW"};
 float windAng[16] = {0,22.5,45,67.5,90,112.5,135,157.5,180,202.5,225,247.5,270,292.5, 315,337.5};

//2D array for reed switches
int reedSwitchReadings [16] [pinNums] = 
  { {1,0,0,0,0,0,0,0},
    {1,1,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0},
    {0,1,1,0,0,0,0,0},
    {0,0,1,0,0,0,0,0},
    {0,0,1,1,0,0,0,0},
    {0,0,0,1,0,0,0,0},
    {0,0,0,1,1,0,0,0},
    {0,0,0,0,1,0,0,0},
    {0,0,0,0,1,1,0,0},
    {0,0,0,0,0,1,0,0},
    {0,0,0,0,0,1,1,0},
    {0,0,0,0,0,0,1,0},
    {0,0,0,0,0,0,1,1},
    {0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,1},
  };

//2D array for multiplexer combinations
   int muxABC[8] [3] = 
  { {0,0,0},
    {0,0,1},
    {0,1,0},
    {0,1,1},
    {1,0,0},
    {1,0,1},
    {1,1,0},
    {1,1,1},
  };

  // anemometer parameters
 volatile byte rpmCount;   // count signal. A signal is detected every time the magnet interacts with the hall sensor
 volatile unsigned long lastMicros;
 unsigned long timeOld;
 unsigned long timeMeasure = 25.00; // seconds taken to measure winf direction
 unsigned long timeNow;
 int countThing = 0;
 int interPin = 8;  //Interrupt pin
 float rpm, rps;         // rotations per minute and per second
 float radius = 0.90; // meters - measure of the lenght of each the anemometer wing
 float linearVelocity = 0; // m/sec
 float linearVelocityKMH; // km/h
 float omega = 0; // rad/sec

// I2C sensors
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_BME280 bme; 
hp_BH1750 BH1750; 

void setup() {
  //Sets multiplexer select lines as outputs
   pinMode(MUX_A, OUTPUT);
   pinMode(MUX_B, OUTPUT);     
   pinMode(MUX_C, OUTPUT);
   
  Serial.begin(9600);
  
  //Initializes temperature sensor
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }

  delay(1000);
  //Initializes humidity sensor
   if(!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        while (1) delay(10);
    }
    //Initializes lux sensor
     if (!BH1750.begin(BH1750_TO_GROUND)) {
    Serial.println("No BH1750 sensor found!");
    while (true) {};                                        
  }

  delay(100);
  
    //Initializes UV index sensor
  if (!uv.begin()) {
    Serial.println("Didn't find Si1145");
    while (1);
  }

   tempsensor.setResolution(3);

   delay(100);
  //Connects to Wifi
   Serial.print("Connecting to Wifi");
   int status = WiFi.status();
   while (status != WL_CONNECTED){
      status = WiFi.begin(ssid, pass);
      Serial.print(".");
      delay(100);
      
    } 
    Serial.println();
    Serial.print("Connected with IP:");
    Serial.println(WiFi.localIP());
    Serial.println();
    //Initializes Blynk communication
    Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);

    delay(1000);

    
   detachInterrupt(digitalPinToInterrupt(interPin));  // force to initiate Interrupt on zero
   attachInterrupt(digitalPinToInterrupt(interPin), rpmAnemometer, RISING); //Initializes the intterrupt pin
   
   rpmCount = 0;
   rpm = 0;
   timeOld = 0;
   timeNow = 0;
}

void loop() {
   //Starts Blynk communication routine
   Blynk.run();
  //Gets reading from I2C sensors
  printMCP9808();
  printBME();
  printBH1750();
  //printSI1145()

  //Measure RPM
   if ((millis() - timeOld) >= timeMeasure*1000){ 
      //countThing ++;
      detachInterrupt(digitalPinToInterrupt(interPin));        // Disable interrupt when calculating
      rps = float(rpmCount)/float(timeMeasure);       // rotations per second
      rpm = 60*rps;                         // rotations per minute
      omega = 2*PI*rps;                     // rad/seg
      linearVelocity = omega*radius;       // m/sec
      linearVelocityKMH = linearVelocity * 3.6;  // km/h
      
      Serial.println("RPM count: ");
      Serial.println(rpmCount);      
      Serial.print("RPS: ");
      Serial.println(rps);      
      Serial.print("Velocity: ");      
      Serial.println(linearVelocity);      
      Serial.print("Velocity(KMH):");            
      Serial.println(linearVelocityKMH); 
      //Send measured data to Blynk database
      Blynk.virtualWrite(V5, linearVelocityKMH);

      timeOld = millis();
      rpmCount = 0;
      attachInterrupt(digitalPinToInterrupt(interPin), rpmAnemometer, RISING); // enable interrupt
  }

   // read windVane direction
      readMux();  // function to read multiplexer and change the vector of directions
      for (int pin = 0; pin < pinNums; pin++){
          val[pin] = map(val[pin], 0, 1023, 0, 10);
  
          if (val[pin] <= 4) {
            val[pin] = 0;
          }
          else {
            val[pin] = 1;  
          }
          delay(100);
      }
      
      int testResult = binToDec(val[pinNums], pinNums); // transform result to DEC  and compare
      for (int i = 0; i < pinNums*2; i++){
        int testDigit = binToDec(reedSwitchReadings[i], pinNums);
        if (testResult == testDigit){
            wind_Direction_Rose = windRose[i];
            wind_Direction_Angle = windAng[i];
            Serial.print("Direction: ");
            Serial.println(windRose[i]); 
            //Send measured data to Blynk database
            Blynk.virtualWrite(V6, windRose[i]);
            delay(500);
            break;
        }
    }

  delay(1000);

}

void printMCP9808(){
  tempsensor.wake(); 

  // Read and print out the temperature, also shows the resolution mode used for reading.
  tempsensor.getResolution();
  float celsius = tempsensor.readTempC();
  float fahrenheit = tempsensor.readTempF();

  String degreesCelsius = String(celsius, 2);
  String degreesFahrenheit = String(fahrenheit, 2);

  Serial.print("Temp(Celsius): "); 
  Serial.print(degreesCelsius); 
  Serial.println(" °C"); 
  Serial.print("Temp(Fahrenheit): "); 
  Serial.print(degreesFahrenheit); 
  Serial.println("°F");
  //Send measured data to Blynk database
  Blynk.virtualWrite(V0, degreesCelsius);
  Blynk.virtualWrite(V1, degreesFahrenheit);
  
  delay(100);
  tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
  }

void printBME() {

    Serial.print("Pressure = ");
    float pressureReading = bme.readPressure() / 100.0F;
    Serial.print(pressureReading);
    Serial.println(" hPa");
    //Send measured data to Blynk database
    Blynk.virtualWrite(V2, pressureReading);

    Serial.print("Humidity = ");
    float humidityReading = bme.readHumidity();
    Serial.print(humidityReading);
    Serial.println(" %");
    //Send measured data to Blynk database
    Blynk.virtualWrite(V3, humidityReading);

    delay(100);
}

void printBH1750(){
    BH1750.start();   //starts a measurement
  float lux = BH1750.getLux(); //  waits until a conversion finished
  Serial.print("Lux: ");
  Serial.println(lux);
  String dayTime;
  if (lux <= 1){
      dayTime = "Night Time";
    } else {
        dayTime= "Day Time";
      }  
      //Send measured data to Blynk database
     Blynk.virtualWrite(V4, dayTime);

     delay(100);
}

void printSI1145(){
  float UVindex = uv.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;  
  Serial.print("UV: ");  Serial.println(UVindex);
  //Send measured data to Blynk database
  Blynk.virtualWrite(V8, UVindex);
  }

// This function is called whenever a magnet/interrupt is detected by the Arduino board
void rpmAnemometer(){
    if(long(micros() - lastMicros) >= 5000){      // time to debounce measures
        rpmCount++;
        lastMicros = micros();
    }    
   Serial.println("***** detect *****");
 }

 
// read 8 pin of multiplexer according to A B C combination
void readMux() {   
  int i;
  for (i = 0; i < pinNums; i++){
    changeMux(muxABC[i][0],muxABC[i][1],muxABC[i][2]);
    val[i] = analogRead(readPin); //Value of the sensor connected pin[i] of Mux
    delay(100);
  }
}
//Sets multiplexer A B C select outputs
void changeMux(int a, int b, int c) {
  if (a == 0){ 
      digitalWrite(MUX_A, HIGH);
    }else {
      digitalWrite(MUX_A, LOW);
    }

    if (b == 0){ 
      digitalWrite(MUX_B, HIGH);
    }else {
      digitalWrite(MUX_B, LOW);
    }

    if (c == 0){ 
      digitalWrite(MUX_C, HIGH);
    }else {
      digitalWrite(MUX_C, LOW);
    }
    delay(500);
 }   
 
 //Function to transform binary vector to decimal for camparison between mux signal read and windvane
 int binToDec(int orig[], int temp){ 
     int result = 0;
    int base = 1; // the base for the first bit is 2^0

    for (int i = temp - 1; i >= 0; i--) {
        result += orig[i] * base;
        base *= 2;
    }

    return result;
    delay(100);
}  
