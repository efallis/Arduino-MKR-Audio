#define THRESHOLD 200
#define THRESHOLD_TIME 5000 // Time in ms after threshold is broken, resets if threshold is broken again
#define MAG_QUEUE_SIZE 50 // Averages the amount of samples
#define MAG_SAMPLE_SIZE 50  // Find the max value out of samples
#define SLEEP true  // Sleep functionality
#define SERIAL_USE true  // Log to the serial port, screws up when the MCU sleeps

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoLowPower.h>

double audioMax;
unsigned long startTime;
double threshold;
char ssid[] = "";        // your network SSID (name)

int status = WL_IDLE_STATUS;
char server[] = "";
WiFiClient client;

void setup() {
  if (SERIAL_USE){
    //Initialize serial and wait for port to open:
    Serial.begin(9600);
    while (!Serial) {} // wait for serial port to connect. Needed for native USB port only
    delay(200);
    Serial.println("Serial Started");
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    printSerial("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // Attaching pin interrupt
  LowPower.attachInterruptWakeup(0, wakeUp, CHANGE);

  digitalWrite(LED_BUILTIN, HIGH);

  // Enable Wi-Fi
  wifiConnect();
}

void loop() {
  printSerial("Start Loop");
  delay(1000);

  // Sleep if interrupt pin is high
  if (SLEEP && digitalRead(0) == HIGH){
    printSerial("Sleep Time");
    // Completely turn off the wifi module
    wifiDisconnect();
    digitalWrite(LED_BUILTIN, LOW);
    LowPower.deepSleep();
    digitalWrite(LED_BUILTIN, HIGH);
    printSerial("Wake Time");

    // Enable Wi-Fi
    wifiConnect();
  }
  
  startTime = millis();
  while (abs(millis() - startTime) < THRESHOLD_TIME || not(SLEEP)){
    double mag = updateAverageMag();

    client.stop();
    if (client.connect(server, 33333)){
      printSerial("Connected to Server");
      
      // Make an HTTP request:
      client.println(String("") + "POST /.uploadMkr.php?project=mkrAudio&val1=" + mag + " HTTP/1.1");
      client.println("Host: " + server);
      client.println("Connection: close");
      client.println();

      if (SERIAL_USE){
        Serial.println(String("") + "Transmitted: " + mag);
      }
    } else{
      printSerial("Can't to Server");
    }

    // Refresh the timer
    if (analogRead(1) < THRESHOLD){
      startTime = millis();
    }
  }
}

void printSerial(char message[]){
  if (SERIAL_USE){
    Serial.println(message);
  }
}

bool wifiConnect(){
  bool wifiConnected = false;
  
  while(!wifiConnected){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  
    // attempt to connect to Wifi network:
    status = WL_IDLE_STATUS;
    while (status != WL_CONNECTED) {
      // Connect to WEP network:
      status = WiFi.begin(ssid);
  
      // wait 1 second for connection:
      delay(1000);
    }
    
    if (client.connect(server, 33333)) {
      //Serial.println("Connected to server");
      wifiConnected = true;
    }else{
      //Serial.println("Cannot connect to server");
      wifiConnected = false;
    }
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

bool wifiDisconnect(){
  client.stop();
  WiFi.end();
}

void wakeUp(){}

double updateAverageMag(){
  double averageMag;
  double i;
  double j;
  int val;
  
  averageMag = 0;
  for(i=0;i<MAG_QUEUE_SIZE;i++){

    //Get local maximum
    audioMax = 0;  
    for(j=0;j<MAG_SAMPLE_SIZE;j++){
      
        val = analogRead(0);
        if (audioMax < val)
        {
          audioMax = val;
        }
    }
    averageMag += audioMax;
  }

  averageMag /= MAG_QUEUE_SIZE;
  return averageMag;
}
