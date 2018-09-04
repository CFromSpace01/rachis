#include <avr/dtostrf.h>

//lib required #includes
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Wire.h>
#include <Adafruit_INA219.h>

#include <SPI.h>
#include <RH_RF95.h>

#define sensorid "eplbrd2"
//#define sensorid "TSTPOS1"

#define donePin A0
#define randomPin A1
#define somsPin A3
#define sendTrials 5

//lib required #define and global variable declarations
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 433.0

Adafruit_BNO055 bno = Adafruit_BNO055();

int AXLadr = 0x28;

Adafruit_INA219 ina219;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup(void)
{
  Serial.begin(9600);
  delay(2000);
  Serial.println(sensorid);Serial.println();
  //pinMode(donePin,OUTPUT);
      
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  
 //start current sense
  uint32_t currentFrequency;
  ina219.begin();
  Serial.println("initializing ina219 done");
  
  //Start rx
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  rf95.setTxPower(23, false);

  Serial.println("done initializing......\n");
  randomSeed(analogRead(randomPin));
  delay(random(3000,4000));
  blinkled();
  Serial.println("done setup");
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop(void)
{
  Serial.println("begin loop");
  
  imu::Vector<3> grvty = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> mgntr = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.println("done init axl variables");
  
  //Read data from sensors: [1] axl (g's), [2] axl (m's), [3] power, [4] soms
  char axlX[4];assignNull(axlX);
  float gx = grvty.x()/9.8;
  float gy = grvty.y()/9.8;
  float gz = grvty.z()/9.8;

  float mx = mgntr.x()/9.8;
  float my = mgntr.y()/9.8;
  float mz = mgntr.z()/9.8;
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  Serial.println("here");
  //read current
  float busvoltage_V = 0;
  float current_mA = 0;
  float power_mW = 0;
  
  busvoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();

  float somsADC = 0;
  float somsVWC = 0;
  
  somsADC = analogRead(somsPin);
  //Serial.println(somsADC);
  somsVWC = (0.004*somsADC) - 0.4839;
  //Serial.println(somsVWC);
  
  //declare line "packet" variables
  char line1[50] = "lgrid:";//axl gravity
  char line2[50] = "lgrid:";//axl mag
  char line3[57] = "lgrid:";//ina power + soms

  //build/parse the line packets
  buildLineAxl(line1,";AXL:",gx,gy,gz);
  buildLineAxl(line2,";MGR:",mx,my,mz);
  //buildLineOth(line3,busvoltage_V,current_mA,power_mW,1.234);
  buildLineOth(line3,busvoltage_V,current_mA,power_mW,somsVWC);

  //delete me
  //digitalWrite(A0,HIGH);
  //Serial.println("TPL 5110 failed");

  //transmit data
  sendLine(line1,50,1);
  sendLine(line2,50,2);
  sendLine(line3,58,3);
  
  Serial.println("#################################3");
  digitalWrite(donePin,HIGH);
  //exit(0);
  //delay(10000);
}

void sendLine(char* line,int inLen,int blinks){
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t  len = sizeof(buf);
  int sendCounter = 0;
  do{
    randomSeed(analogRead(randomPin));
    int del = random(1000,2000);
    Serial.print("..........................................");
    Serial.println(del);
    for(int i=0;i<blinks;i++)blinkled();
    
    Serial.print("Sending "); Serial.println(line);
    Serial.println("Sending...");
    delay(10);
    rf95.send((uint8_t *)line, inLen);
  
    Serial.println("Waiting for packet to complete..."); 
    delay(10);
    rf95.waitPacketSent();
  
    // Now wait for a reply
    Serial.println("Waiting for reply...");
    if (rf95.waitAvailableTimeout(del))
    { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len))
     {
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);    
      }
      else
      {
        Serial.println("Receive failed");
      }
    }
    else
    {
      Serial.println("No reply, is there a listener around?");
    }

    sendCounter++;
    if(sendCounter > sendTrials-1) break;
  }while(strcmp((char*)buf,"ACK"));
}

void buildLineAxl(char* line, char * type, float vx, float vy, float vz){
  char axlString[4];assignNull(axlString);
  
  //line parsing
  strcat(line,sensorid);
  strcat(line,type);
  
  dtostrf(vx,7,4,axlString);
  strcat(line,axlString);
  strcat(line,",");
  
  dtostrf(vy,7,4,axlString);
  strcat(line,axlString);
  strcat(line,",");
  
  dtostrf(vz,7,4,axlString);
  strcat(line,axlString);
  
  Serial.println(line);
}

void buildLineOth(char* line, float vx, float vy, float vz, float vs){
  char fltString[4];assignNull(fltString);
  
  //line parsing
  strcat(line,sensorid);
  strcat(line,";");
  
  strcat(line,"BTV:");
  dtostrf(vx,6,2,fltString);
  strcat(line,fltString);
  strcat(line,";");

  strcat(line,"BTA:");
  dtostrf(vy,6,2,fltString);
  strcat(line,fltString);
  strcat(line,",");

  strcat(line,"BTP:");
  dtostrf(vz,6,2,fltString);
  strcat(line,fltString);
  strcat(line,",");
  
  strcat(line,"SMS:");
  dtostrf(vs,6,3,fltString);
  strcat(line,fltString);
  
  Serial.println(line);
}

void blinkled(){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
}

void assignNull(char* txt){
  for(int i=0;i<strlen(txt);i++){
    txt[i] = '\0'; 
  }
}
