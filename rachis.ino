#include <avr/dtostrf.h>

//define sensor name/s
#define SENSEID "04"
#define AREA "BCM"
#define SITE "PDE"
#define terminator "$"

//define pin assignments
#define donePin A0	//done pin for tpl
#define randomPin A1	//pin for generating random seed for random delay when sending
#define somsPin A3	//SOMS input
#define sendTrials 5	//number of send retries before exit

//lib required #includes
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Wire.h>
#include <Adafruit_INA219.h>

#include <SPI.h>
#include <RH_RF95.h>


//lib required #define and global variable declarations
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 433.0

Adafruit_BNO055 bno = Adafruit_BNO055();

Adafruit_INA219 ina219;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup(void)
{
  Serial.begin(9600);
  delay(2000);//put 2 sec delay to allow serial monitor to open
  Serial.println(SENSEID);Serial.println();
  
  pinMode(donePin,OUTPUT);
      
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  //delay 1000 before setting axl settings
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  
  //initialize ina219
  uint32_t currentFrequency;
  ina219.begin();
  Serial.println("initializing ina219 done");
  
  //initialize rf module
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

  //do not continue if rf is not okay
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

  //randomSeed(analogRead(randomPin));
  blinkled();
  Serial.println("done setup");
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop(void)
{
  Serial.println("begin loop");
  
  //Read data from axl
  imu::Vector<3> grvty = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> mgntr = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.println("done initializing axl variables");
  
  char axlX[4];assignNull(axlX);
  float gx = grvty.x()/9.8;
  float gy = grvty.y()/9.8;
  float gz = grvty.z()/9.8;

  float mx = mgntr.x()/9.8;
  float my = mgntr.y()/9.8;
  float mz = mgntr.z()/9.8;
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  //read power
  float busvoltage_V = 0;
  float current_mA = 0;
  float power_mW = 0;
  
  busvoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();

  //read soms
  float somsADC = 0;
  float somsVWC = 0;
  
  somsADC = analogRead(somsPin);
  somsVWC = (0.004*somsADC) - 0.4839;//calibration equation from reference paper
  
  //declare message variables
  char line1[72] = AREA;//axl mag/
  char line2[49] = AREA;//power
  char line3[27] = AREA;//soms

  //build/parse the line packets
  buildLine_01(line1,gx,gy,gz,mx,my,mz);
  buildLine_02(line2,busvoltage_V,current_mA,power_mW);
  buildLine_03(line3,somsVWC);
  
  //transmit data
  sendLine(line1,71,1);
  sendLine(line2,48,2);
  sendLine(line3,26,3);
  
  Serial.println("#################################3");
  digitalWrite(donePin,HIGH);
  //exit(0);
  //delay(10000);
}

//send message using rf. inLen variable is the number of characters to be sent. blinks variable determines the number of times led will blink before sending. used primarily for determining which line is being sent
void sendLine(char* line,int inLen,int blinks){
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t  len = sizeof(buf);
  int sendCounter = 0;
  do{
    randomSeed(analogRead(randomPin));
    int del = random(1,30) * 100;//random delay between 100 - 3000 ms
    Serial.print("..........................................");
    Serial.println(del);
    for(int i=0;i<blinks;i++)blinkled();//blink 
    
	//send data
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

	//check if # of retries is still valid or until an ACK is rcvd
    sendCounter++;
    if(sendCounter > sendTrials-1) break;
  }while(strcmp((char*)buf,"ACK"));
}

//parses the id part of the message to be sent
void buildID(char* line,char* sensor){
  strcat(line,"-");
  strcat(line,SITE);
  strcat(line,"-");
  strcat(line,sensor);
  strcat(line,SENSEID);
  strcat(line,terminator);
}

//parses the data part of the message to be sent
void buildLine_01(char* line, float vx, float vy, float vz,
  float wx, float wy, float wz){
  
  char sensorType[4] =  "TLT";
  char tmpString[7];assignNull(tmpString);
  
  buildID(line,sensorType);
  
  //parse axl data here
  strcat(line,"AXL:");
  
  dtostrf(vx,7,4,tmpString);
  strcat(line,tmpString);
  strcat(line,",");
  
  dtostrf(vy,7,4,tmpString);
  strcat(line,tmpString);
  strcat(line,",");
  
  dtostrf(vz,7,4,tmpString);
  strcat(line,tmpString);
  strcat(line,";");

  //parse magnetometer data here
  strcat(line,"MGR:");
  
  dtostrf(wx,7,4,tmpString);
  strcat(line,tmpString);
  strcat(line,",");

  dtostrf(wy,7,4,tmpString);
  strcat(line,tmpString);
  strcat(line,",");
  
  dtostrf(wz,7,4,tmpString);
  strcat(line,tmpString);
  
  //concatenate terminating character
  strcat(line,"$");
  Serial.println(line);
}

//parses the data part of the message to be sent
void buildLine_02(char* line, float vx, float vy, float vz){
  char sensorType[4] = "SLR";
  char tmpString[6];assignNull(tmpString);
  
  buildID(line,sensorType);
  
  //parse ina219 data here
  strcat(line,"BTV:");
  dtostrf(vx,6,2,tmpString);
  strcat(line,tmpString);
  strcat(line,";");

  strcat(line,"BTA:");
  dtostrf(vy,6,2,tmpString);
  strcat(line,tmpString);
  strcat(line,";");

  strcat(line,"BTP:");
  dtostrf(vz,6,2,tmpString);
  strcat(line,tmpString);
  
  //concatenate terminating character
  strcat(line,"$");
  Serial.println(line);
}//end of fxn

//parses the data part of the message to be sent
void buildLine_03(char* line, float vs){
  char sensorType[4] = "SMS";
  char tmpString[6];assignNull(tmpString);
  
  buildID(line,sensorType);
  strcat(line,"VWC:");
  dtostrf(vs,6,3,tmpString);
  strcat(line,tmpString);
  
  //concatenate terminating character
  strcat(line,"$");
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
