#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

bool testing = false;
bool slave = true;

// HC05
const byte btrx = 10;
const byte bttx = 9;
const byte HCEN = 5;
const byte HCVCC = 2;

SoftwareSerial btSerial(btrx,bttx);

const byte MotAEn = 3;
const byte MotAPh = 4;
const byte MotBEn = 6;
const byte MotBPh = 7;

const bool invertA = 0;
const bool invertB = 1;

// Signal Strength Parsing
bool lineBufferAvailable = false;
int lastSignalStrength = -300;
String serialLineBuffer = "";
String serialRLineBuffer = "";

// Gyro Stuff
MPU6050 mpu6050(Wire);

// State Machine
char state = 5;

String targetClass = "FFFFF3";

unsigned long maxRSSI = 0;
unsigned long maxGyro = 0;
unsigned long tagThreshold = 65495;

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);  
  
  pinMode(btrx, INPUT);
  pinMode(bttx, OUTPUT);

  pinMode(HCEN, OUTPUT);
  digitalWrite(HCEN, LOW);
  pinMode(HCVCC, OUTPUT);
  digitalWrite(HCVCC, HIGH);

  btSerial.begin(38400);
  Serial.begin(9600);

  // Setup Motor Controller
  pinMode(MotAEn, OUTPUT);
  pinMode(MotAPh, OUTPUT);
  digitalWrite(MotAPh, invertA);
  pinMode(MotBEn, OUTPUT);
  pinMode(MotBPh, OUTPUT);
  digitalWrite(MotBPh, invertB);

  delay(500);
  btSerial.write("AT");
  btSerial.write(0x0d);
  btSerial.write(0x0a);

  delay(500);
  RunMotors(0, 0, 0, 0);
  btSerial.write("AT+INIT");
  btSerial.write(0x0d);
  btSerial.write(0x0a);

  //HCReset(false);

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

}

void loop() {
  mpu6050.update();
  lineBufferAvailable = parseSerial();

  if(lineBufferAvailable){
    Serial.println("\n");
    
    if(serialLineBuffer.length() >= 24){
      // INQ response line found. 
      getRSSI(serialLineBuffer);
    }else{
      // Not INQ response line.
      Serial.println(serialLineBuffer);
      if(slave && serialLineBuffer == "TAG"){
        slave = false;
      }
    }
    
    serialLineBuffer = "";
    serialRLineBuffer = "";
  }

  if(testing){
    return;  
  }
  
  // = = = = = = = = = = = = STATE MACHINE = = = = = = = = = = = =
  
  // ~65495 = point blank RSSI

  switch(state){
    case 0:
      digitalWrite(13, LOW);
      Serial.println("Start State 0");
      // Star Pattern Seeking
      
      mpu6050.update();

      for (int i = 0; i < 330; i += 55) {
        mpu6050.update();
        //Serial.println(mpu6050.getGyroAngleX() - i);
        while (abs(mpu6050.getGyroAngleX() - i) > 10){
          // Turn to next seek angle
          mpu6050.update();
          Serial.println(mpu6050.getGyroAngleX());
          if (abs(mpu6050.getGyroAngleX()) > i) {
            //Serial.println("right turn");
            RunMotors(1,0,50,50);
          } else {
            //Serial.println("left turn");
            RunMotors(0,1,50,50);
          }
        }
        // Desired seek angle reached. Move Foward.
        Serial.println("Seek Angle Reached. Foward.");
        delay(300);
        RunMotors(0,0,130,130);
        nonBlockingDelay(2300);
        RunMotors(0,0,0,0);

        Serial.println("Getting Average...");
        long curRSSI = getAverageRSSI();
        
        if(curRSSI > tagThreshold){
          Tag();
          return;
        }
        if(curRSSI > maxRSSI){ 
          digitalWrite(13, HIGH);         
          Serial.println("New Best Seek!");
          Serial.println(curRSSI);
          Serial.println(maxRSSI);
          maxRSSI = curRSSI;
          maxGyro = mpu6050.getGyroAngleX();
          nonBlockingDelay(800);
          digitalWrite(13, LOW);
        }

        Serial.println("Average Collected. Move Back.");
        RunMotors(1,1,130,130);
        nonBlockingDelay(2200);
        RunMotors(0,0,0,0);
        nonBlockingDelay(500);
      }
      
      RunMotors(0,0,0,0);
      state = 1;
      break;
        
    case 1:
      // Seek complete. Turn to best angle
      Serial.println("Start State 1");

      // Turn to desired angle
      while (abs(mpu6050.getGyroAngleX() - maxGyro) > 3){
        mpu6050.update();
        if (abs(mpu6050.getGyroAngleX()) > maxGyro) {
          //Serial.println("right turn");
          RunMotors(1,0,50,50);
        } else {
          //Serial.println("left turn");
          RunMotors(0,1,50,50);
        }
      }

      for(int i=0; i<1; i++){  
        RunMotors(0,0,120,120);
        nonBlockingDelay(1800);
        RunMotors(0,0,0,0);
  
        unsigned long curRSSI = getAverageRSSI();
        if(curRSSI >= tagThreshold){
          Tag();
          return;
        }
      }
      
      state = 0;
      break;

    case 2:
      //Turn Left.
      RunMotors(0,1,45,45);
      nonBlockingDelay(3000);
      //Turn Right.
      RunMotors(1,0,45,45);
      nonBlockingDelay(3000);
      RunMotors(0,0,0,0);
      break;

    case 5:
      if(!slave){
        delay(100);
        HCReset(1);
        delay(900);
        btSerial.write("AT+ROLE=1\r\n");
        state = 99;
      }else{
        Serial.print('.');
      }
  
  }
}

///////////////////////////////////////////////////////////
// METHODS
///////////////////////////////////////////////////////////

void RunMotors(bool phA, bool phB, byte enA, byte enB){
  if(invertA){
    phA = !phA;
  }
  if(invertB){
    phB = !phB;
  }
  
  digitalWrite(MotAPh, phA);
  digitalWrite(MotBPh, phB);
  analogWrite(MotAEn, enA);
  analogWrite(MotBEn, enB);
}

void HCReset(bool ATEnable){
  digitalWrite(HCVCC, LOW);
  digitalWrite(HCEN, ATEnable);
  delay(50);
  digitalWrite(HCVCC, HIGH);
}

bool parseSerial(){
  if(btSerial.available()){
  
    char nextChar = btSerial.read();
    
    if(nextChar == 0x0d){
      // CR Found. End of Line. Also remove LF
      btSerial.read();
      Serial.println("= = = = = LINE END = = = = =");
      
      return true;
    }else{
      serialLineBuffer = serialLineBuffer + nextChar;
    }
  }

  if (Serial.available()){
    delay(5);
    while(Serial.available()){
      // In Line Mode. Read everything from buffer.
      char nextChar = Serial.read();
      serialRLineBuffer = serialRLineBuffer + nextChar;
    }

    //if(nextChar = 0x0a){
    if(true){

      if(isST(serialRLineBuffer)){
        // ST Command found. Execute only.
        Serial.println("Found ST Command.");
        STCommand(serialRLineBuffer);
        
      }else{
        // Not ST Command. Write to HC.
        for(char c : serialRLineBuffer){
          btSerial.write(c);
        }
      }
      // Clear Line Buffer
      serialRLineBuffer = "";
    }
  }
  
  return false; 
}

long getRSSI(String inString){
  Serial.println("RSSI Response Found");
  Serial.println(inString);

  char RSSIString[4] = "0000";

  int curInd = 0;
  
  for(int i = inString.length()-4; i < inString.length(); i++){
    RSSIString[curInd] = inString[i];
    curInd++;
  }
  
  Serial.println(RSSIString);
  long signalStrength = strtol(RSSIString, NULL, 16);
  Serial.println("V Signed Signal Strength V");
  Serial.println(signalStrength);

  return signalStrength;
}

bool isST(String inString){
  if(inString.length() < 4){
    return false;
  }
  
  String fT;
  fT += inString[0];
  fT += inString[1];
  fT += inString[2];

  if(fT == "ST+"){
    return true;
  }
  return false;
}

void STCommand(String inString){
  // COMMAND: ST+XXN
  // ST+BT | Bluetooth Module Commands
  //         BT0 > Restart into COMM mode.
  //         BT1 > Restart into AT mode
  // ST+GY | Gyro Module Commands
  //         GY0 > Calibrate Gyro
  //         GY1 > Update Gyro
  //         
  
  String cmd = "";
  for(int i=0; i<2; i++){
    cmd += inString[i+3];
  }

  Serial.println("Received ST Command: " + cmd);
  
  if(cmd == "BT"){
    char cmdVal = inString[5];
    switch(cmdVal){
      case '0':
        HCReset(0);
        Serial.println("ST > Restarting HC in COMM...");
        break;
      case '1':
        HCReset(1);
        Serial.println("ST > Restarting HC in AT...");
        break;
      case '2':
        Serial.println("ST > Now Getting Average RSSI to Target Class");
        Serial.println(getAverageRSSI());
        Serial.println("||||||^ AVERAGE ^||||||");
        break;
    }
  }
  else if(cmd == "GY"){
    char cmdVal = inString[5];
    switch(cmdVal){
      case '0':
        mpu6050.begin();
        mpu6050.calcGyroOffsets(true);
        break;
      case '1':
        Serial.println("ST > Get Gyro Angles");
        Serial.print("gyroX : ");
        Serial.print(mpu6050.getGyroX());
        Serial.print("\tgyroY : ");
        Serial.print(mpu6050.getGyroY());
        Serial.print("\tgyroZ : ");
        Serial.println(mpu6050.getGyroZ());
        break;
      case '2':
        Serial.println("ST > Get Angles");
        Serial.print("gyroX : ");
        Serial.print(mpu6050.getAngleX());
        Serial.print("\tgyroY : ");
        Serial.print(mpu6050.getAngleY());
        Serial.print("\tgyroZ : ");
        Serial.println(mpu6050.getAngleZ());
        break;
    }
  }
  serialRLineBuffer = "";
}

void nonBlockingDelay(int toWait){
  long long int startTime = millis();
  while(millis() - startTime < toWait){
    mpu6050.update();
  }
}

long getAverageRSSI(){
  // Get 8 point average RSSI from BT device with target class.
  int sPoints = 9;
  
  String currentClass;
  int curInd = 0;
  long signalStr = 0;
  int gotValues = 0;

  bool done = false;
  
  // Setup INQM and begin INQ.
  btSerial.write("AT+INIT\r\n");
  delay(300);
  while(!parseSerial()){}
  serialLineBuffer = "";
  btSerial.write("AT+INQM=1,2,3\r\n");
  delay(300);
  while(!parseSerial()){}
  serialLineBuffer = "";
  btSerial.write("AT+INQM=1,50,20\r\n");
  delay(300);
  while(!parseSerial()){}
  serialLineBuffer = "";
  btSerial.write("AT+INQ\r\n");
  
  // Flush Line Buffer
  serialLineBuffer = "";
  serialRLineBuffer = "";
  Serial.flush();
  btSerial.flush();
  delay(800);
  

  // Start Capturing Values from INQ reults.
  while(!done){
    lineBufferAvailable = parseSerial();
    currentClass = "";
  
    if(lineBufferAvailable){
      // Got a full line from serial buffer.
      Serial.println("= = Serial Line = =");
      
      if(serialLineBuffer.length() >= 24){
        // INQ response line found. 
        // Get CLASS value from current INQ message.
        
        for(int i=serialLineBuffer.length()-11; i < serialLineBuffer.length() - 5; i++){
          currentClass = currentClass + serialLineBuffer[i];
          curInd++;
        }
      
        if(currentClass == targetClass){
          Serial.println(":) Class Matching RSSI!");
          signalStr += getRSSI(serialLineBuffer);
          Serial.println(signalStr);
          gotValues += 1;
        }else{
          Serial.println("/!\\ Non-Matching RSSI.");
          Serial.println(currentClass);
        }
      }else{
        // Not INQ response line.
        char fT[2];
        fT[0] = serialLineBuffer[0];
        fT[1] = serialLineBuffer[1];
        /*
        if(fT == "OK"){
          btSerial.write("AT+INQ\r\n");
        }*/
      }
      
      if(gotValues >= sPoints){
        Serial.write("AT+INQC\r\n");
        delay(500);
        while(!parseSerial()){}
        serialLineBuffer = "";
        btSerial.write("AT+INQC\r\n");
        done = true;
        Serial.flush();
        btSerial.flush();
        return signalStr / gotValues;
      }
    }
  }
  return 0;
}

void Tag(){
  RunMotors(0,0,0,0);
  digitalWrite(13, HIGH);
  state = 99;
  btSerial.write("AT+RMAAD\r\n");
  delay(1200);
  btSerial.write("AT+PSWD=1234\r\n");
  delay(1200);
  btSerial.write("AT+INIT\r\n");
  delay(1200);
  btSerial.write(rnameString);
  delay(1200);
  btSerial.write(pairString);
  delay(1200);
  btSerial.write(linkString);
  while(true){
    digitalWrite(13, HIGH);
    delay(333);
    digitalWrite(13, LOW);
    delay(333);
  }
  return;
}