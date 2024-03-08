#include <LiquidCrystal.h>
#include <LowPower.h>

//Hardware Pinouts
const int RS=7, E1=8, E2=9, LCD4=10, LCD5=11, LCD6=12, LCD7=13; //LCDpinout, 40x4
const int BTN=2, ECUREF=3, BL=5, REDLED=6; //OtherHardware. 2/3 is interupt capable, 5/6 allows for 960hz PWM

//Library Setup
LiquidCrystal lcdTop (RS, E1, LCD4, LCD5, LCD6, LCD7);      //Top half of the 40x4 LCD
LiquidCrystal lcdBottom (RS, E2, LCD4, LCD5, LCD6, LCD7);   //Bottom half of the 40x4

//Variables
volatile bool buttonPressed = false;  //interupt sets this to true whenever button is pressed
volatile bool ecuState = false;       //This bool is used for determining if the ECU is on or not. Set by interrupt on a pin
byte mode1RX[64];                     //Mode 1 response array (starts at 1)
bool mode1CheckSumError = false;      //Error flag for mode1 checksum failure
bool mode1WriteError = false;         //Error flag for mode1 write failure
bool mode1ReadError = false;          //Error flag for mode1 read failure
byte checkSumTotal = 0;               //Checksum summing byte
byte screenNum = 1;                   //Start up screen page 
unsigned long timeStart = millis();   //millis()reference for timout calculations

//Mode1 Variables
word RPM=0;                 //RPM                   RPM, only has a resolution of 8bit so it's rounded to nearest 25 RPM window
byte TPS=0;                 //Raw TPS               VOLTS = N*5/256
byte BATT=0;                //Raw Battery Voltage   VOLTS = N*0.1
byte MAP=0;                 //Raw MAP               kpa = (N + 28.06)/2.71
byte IAT=0;                 //Raw Intake Air Temp   DEGREES C = .75N - 40   DEGREES F = 1.35N - 40
byte CLT=0;                 //Raw Coolant Temp      DEGREES C = .75N - 40   DEGREES F = 1.35N - 40
byte O2=0;                  //O2                    mV = 4.42N
byte MPH=0;                 //MPH
byte EGR=0;                 //EGR Duty Cycle        % = N/2.56
byte BARO=0;                //ECU Baro Calculation  Calculated from MAP during full throttle and before starting
word BPW=0;                 //Base Pulse Width
word RPMref = 0;            //Cycles since last TDC reference pulse   MSEC = N/65.536   RPM = 65.536* # OF CYLINDER/N   //6 CYL = 20  8 CYL = 15
int SAREF=0;                //Spark Angle Reference(?)                DEGREES = (N*90/256) + 66 (double check this math)
byte BLMcell=0;             //Block Learn Multiplier Cell
byte BLM=0;                 //Block Learn Multiplier    (Long Term Fuel Trim)
byte INT=0;                 //Integrator                (Short Term Fuel Trim)

//Mode1 Mathed decodes
byte TPSp=0;                //TPS scaled to 100%
byte MAPkpa=0;              //MAP mathed to kpa
int IATc=0;                 //IAT mathed to C
int IATf=0;                 //IAT mathed to F
int CLTc=0;                 //CLT mathed to C
int CLTf=0;                 //CLT mathed to F
word O2mv=0;                 //O2 mathed to volts
float BPWms=0;              //Base Pulse Width mathed to mS
float SAREFd=0;             //Spark Angle mathed to Degree
char BLMp=0;                //Block Learn Multiplier mathed to +/- %
char INTp=0;                //Integrator mathed to +/- %
bool CELerror = false;      //An error code was set

//Mode1 Booleans
bool O2rdy = false;         //O2 Sensor Ready
bool DFCO = false;          //Decel Fuel Cutoff Active
bool RICH = false;          //Rich/Lean flag (Rich = True)
bool DE = false;            //Decel Enleanment
bool PE = false;            //Power Enrichment
bool AE = false;            //Accel Enrichment
bool CLOOP = false;         //Closed loop flag (Closed = True)
bool LEARN = false;         //Block Learning Active
bool PNsw = false;          //Neutral Switch (P/N = False)
bool Brake = false;         //Brake Switch (Pressed = True)
bool TCC = false;           //Torque Converter Lockup Commanded
bool CODE13 = false;        //CEL Code 13
bool CODE14 = false;        //CEL Code 14
bool CODE15 = false;        //CEL Code 15   
bool CODE16 = false;        //CEL Code 16
bool CODE21 = false;        //CEL Code 21
bool CODE22 = false;        //CEL Code 22
bool CODE23 = false;        //CEL Code 23
bool CODE25 = false;        //CEL Code 25
bool CODE32 = false;        //CEL Code 32
bool CODE33 = false;        //CEL Code 33
bool CODE34 = false;        //CEL Code 34
bool CODE35 = false;        //CEL Code 35 (Not set in ROM but mentioned as valid in service manual)
bool CODE41 = false;        //CEL Code 41
bool CODE42 = false;        //CEL Code 42
bool CODE44 = false;        //CEL Code 44
bool CODE45 = false;        //CEL Code 45
bool CODE51 = false;        //CEL Code 51
bool CODE53 = false;        //CEL Code 53 (Not set in ROM but mentioned as valid in service manual)
bool CODE54 = false;        //CEL Code 54
bool CODE55 = false;        //CEL Code 55 (Not set in ROM but mentioned as valid in service manual)

//Custom LCD Characters
byte Seg1[8] = {0B10000, 0B10000, 0B10000, 0B10000, 0B10000, 0B10000, 0B10000, 0B10000};  //Progress bar segment 1
byte Seg2[8] = {0B11000, 0B11000, 0B11000, 0B11000, 0B11000, 0B11000, 0B11000, 0B11000};  //Progress bar segment 2
byte Seg3[8] = {0B11100, 0B11100, 0B11100, 0B11100, 0B11100, 0B11100, 0B11100, 0B11100};  //Progress bar segment 3
byte Seg4[8] = {0B11110, 0B11110, 0B11110, 0B11110, 0B11110, 0B11110, 0B11110, 0B11110};  //Progress bar segment 4
byte Seg5[8] = {0B11111, 0B11111, 0B11111, 0B11111, 0B11111, 0B11111, 0B11111, 0B11111};  //Progress bar segment 5

void setup() {
  pinMode(BTN, INPUT_PULLUP);         //User Button with internal Pullup
  pinMode(ECUREF, INPUT);             //5v ref from ECU to determine power state
  pinMode(REDLED, OUTPUT);            //Red LED next to button
  pinMode(BL, OUTPUT);                //Backlight, PWM Capable
  attachInterrupt (digitalPinToInterrupt (BTN), buttonINT, FALLING);      //User button. Sets only if its been pressed. Code has to unset
  attachInterrupt (digitalPinToInterrupt (ECUREF), ecuStateINT, CHANGE);  //Used to set ecuState based on 5v interrupt from ECU
  ecuStateINT();                      //set inital state for ECU 
  
  ecuState = true;            //debug, sets inital state on for testing
  
  Serial.begin(8192);                 //GM ALDL baud rate
  Serial.setTimeout(250);             //ECU timeout to respond to request (about twice as long as it'd ever need)
  lcdTop.createChar(1, Seg1); lcdBottom.createChar(1, Seg1);            //Progress bar segment 1
  lcdTop.createChar(2, Seg2); lcdBottom.createChar(2, Seg2);            //Progress bar segment 2
  lcdTop.createChar(3, Seg3); lcdBottom.createChar(3, Seg3);            //Progress bar segment 3
  lcdTop.createChar(4, Seg4); lcdBottom.createChar(4, Seg4);            //Progress bar segment 4
  lcdTop.createChar(5, Seg5); lcdBottom.createChar(5, Seg5);            //Progress bar segment 5
  lcdTop.begin(40, 2); lcdBottom.begin(40, 2);                          //LCD start
  lcdTop.clear(); lcdBottom.clear();                                    //Clear any potential junk on screen
  digitalWrite(BL, HIGH);             //Enable Backlight Full Bright    //TODO: Add dimming via button
}

void loop() {
  while(ecuState) { //while ECU on
    Display();
  }
  while(!ecuState) {//while ECU on't
    lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(0,1); lcdBottom.print("ECU Power off");
    for(timeStart = millis(); millis() - timeStart < 2500;) {
      if( ecuState ) { return; }
    } 
    if(!ecuState) { Off(); }
  } 
}

void Display() {
  if(buttonPressed) {screenNum++; buttonPressed = false;}   //enable this feature later
  if(screenNum == 0) Off(); //Screen zero stay off
  else if(screenNum == 1) {      //Datastream debug screen 
    mode1();
    if(mode1WriteError) {lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(23,1); lcdBottom.print("ECU write timeout"); return;}    
    if(mode1ReadError) {lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(24,1); lcdBottom.print("ECU read timeout"); return;}    

    lcdTop.setCursor(0,0); lcdBottom.setCursor(0,0);
    for(byte i=0; i<= 64; i++) {     //dump all bytes onto LCD (for debug)
      if(i<40) {
        if(mode1RX[i]<10) lcdTop.print("0");
        lcdTop.print(mode1RX[i], HEX); 
      }
      if(i>=40) {
        if(mode1RX[i]<10) lcdBottom.print("0");
        lcdBottom.print(mode1RX[i], HEX); 
      }
    }
    lcdBottom.print("      Calculated Checksum 0x"); if (checkSumTotal < 10) lcdBottom.print("0"); lcdBottom.print(checkSumTotal, HEX);
  }
  else if(screenNum == 2) {      //Everything Useful, All At Once
    mode1();
    if(mode1WriteError) {lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(23,1); lcdBottom.print("ECU write timeout"); return;}    
    if(mode1ReadError) {lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(24,1); lcdBottom.print("ECU read timeout"); return;} 
    //if(mode1CheckSumError) {lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(15,1); lcdBottom.print("ECU Checksum fail 0x"); if (checkSumTotal < 10) lcdBottom.print("0"); lcdBottom.print(checkSumTotal, HEX); return;} 
    mode1Decode();
    if(CELerror) digitalWrite(REDLED, HIGH); else digitalWrite(REDLED, LOW);

    CLTf = CLT *1.35-40;  //CLTc = CLT *0.75-40;
    IATf = IAT *1.35-40;  //IATc = IAT *0.75-40;
    MAPkpa = (MAP + 28.06)/2.71;
    TPSp = TPS/2.56;      //Scale to 100%
    INTp = (INT *0.78125)-100;
    BLMp = (BLM *0.78125)-100;
    O2mv = O2 * 4.42;     //Convert to Millivolts
    BPWms = BPW/65.536;   //Convert to Milliseconds
    SAREFd = (SAREF*.351)+66;
    
    lcdTop.setCursor(0,0);
    lcdTop.print("RPM:"); if(RPM < 10) lcdTop.print("   "); else if(RPM < 100) lcdTop.print("  "); else if(RPM < 1000) lcdTop.print(" "); lcdTop.print(RPM); lcdTop.print(" "); progressBarTop(10, RPM, 6000);
    lcdTop.print("MAP:"); if(MAPkpa < 100) lcdTop.print(" "); lcdTop.print(MAPkpa); lcdTop.print(" "); progressBarTop(10, MAPkpa, 100); lcdTop.print("kPa");
    
    lcdTop.setCursor(0,1);
    lcdTop.print("CLT:"); if(CLTf < 10 && CLTf >= 0) lcdTop.print("  "); else if(CLTf > -10 && CLTf <= 0 || CLTf >= 10 && CLTf < 100) lcdTop.print(" "); lcdTop.print(CLTf); lcdTop.print("F "); 
    lcdTop.print("IAT:"); if(IATf < 10 && IATf >= 0) lcdTop.print("  "); else if(IATf > -10 && IATf <= 0 || IATf >= 10 && IATf < 100) lcdTop.print(" "); lcdTop.print(IATf); lcdTop.print("F "); 
    lcdTop.print("TPS:"); if(TPSp < 10) lcdTop.print(" "); if(IATf < 100) lcdTop.print(" "); lcdTop.print(TPSp); lcdTop.print("% "); 
    if(MPH < 10) lcdTop.print("  "); else if(IATf < 100) lcdTop.print(" "); lcdTop.print(MPH); lcdTop.print("MPH  ");
    if(BATT < 100) lcdTop.print(" "); lcdTop.print(byte(BATT*.1)); lcdTop.print("."); lcdTop.print(BATT % 10,1); lcdTop.print("v");
    
    lcdBottom.setCursor(0,0);
    lcdBottom.print("INT:"); if(INTp >= 0 && INTp < 10) lcdBottom.print("  "); else if(INTp > -10 && INTp < 99) lcdBottom.print(" "); lcdBottom.print(INTp, DEC); lcdBottom.print("% "); 
    lcdBottom.print("BLM:"); if(BLMp >= 0 && BLMp < 10) lcdBottom.print("  "); else if(BLMp > -10 && BLMp < 99) lcdBottom.print(" "); lcdBottom.print(BLMp, DEC); lcdBottom.print("% "); 
    lcdBottom.print("O2:"); if(O2mv < 10) lcdBottom.print("   "); else if(O2mv < 100) lcdBottom.print("  "); else if(O2mv < 1000) lcdBottom.print(" "); lcdBottom.print(O2mv); lcdBottom.print("mv ");
    lcdBottom.setCursor(29,0);
    if(!PNsw) lcdBottom.print("P/N "); if(!CLOOP) lcdBottom.print("OL "); if(EGR > 0) lcdBottom.print("EGR "); if(TCC) lcdBottom.print("TCC "); lcdBottom.print("    ");//##move to bottom left
    
    lcdBottom.setCursor(0,1);  //##move up and right
    lcdBottom.print("PW:"); if (BPWms < 10) lcdBottom.print(" "); lcdBottom.print(BPWms,1); ; lcdBottom.print("mS ");
    lcdBottom.print("IGN:"); if (SAREFd < 10 && SAREFd > -10) lcdBottom.print(" "); lcdBottom.print(SAREFd, 1); lcdBottom.print((char)223); lcdBottom.print(" ");
    lcdBottom.setCursor(19,1);
    if(CELerror) {lcdBottom.print("ERROR"); if(CODE13) lcdBottom.print(" 13"); if(CODE14) lcdBottom.print(" 14"); if(CODE15) lcdBottom.print(" 15"); if(CODE16) lcdBottom.print(" 16"); if(CODE21) lcdBottom.print(" 21"); if(CODE22) lcdBottom.print(" 22"); if(CODE23) lcdBottom.print(" 23"); if(CODE25) lcdBottom.print(" 25"); if(CODE32) lcdBottom.print(" 32"); if(CODE33) lcdBottom.print(" 33"); if(CODE34) lcdBottom.print(" 34"); if(CODE35) lcdBottom.print(" 35"); if(CODE41) lcdBottom.print(" 41"); if(CODE42) lcdBottom.print(" 42"); if(CODE44) lcdBottom.print(" 44"); if(CODE45) lcdBottom.print(" 45"); if(CODE51) lcdBottom.print(" 51"); if(CODE53) lcdBottom.print(" 53"); if(CODE54) lcdBottom.print(" 54"); if(CODE55) lcdBottom.print(" 55");}
    else lcdBottom.print("                    ");
  }
  else if(screenNum == 3) {      //RPM, MPH, TPS, MAP
    mode1();
    if(mode1WriteError) {lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(23,1); lcdBottom.print("ECU write timeout"); return;}    
    if(mode1ReadError) {lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(24,1); lcdBottom.print("ECU read timeout"); return;} 
    //if(mode1CheckSumError) {lcdTop.clear(); lcdBottom.clear(); lcdBottom.setCursor(15,1); if (checkSumTotal < 10) lcdBottom.print("0"); lcdBottom.print("ECU checksum failure 0x"); lcdBottom.print(checkSumTotal, HEX); return;} 
    mode1Decode();
    if(CELerror) digitalWrite(REDLED, HIGH); else digitalWrite(REDLED, LOW);
     
    lcdTop.setCursor(0,0);
    progressBarTop(32, RPM, 6000);
    if(RPM < 10) lcdTop.print("   "); else if(RPM < 100) lcdTop.print("  "); else if(RPM < 1000) lcdTop.print(" ");   //padding
    lcdTop.print(RPM); lcdTop.print(" RPM");
     
    lcdTop.setCursor(0,1);
    progressBarTop(33, MPH, 100);
    if(MPH < 100) lcdTop.print(" "); if(MPH < 10) lcdTop.print(" ");   //padding
    lcdTop.print(MPH); lcdTop.print(" MPH");

    lcdBottom.setCursor(0,0);
    progressBarBottom(33, TPS, 255);
    if(TPS < 100) lcdBottom.print(" "); if(TPS < 10) lcdBottom.print(" ");   //padding
    lcdBottom.print(TPS); lcdBottom.print(" TPS");
      
    lcdBottom.setCursor(0,1);
    progressBarBottom(33, MAP, 255);
    if(MAP < 100) lcdBottom.print(" "); if(MAP < 10) lcdBottom.print(" ");   //padding
    lcdBottom.print(MAP); lcdBottom.print(" MAP");
  }
  else {screenNum = 0;}    //Loop to off screen when out of screens
}

void mode1() {        //Request the ECU datastream
  mode1WriteError = true;  mode1ReadError = true; mode1CheckSumError = true;
  while (Serial.available() > 0) { Serial.read(); }       //clear serial buffer?
  Serial.write(0xF4);Serial.write(0x57);Serial.write(0x01);Serial.write(0x00);Serial.write(0xB4);   //Mode 1 Request

  for (timeStart = millis(); millis()-timeStart < 250;) {
    if (Serial.read() == 0xF4) {
      timeStart = millis();
      while(millis()-timeStart < 250 && Serial.available() == 0);
      if (Serial.peek() == 0x57) { mode1WriteError = false; }
      if (Serial.read() == 0x95) { timeStart = millis(); Serial.readBytes(mode1RX, 65); timeStart -= 250; mode1ReadError = false;}
    }
  }
  checkSumTotal = 0xF4+0x95; 
  for(byte i=0; i <=64; i++) {checkSumTotal += mode1RX[i];}    //calculate checksum from retrieved bytes
  if(checkSumTotal == 0xFF) mode1CheckSumError = false;   //set error to false
}

void mode1Decode() {  //Take that data and stuff it into various variables
  CLT = mode1RX[15];
  BATT = mode1RX[16];
  TPS = mode1RX[17];
  MAP = mode1RX[18];
  O2 = mode1RX[19];
  BARO = mode1RX[27];
  MPH = mode1RX[31];
  RPM = mode1RX[34]*25;
  RPMref = (mode1RX[35] << 8) | mode1RX[36];
  EGR = mode1RX[37];
  SAREF = (mode1RX[45] << 8) | mode1RX[46];
  INT = mode1RX[49];
  BLMcell = mode1RX[54];
  BLM = mode1RX[55];
  BPW = (mode1RX[57] << 8) | mode1RX[58];
  IAT = mode1RX[63];
  
  O2rdy = bitRead(mode1RX[3], 0);
  CODE13 = bitRead(mode1RX[6], 7);
  CODE14 = bitRead(mode1RX[6], 6);
  CODE15 = bitRead(mode1RX[6], 5);
  CODE16 = bitRead(mode1RX[6], 4);
  CODE21 = bitRead(mode1RX[6], 0);
  CODE22 = bitRead(mode1RX[7], 7);
  CODE23 = bitRead(mode1RX[7], 6);
  CODE25 = bitRead(mode1RX[7], 4);
  CODE32 = bitRead(mode1RX[8], 6);
  CODE33 = bitRead(mode1RX[8], 5);
  CODE34 = bitRead(mode1RX[8], 4);
  CODE35 = bitRead(mode1RX[8], 3);
  CODE41 = bitRead(mode1RX[9], 6);
  CODE42 = bitRead(mode1RX[9], 5);
  CODE44 = bitRead(mode1RX[9], 3);
  CODE45 = bitRead(mode1RX[9], 2);
  CODE51 = bitRead(mode1RX[10], 5);
  CODE53 = bitRead(mode1RX[10], 3);
  CODE54 = bitRead(mode1RX[10], 2);
  CODE55 = bitRead(mode1RX[10], 1);
  if (mode1RX[6], mode1RX[7], mode1RX[8], mode1RX[9], mode1RX[10]) CELerror = true; else CELerror = false;
  DE = bitRead(mode1RX[20], 4);
  PE = bitRead(mode1RX[20], 5);
  AE = bitRead(mode1RX[20], 6);
  Brake = bitRead(mode1RX[25], 1);
  PNsw = bitRead(mode1RX[25], 4);
  DFCO = bitRead(mode1RX[26], 3);
  LEARN = bitRead(mode1RX[29], 1);
  RICH = bitRead(mode1RX[29], 6);
  CLOOP = bitRead(mode1RX[29], 7);
  TCC = bitRead(mode1RX[30], 5);
}

void mode10() {       //this isnt even remotely complete
  Serial.write(0xF4);Serial.write(0x56);Serial.write(0x0A);Serial.write(0xAC);  //Mode 10 Clear Request
  Serial.read();Serial.read();Serial.read();Serial.read();Serial.read();        //Clear buffer due to shared serial line
  //Serial.flush();   //Complete outgoing write before continuing. Clearing buffer needed due to shared serial line?
  //check incoming buffer to confirm clear?
}

void Off() {          //TODO: add micro specific sleep code
  lcdTop.clear(); lcdBottom.clear(); lcdTop.noDisplay(); lcdBottom.noDisplay(); digitalWrite(BL, LOW);    //Goodnight sweet prince
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);                                                    //No timeout, ADC/BOC off
  lcdTop.display(); lcdBottom.display(); digitalWrite(BL, HIGH);                                          //02. Bring Me To Life.mp3
}

void buttonINT() {    //Did ya press it?
  buttonPressed = true;
}
  
void ecuStateINT() {  //Is the ECU on?
  if (digitalRead(ECUREF) == HIGH) ecuState = true;
  else ecuState = false;
}

void progressBarTop(byte len, int value, int top) {  //Length, Data, Max Value for scaling purposes
  value = (len * 5.0000000 / top) * value;
  while(len != 0) {
    if(value > 4){lcdTop.write((byte)5); len--; value -= 5;}
    else if(value == 4){lcdTop.write((byte)4); len--; value -= 4;}
    else if(value == 3){lcdTop.write((byte)3); len--; value -= 3;}
    else if(value == 2){lcdTop.write((byte)2); len--; value -= 2;}
    else if(value == 1){lcdTop.write((byte)1); len--; value -= 1;}
    else if(value == 0){lcdTop.print(" "); len--;};
  }
}
  
void progressBarBottom(byte len, int value, int top) {  //Length, Data, Max Value for scaling purposes
  value = (len * 5.0000000 / top) * value;
  while(len != 0) {
    if(value > 4){lcdBottom.write((byte)5); len--; value -= 5;}
    else if(value == 4){lcdBottom.write((byte)4); len--; value -= 4;}
    else if(value == 3){lcdBottom.write((byte)3); len--; value -= 3;}
    else if(value == 2){lcdBottom.write((byte)2); len--; value -= 2;}
    else if(value == 1){lcdBottom.write((byte)1); len--; value -= 1;}
    else if(value == 0){lcdBottom.print(" "); len--;};
  }
} 
/*   Information taken and modified from the A265.ds datastream specs, which is very similar to the LLV's bitstream

        MODE 0 (RETURN TO NORMAL MODE)
                ALDL REQUEST:
                - MESSAGE ID     = $F4
                - MESSAGE LENGTH = $56
                - MODE           = $00
                - SUM CHECK

                THE PCM WILL RESPOND WITH THE FOLLOWING MESSAGE:
                - MESSAGE ID     = $F4
                - MESSAGE LENGTH = $56
                - MODE           = $00
                - SUM CHECK

        MODE 1 (TRANSMIT FIXED DATA STREAM)
                ALDL REQUEST:
                - MESSAGE ID     = $F4
                - MESSAGE LENGTH = $57
                - MODE           = $01
                - MESSAGE        = $00
                - SUM CHECK

                THE PCM WILL RESPOND WITH THE FOLLOWING MESSAGE:
                - MESSAGE ID     = $F4
                - MESSAGE LENGTH = $95
                - MODE           = $01
                - DATA BYTE 1
                  .
                  .
                - DATA BYTE 63
                - SUM CHECK

        MODE 10 (CLEAR MALFUNCTION CODES)
                ALDL REQUEST:
                - MESSAGE ID     = $F4
                - MESSAGE LENGTH = $56
                - MODE           = $0A
                - CHECKSUM


                THE PCM WILL RESPOND WITH THE FOLLOWING MESSAGE:
                - MESSAGE ID     = $F4
                - MESSAGE LENGTH = $56
                - MODE           = $0A
                - SUM CHECK

WORD #   DATA NAME    DESCRIPTION

* 1      PROMIDA      FIRST PROM I.D. WORD (MSB)
* 2      PROMIDA+1    SECOND PROM I.D. WORD (LSB)      ID = N
  3      NVMW         NON-VOLITALE MODE WORD
          0            02 SENSOR READY FLAG            1 = READY
          1            CLOSED LOOP TIMER O.K. FLAG     1 = TIMER O.K.
          2            1 = RAM REFRESH ERROR HAS OCCURRED
          3            IMPROPER SHUTDOWN FLAG          1 = IMPRO  0 = PROPER
          4            HOT RESTART PROCEEDING FLAG
          5            NOT USED
          6            NOT USED
          7            MALF 42 FAIL FLAG (EST. MONITOR)
  4      DIACMW3      MODE WORD FOR IDLE CONTROL
          0            NOT USED
          1            1 = DRIVE  0 = P/N
          2            1 = CONDITIONS FOR C/L ON RPM HAVE BEEN MET    CONDITIONS ARE TPS CLOSED AND LOW MPH.
          3            1 = C/L ON RPM ENABLED   CONDITIONS MET LONG ENOUGH OR LOW RPM ON A/T VEH.
          4            1 = STALL SAVER
          5            NOT USED
          6            NOT USED
          7            1 = IDLE RPM TO HIGH (SIGN)
* 5      ISSPMP       IAC PRESENT MOTOR POSITION          N = IAC MOTOR STEPS
* 6      MALFFLG1     MALF FLAG WORD 1
          0            CODE 21  THROTTLE POSITION HIGH
          1            CODE 19  NOT USED
          2            CODE 18  NOT USED
          3            CODE 17  NOT USED
          4            CODE 16  2002 PPM VEH. SPEED SENSOR FAILURE
          5            CODE 15  COOLANT SENSOR LOW TEMPERATURE
          6            CODE 14  COOLANT SENSOR HIGH TEMPERATURE
          7            CODE 13  OXYGEN SENSOR
* 7      MALFFLG2     MALF FLAG WORD 2
          0            CODE 29  NOT USED
          1            CODE 28  NOT USED
          2            CODE 27  NOT USED
          3            CODE 26  NOT USED
          4            CODE 25  MAT SENSOR HIGH
          5            CODE 24  NOT USED
          6            CODE 23  MAT SENSOR LOW
          7            CODE 22  THROTTLE POSITION LOW
* 8      MALFFLG3     MALF FLAG WORD 3
          0            CODE 38  NOT USED
          1            CODE 37  NOT USED
          2            CODE 36  NOT USED
          3            CODE 35  IAC FAILURE           //Not set in rom, but mentioned in service manual as a valid code
          4            CODE 34  MAP SENSOR LOW
          5            CODE 33  MAP SENSOR HIGH
          6            CODE 32  EGR FAILURE
          7            CODE 31  NOT USED
* 9      MALFFLG4     MALF FLAG WORD 4
          0            CODE 47  NOT USED
          1            CODE 46  NOT USED
          2            CODE 45  OXYGEN SENSOR RICH
          3            CODE 44  OXYGEN SENSOR LEAN
          4            CODE 43  NOT USED
          5            CODE 42  EST. MONITOR          //Ignition System Failure
          6            CODE 41  1X (CAM PULSE) SENSOR FAILURE
          7            CODE 39  NOT USED
*10      MALFFLG5     MALF FLAG WORD 5
          0            CODE 56  NOT USED
          1            CODE 55  PCM ERROR             //Not set in rom, but mentioned in service manual as a valid code
          2            CODE 54  FUEL PUMP RELAY MALFUNCTION
          3            CODE 53  SYSTEM VOLTAGE HIGH   //Not set in rom, but mentioned in service manual as a valid code
          4            CODE 52  NOT USED
          5            CODE 51  PROM ERROR
          6            CODE 49  NOT USED
          7            CODE 48  NOT USED
*11      ISDSMP       IAC DESIRED MOTOR POSITION           N = IAC MOTOR STEPS
 12      DIACMW1      IDLE AIR CONTROL NV MODE WORD
          0            1 = MOTOR RESET IN PROGRESS
          1            FIRST DRIVEAWAY FLAG FOR IAC KICKDOWN LOGIC    1 = IACV COLD OFFSET HAS BEEN KICKED DOWN THIS START
          2            0 = RESET REQUESTED
          3            NOT USED
          4            1 = A STABLE IDLE WITH WARM ENGINE HAS OCCURRED THIS RUN CYCLE - A/C OFF IN DRIVE
          5            NOT USED
          6            1 = FIRST PASS OF MALF 36 HAS FAILED
          7            NOT USED
 13      DIACMW2      IDLE AIR CONTROL NV MODE WORD
          0            MOTOR DIRECTION                    1=EXTEND 0=RETRACT
          1            NOT USED
          2            COIL A STATE
          3            COIL B STATE
          4            STEPPER MOTOR ON/OFF STATUS        1=ON     0=OFF
          5            NOT USED
          6            NOT USED
          7            NOT USED
 14      DIACMW4      MODE WORD FOR IDLE CONTROL
          0            1 = ETC ONCE FLAG                  1= SPECIAL IACV OPEN LOOP COLD ENG MODIFIERS ARE DISABLE
          1            1 = ETC * K97_EDP
          2            NOT USED
          3            NOT USED
          4            NOT USED
          5            1 = PART 2 OF DIAGNOSTIC TEST TO BE RUN
          6            1 = PROP LIMITING AUTHORITY BEING EXERCISED
          7            1 = ADD DERIVATIVE TERM TO GPSFLOW   0 = SUBTRACT DEVIVTIVE TERM FROM GPSFLOW
*15      COOLDEGA     NORMALIZED ENGINE TEMPERATURE (nondefaulted)        DEGREES C = .75N - 40     DEGREES F = 1.35N - 40
*16      ADBAT        BATTERY VOLTAGE A/D VALUE                           VOLTAGE = N/10
*17      ADTHROT      THROTTLE POSITION A/D VALUE                         VOLTS = N*5/256
*18      ADMAP        MANIFOLD PRESSURE A/D VALUE (updated in 100ms loop) VOLTS = N*5/256     kpa = (N + 28.06)/2.71
*19      ADO2A        OXYGEN SENSOR VARIABLE                              mV = 4.42N
 20      MWAF         AIR FUEL MODE WORD
          0            PE DELAY TIME COMPLETE FLAG
          1            NOT USED
          2            BL. ADDRESS CHANGE FLAG             1=CHANGE
          3            DELAY BLM UPDATE                    1=BL ADDR CHANGE
          4            DE FLAG                             1=DE IS ACTIVE
          5            PE FLAG                             1=PE IS ACTIVE
          6            AE FLAG                             1=AE IS ACTIVE
          7            DELIVER ASYNCH. PULSE FLAG
 21      SDMW         SERIAL DATA MODE WORD
          0            1 = IN MODE 10 OF ALDL (MESSAGE ID=$F5)
          1            1 = IN MODE 10 OF ALDL (MESSAGE ID=$F4)
          2            1 = TRANSMISSION DIAGNOSTICS DISABLED
          3            NOT USED
          4            NOT USED
          5            1 = TRANSMIT OVERRUN HAS OCCURRED
          6            1 = TRANSMISSION IN PROGRESS
          7            1 = SECOND BYTRE TRANSMISSION PENDING
 22      MWBG         MINOR LOOP MODE FLAG
          0            FACTORY TEST ENTERED
          1            AE CLAMP FLAG                       1=CLAMP IS ACTIVE
          2            SKIP MALF 42 DUE TO ALDL
          3            1st REF FLAG                        1=REFERENCE PERIOD
          4            1=IGNITION OFF
          5            1=HIGH MAT CONDITIONS OBSERVED
          6            FIRST GOOD M42A FLAG
          7            LOCK-IN MALF 42A                    1=LOCKED IN
 23      MW1          MINOR MODE WORD 1
          0            ADVANCE FLAG                        0=ADV.  1=RETARD
          1            CHECK ENGINE LIGHT DELAY FLAG
          2            LOOP RAN OVER 6.25 MSEC
          3            OPEN TPS VE FLAG                    1=OPEN
          4            RUN FUEL FLAG                       1=RUNNING
          5            VE INT RESET FLAG                   1=RESET
          6            MAJOR LOOP EST MONITOR ENABLE
          7            ENGINE RUNNING FLAG                 1=RUNNING
 24      MW2          MINOR LOOP MODE WORD 2
          0            SYNCHRONOUS MAP SENSOR READS IN EFFECT
          1            O/L IDLE FLAG FOR AIR SWITCH ENGAGE AT IDLE
          2            REFERENCE PULSE OCCURRED
          3            1 = DIAGNOSTIC SWITCH IN FACTORY TEST POSITION
          4            1 = DIAGNOSTIC SWITCH IN DIAGNOSTIC POSITION
          5            1 = REF PULSE HAS OCCURRED
          6            1 = IDLE SPARK ENABLED
          7            IDLE FLAG
*25      IODPORTC     I/O PORT C
          0            NOT USED
          1            BRAKE SWITCH          (1=BRAKE PRESSED)
          2            NOT USED
          3            NOT USED
          4            Park/Neutral Switch   (0=P/N)
          5            NOT USED
          6            NOT USED
          7            NOT USED
 26      CLCCMW       MAJOR LOOP MODE WORD 1
          0            SYNCHRONOUS AE FLAG               1=SYNCH AE
          1            SLOW RICH/LEAN FLAG               1=RICH
          2            AIR MANAGEMENT ON                 1=ON           //I dont think I have this one enabled, check rom
          3            DECEL FUEL CUT-OFF FLAG           1=DFCO
          4            1 = OVERSPEED FUEL SHUTOFF
          5            DFCO IAC FAST FILTERED FLAG
          6            1 = N.V. MEMORY BOMBED
          7            1 = HAS BEEN IN C.L AT LEAST ONCE SINCE RESTART
*27      ADBARO       RAW A/D COUNTS FOR BARO FILT IN TRANS   VOLTS = N*5/256   kpa = (N + 28.06)/2.71
 28      LCCPMW       TCC & A/C MODE WORD
          0            NOT USED
          1            1 = SPK. CORRECTION DUE TO MAT IS NEGATIVE
          2            NOT USED
          3            NOT USED
          4            NOT USED
          5            PARK/NEUTRAL
          6            NOT USED
          7            NOT USED
 29      MWAF1        AIR FUEL MODE WORD 1
          0            DFCO TPS AE FLAG
          1            LEARN CONTROL ENABLE FLAG      1=ENABLE
          2            1 = LOW BATTERY
          3            A/F DECAY INT DONE FLAG FOR COLD PRK TO DRIVE
          4            ASYNCHRONOUS PULSE FLAG (AP FLAG)
          5            CLOSED LOOP FOR O/L IDLE CONDITION
          6            RICH-LEAN FLAG                 1=RICH  0=LEAN
          7            CLOSED LOOP FLAG               1=CLOSED
 30      TCCMODE      TCC MODE WORD FLAGS
          0            1 = TCC IS BEING FORCED OFF
          1            1 = TCC ENABLE SOLENOID VALID
          2            1 = LOW THRESHOLD FOR COPETCC SELECTED
          3            1 = USE HYST FOR VEH. SPD.
          4            1 = TCC IS IN RELEASE MODE SLIPPING
          5            1 = TCC IS IN LOCK-ADJUST MODE   //seen active in logs, Lockup Output on? Rest is likely unused.
          6            1 = TCC IS BEING APPLIED
          7            1 = NEGATIVE SLIP RECENTLY PREVENTS APPLY
*31      FILTMPH      FILTERED MPH VARIABLE     MPH = N     KPH = 1.61N
 32      IODPORTB     SOLENOID COMBINATION FOR DIGITAL EGR
          0            NOT USED
          1            NOT USED
          2            IAC COIL A
          3            IAC COIL B
          4            IAC ENABLE                        1=ENABLE
          5            FORCE MOTOR ENABLE                1=ENABLE   //unused?
          6            REF IRQ CLR
          7            M/CLR
*33      PPSWVLT      BATTERY VOLTAGE FROM PPSW A/D COUNTS (Fuel Pump Relay Output Volts)   VOLTS = N/10
*34      NTRPMX       RPM VARIABLE USED FOR TABLE F1 EXTENSION LOGIC      RPM = N * 25
*35      OLDRFPER     LAST MINOR LOOP REFERENCE PERIOD FROM ECU (MSB)
*36      OLDRFPER+1   LAST MINOR LOOP REFERENCE PERIOD FROM ECU (LSB)     MSEC = N/65.536   RPM = 65.536* # OF CYLINDER/N     6 CYL = 20  8 CYL = 15
*37      EGRDC        EGR DUTY CYCLE        % = N/2.56    //only 0% or 100% output
 38      MW3          MISCELLANEOUS MODE WORD
          0            CPI/PFI SINGLE FIRE FLAG           1=SINGLE FIRE
          1            SINGLE FIRE FIRST TIME             1=FIRST TIME
          2            REFRESH RAM IN BACK GROUND         1=YES
          3            1 = POWERDOWN IN PROGRESS
          4            1 = HIGH BATTERY VOLTAGE
          5            DFCO SPARK FILTER FLAG
          6            START-UP SPARK FILTER DONE FLAG    1 = DONE
          7            TRANSITION FLAG                    1=TRANSITION ACTIVE
*39      TIMEENG      ENGINE RUNNING TIME,SECONDS (MSB)
*40      TIMEENG+1    ENGINE RUNNING TIME,SECONDS (LSB)   SECONDS = N
*41      DESSPD       DESIRED IDLE RPM                    RPM = 12.5N
*42      NDTHRPOS     THROTTLE POSITION FOR ENGINE        % = N/2.56
 43      MW4          MODE WORD 4
          0            1 = HIGH ESC ACTIVITY FLAG
          1            1 = ZERO ACTIVITY FLAG
          2            BLOCK LEARN CLOSED THROTTLE POSITION FLAG  1=THROT CLOSED
          3            1 = VARIABLE TUNING CONTROL ENABLED
          4            NOT USED
          5            NOT USED
          6            NOT USED
          7            1 = QUASI CLOSED LOOP ENABLED
 44      MW5          MODE WORD 5
          0            1 = ACTUAL ENGINE SPEED OVER DESIRED SPEED
          1            1 = DELATCH ACTIVE
          2            NOT USED
          3            NOT USED
          4            1 = STALL SAVER ACTIVE
          5            1 = A-INJECTORS FIRED AT FIRST REF. PULSE FLAG
          6            1 = SINGLE FIRE ALT EXIT IS DESIRED
          7            NOT USED
*45      SAREFFNL     FINAL VALUE OF SAREF(MSB)
*46      SAREFFNL     FINAL VALUE OF SAREF(LSB)                           DEGREES = (N*90/256) + 66  //Double check this math, its likely wrong
 47      PA2OLD       ECU PA2 COUNTER VALUE FROM LAST MINOR LOOP(MSB)     N = COUNTS
 48      PA2OLD+1     ECU PA2 COUNTER VALUE FROM LAST MINOR LOOP(LSB) NOT USED (?)
*49      INT          CLOSED LOOP INTEGRATOR VALUE                        N = INTEGRATOR COUNTS
 50      DESTPS       DESIRED GOVERNING TPS VALUE TO BE OUTPUT            % = N/2.56
*51      ALDLCNTR     ALDL RICH/LEAN CHANGE COUNTER                       N = COUNTS
*52      LEGRMW       EGR & SPARK MODE WORD
          0            EGR DIAG INT RESET FLAG
          1            NOT USED
          2            EGR HI VAC HYST FLAG
          3            AE FIRST TIME FLAG
          4            EGR MAP HYST FLAG
          5            EGR TPS HYST FLAG
          6            EGR MPH HYST FLAG
          7            EGR ON CONDITION
 53      GOVMW        ELECTONIC GOVERNOR MODE WORD
          0            1 = CURRENTLY IN RPM GOVERNING MODE
          1            1 = GOV LEAD MODE ANTICIPATING RPM GOVERNING
          2            RPM OVERSPEED FLAG     1 = RPM OVERSPEED
          3            GOVERNOR OVER SPEED LIGHT ON
          4            1 = RPM LEAD TPS RETURN MODE
          5            NOT USED
          6            MPH OVERSPEED FLAG     1 = MPH OVERSPEED
          7            1 = CURRENTLY IN MPH GOVERNING MODE
*54      BLMCELL      BLOCK LEARN MULTIPLIER CELL     N = BLOCK LEARN CELL 
*55      BLM          BLOCK LEARN MULTIPLIER          N = MULTIPLIER
*56      N/A
*57      BPW          BASE PULSE WIDTH(MSB)
*58      BPW+1        BASE PULSE WIDTH(LSB)     mSEC = N/65.536
*59      DSEGRPOS     DESIRED EGR COMMAND       % = N/2.56         //only 0% or 100% output
*60      N/A
*61      N/A 
*62      N/A
 63      MATDEGA      NON DEFAULTED MAT        DEGREES C = .75N - 40    DEGREES F = 1.35N - 40    //nonfuctional on my LLV? possible hardware failure?
 
* = Confirmed for LLV, notes added otherwise
 */
