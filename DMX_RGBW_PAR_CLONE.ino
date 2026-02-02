#include <DMXSerial.h>
#include <EEPROM.h>

// ----- Pins -----
#define ledR  9
#define ledG 10
#define ledB 11
#define ledW  3

const int pins[6] = {2,4,5,6,7,8}; // 7-seg pins

#define btnMenu   A0
#define btnUp     A1
#define btnDown   A2
#define btnEnter  A3
#define mic       12
#define RXLED     13

// ----- DMX -----
enum DMXChMode : uint8_t { MODE_4CH = 0, MODE_8CH = 1 };
DMXChMode dmxChMode = MODE_4CH;

struct DMXChannels8 { uint8_t main, red, green, blue, white, strobe, macro, speed; };
DMXChannels8 dmx8;
uint16_t addr = 1;

// ----- States -----
enum State : uint8_t { STATE_ADDR=0, STATE_MODE, STATE_COLOR, STATE_MACRO, STATE_STROBE, STATE_SPEED };
State state = STATE_ADDR;

enum SubState : uint8_t { SUB_RED, SUB_GREEN, SUB_BLUE, SUB_WHITE };
SubState substate = SUB_RED;

// ----- EEPROM addresses -----
#define EEPROM_STATE     0
#define EEPROM_ADDR_LOW  1
#define EEPROM_ADDR_HIGH 2
#define EEPROM_MODE      3
#define EEPROM_VALUE_R   4
#define EEPROM_VALUE_G   5
#define EEPROM_VALUE_B   6
#define EEPROM_VALUE_W   7
#define EEPROM_MACRO     8
#define EEPROM_STROBE    9
#define EEPROM_SPEED     10

// ----- Static colors -----
enum Colors : uint8_t { STAT_COL_RED = 0, STAT_COL_GREEN, STAT_COL_BLUE, STAT_COL_WHITE, STAT_COL_YELLOW, STAT_COL_PURPLE , STAT_COL_CYAN, STAT_COL_RGB };
struct LEDsColor  { uint8_t red, green, blue, white; };
const LEDsColor staticColors[8] = {
  { 255, 0, 0, 0},    //STAT_COL_RED
  { 0, 255, 0, 0},    //STAT_COL_GREEN
  { 0, 0, 255, 0},    //STAT_COL_BLUE
  { 0, 0, 0, 255},    //STAT_COL_WHITE
  { 255, 127, 0, 0},  //STAT_COL_YELLOW
  { 255, 0, 127, 0},  //STAT_COL_PURPLE
  { 0, 255, 255, 0},  //STAT_COL_CYAN
  { 255, 255, 255, 0} //STAT_COL_RGB
};

// ----- Display -----
struct Segment { uint8_t anode,cathode; };
const Segment digitMap[4][7] = {
  {{2,6},{4,2},{5,2},{8,2},{7,2},{5,6},{6,2}},
  {{2,8},{4,8},{5,8},{7,8},{6,8},{5,4},{2,4}},
  {{8,4},{7,4},{6,4},{4,7},{5,7},{2,7},{8,7}},
  {{6,7},{4,5},{8,5},{7,5},{2,5},{6,5},{4,6}}
};
char displayChars[4] = {'0','0','0','0'};
int refreshDigit=0, refreshSegment=0;
unsigned long lastRefresh=0;
const unsigned int segmentTime = 150;

// Numeric patterns (0–9)
const bool numPatterns[10][7] = {
  {1,1,1,1,1,1,0}, // 0
  {0,1,1,0,0,0,0}, // 1
  {1,1,0,1,1,0,1}, // 2
  {1,1,1,1,0,0,1}, // 3
  {0,1,1,0,0,1,1}, // 4
  {1,0,1,1,0,1,1}, // 5
  {1,0,1,1,1,1,1}, // 6
  {1,1,1,0,0,0,0}, // 7
  {1,1,1,1,1,1,1}, // 8
  {1,1,1,1,0,1,1}  // 9
};

// Letter patterns (A..Z subset usable on 7-seg)
struct LetterPattern {
  char c;
  bool segs[7];
};

const LetterPattern letterPatterns[] = {
  { 'A', {1,1,1,0,1,1,1} },
  { 'b', {0,0,1,1,1,1,1} },
  { 'C', {1,0,0,1,1,1,0} },
  { 'd', {0,1,1,1,1,0,1} },
  { 'E', {1,0,0,1,1,1,1} },
  { 'F', {1,0,0,0,1,1,1} },
  { 'G', {1,0,1,1,1,1,0} },
  { 'H', {0,1,1,0,1,1,1} },
  { 'L', {0,0,0,1,1,1,0} },
  { 'O', {1,1,1,1,1,1,0} },
  { 'o', {0,0,1,1,1,0,1} },
  { 'P', {1,1,0,0,1,1,1} },
  { 'r', {0,0,0,0,1,0,1} },
  { 'S', {1,0,1,1,0,1,1} },
  { 't', {0,0,0,1,1,1,1} },
  { 'U', {0,1,1,1,1,1,0} },
  { 'u', {0,0,1,1,1,0,0} }
};
const int numLetters = sizeof(letterPatterns) / sizeof(letterPatterns[0]);

// ----- Strobe -----
unsigned long lastStrobeTime=0;
int strobeDisplay = 0;

// ----- Buttons -----
#define DEBOUNCE_DELAY 150  // ms
unsigned long lastButtonTime = 0;
enum Button : uint8_t { BTN_NONE=0, BTN_MENU, BTN_UP, BTN_DOWN, BTN_ENTER };
Button checkBtn() {
  if (millis() - lastButtonTime < DEBOUNCE_DELAY) return BTN_NONE; // debounce

  if (!digitalRead(btnMenu))  { lastButtonTime = millis(); return BTN_MENU; }
  if (!digitalRead(btnUp))    { lastButtonTime = millis(); return BTN_UP; }
  if (!digitalRead(btnDown))  { lastButtonTime = millis(); return BTN_DOWN; }
  if (!digitalRead(btnEnter)) { lastButtonTime = millis(); return BTN_ENTER; }

  return BTN_NONE;
}

// ----- Core functions -----
void resetPins() {
  for(int i=0;i<6;i++) pinMode(pins[i], INPUT);

  pinMode(ledR,OUTPUT); pinMode(ledG,OUTPUT); pinMode(ledB,OUTPUT); pinMode(ledW,OUTPUT);

  pinMode(RXLED,OUTPUT); digitalWrite(RXLED,LOW);

  pinMode(btnMenu, INPUT_PULLUP);
  pinMode(btnUp, INPUT_PULLUP);
  pinMode(btnDown, INPUT_PULLUP);
  pinMode(btnEnter, INPUT_PULLUP);

  pinMode(mic, INPUT);
}

void refreshDisplayNonBlocking() {
  unsigned long now = micros();
  if (now - lastRefresh < segmentTime) return;
  lastRefresh = now;
  resetPins();

  char c = displayChars[refreshDigit];
  bool segs[7] = {0};
  if (c >= '0' && c <= '9') {
    for (int i=0; i<7; i++) segs[i] = numPatterns[c - '0'][i];
  } else {
    for (int i = 0; i < numLetters; i++) {
      if (letterPatterns[i].c == c) {
        for (int j = 0; j < 7; j++) segs[j] = letterPatterns[i].segs[j];
        break;
      }
    }
  }

  if (segs[refreshSegment]) {
    Segment s = digitMap[refreshDigit][refreshSegment];
    int an=-1, ca=-1;
    for(int i=0;i<6;i++){
      if(pins[i]==s.anode) an=i;
      if(pins[i]==s.cathode) ca=i;
    }
    if(an>=0 && ca>=0){
      pinMode(pins[an],OUTPUT);
      pinMode(pins[ca],OUTPUT);
      digitalWrite(pins[an],HIGH);
      digitalWrite(pins[ca],LOW);
    }
  }

  refreshSegment++;
  if(refreshSegment>=7){ refreshSegment=0; refreshDigit++; if(refreshDigit>=4) refreshDigit=0; }
}

void displayStringOrNumber(const char* str){
  for(int i=0;i<4;i++) displayChars[i]=' ';
  int len=strlen(str); if(len>4) len=4;
  for(int i=0;i<len;i++) displayChars[4-len+i]=str[i];
}

// ----- Setup -----
void setup() {
  DMXSerial.init(DMXReceiver);
  resetPins();
  state = (State)EEPROM.read(EEPROM_STATE);
  addr = EEPROM.read(EEPROM_ADDR_LOW) | (EEPROM.read(EEPROM_ADDR_HIGH)<<8);
  dmxChMode = (DMXChMode)EEPROM.read(EEPROM_MODE);
  dmx8.red   = EEPROM.read(EEPROM_VALUE_R);
  dmx8.green = EEPROM.read(EEPROM_VALUE_G);
  dmx8.blue  = EEPROM.read(EEPROM_VALUE_B);
  dmx8.white = EEPROM.read(EEPROM_VALUE_W);
  dmx8.strobe= EEPROM.read(EEPROM_STROBE);
  dmx8.macro = EEPROM.read(EEPROM_MACRO);
  dmx8.speed = EEPROM.read(EEPROM_SPEED);
  if(state>STATE_SPEED) state=STATE_ADDR;
  if(addr==0 || addr>512) addr=1;

  strobeDisplay = round(map(dmx8.strobe, 0, 255, 0, 16));
}

// ----- DMX reading -----
void readDMX() {
  static unsigned long lastPacket = 0;
  lastPacket = DMXSerial.noDataSince();
  if (lastPacket < 2000) {
    digitalWrite(RXLED, HIGH);
    if(dmxChMode==MODE_4CH){
      dmx8.red   = DMXSerial.read(addr);
      dmx8.green = DMXSerial.read(addr+1);
      dmx8.blue  = DMXSerial.read(addr+2);
      dmx8.white = DMXSerial.read(addr+3);
    }else{
      dmx8.main   = DMXSerial.read(addr);
      dmx8.red    = DMXSerial.read(addr+1);
      dmx8.green  = DMXSerial.read(addr+2);
      dmx8.blue   = DMXSerial.read(addr+3);
      dmx8.white  = DMXSerial.read(addr+4);
      dmx8.strobe = DMXSerial.read(addr+5);
      dmx8.macro  = DMXSerial.read(addr+6);
      dmx8.speed  = DMXSerial.read(addr+7);
    }
    digitalWrite(RXLED, LOW); 
  } else {
    digitalWrite(RXLED, LOW);
  }
}

// ----- Macro handler -----
void handleMacro() {
  if ((state == STATE_ADDR) && (dmxChMode != MODE_8CH)) return;

  uint8_t m = dmx8.macro;
  LEDsColor color;

  // ----- STATIC COLORS -----
  if (m <= 3) {
    // Hold: do nothing
    return;
  } else if (m <= 19) { color = staticColors[STAT_COL_RED]; }    // Red
  else if (m <= 35) { color = staticColors[STAT_COL_GREEN]; }    // Green
  else if (m <= 51) { color = staticColors[STAT_COL_BLUE]; }     // Blue
  else if (m <= 67) { color = staticColors[STAT_COL_WHITE]; }    // White
  else if (m <= 83) { color = staticColors[STAT_COL_RGB]; }      // RGB
  else if (m <= 99) { color = staticColors[STAT_COL_YELLOW]; }   // Yellow
  else if (m <= 109){ color = staticColors[STAT_COL_PURPLE]; }   // Purple
  else if (m <= 127){ color = staticColors[STAT_COL_CYAN]; }     // Cyan

  // ----- DYNAMIC COLOR MODES -----
  else if (m <= 169) { // Color jump
    static uint8_t jumpState = 0;
    static unsigned long lastJump = 0;
    uint16_t jumpInterval = map(dmx8.speed, 0, 255, 6000, 100);
    if (millis() - lastJump > jumpInterval) {
      jumpState = (jumpState + 1) % 7;
      lastJump = millis();
    }
    switch (jumpState) {
      case 0: color = staticColors[STAT_COL_RED]; break;
      case 1: color = staticColors[STAT_COL_PURPLE]; break;
      case 2: color = staticColors[STAT_COL_BLUE]; break;
      case 3: color = staticColors[STAT_COL_WHITE]; break;
      case 4: color = staticColors[STAT_COL_CYAN]; break;
      case 5: color = staticColors[STAT_COL_GREEN]; break;
      case 6: color = staticColors[STAT_COL_YELLOW]; break;
    }
  }
  else if (m <= 210) { // Color fade
    static uint16_t fadeCnt = 0;
    static uint8_t fadeVal = 0;
    static unsigned long lastFade = 0;
    static uint8_t fadeState = 0;
    uint16_t fadeInterval = map(dmx8.speed, 0, 255, 100, 0);
    if (millis() - lastFade > fadeInterval) {
      if (fadeCnt > (7*255)) { 
        fadeCnt = 0; 
      } else {
        fadeCnt++;
      }
      fadeVal = fadeCnt % 255;
      fadeState = fadeCnt / 255.;
      lastFade = millis();
    }
    switch (fadeState) {
      case 0: color.red = 255;            color.green = fadeVal;        color.blue = 0;             color.white = 0; break;
      case 1: color.red = (255-fadeVal);  color.green = 255;            color.blue = 0;             color.white = 0; break;
      case 2: color.red = 0;              color.green = 255;            color.blue = fadeVal;       color.white = 0; break;
      case 3: color.red = 0;              color.green = (255-fadeVal);  color.blue = 255;           color.white = 0; break;
      case 4: color.red = fadeVal;        color.green = 0;              color.blue = 255;           color.white = 0; break;
      case 5: color.red = 255;            color.green = 0;              color.blue = (255-fadeVal); color.white = 0; break;
      case 6: color.red = 255;            color.green = 0,              color.blue = 0;             color.white = 0; break;
    }
  }

  // ----- SOUND MODES -----
  else if (m <= 233) { // Sound jump
    static uint8_t jumpState = 0;
    if (digitalRead(mic) == HIGH) {
      jumpState = (jumpState + 1) % 7;
    }
    switch (jumpState) {
      case 0: color = staticColors[STAT_COL_RED]; break;
      case 1: color = staticColors[STAT_COL_PURPLE]; break;
      case 2: color = staticColors[STAT_COL_BLUE]; break;
      case 3: color = staticColors[STAT_COL_WHITE]; break;
      case 4: color = staticColors[STAT_COL_CYAN]; break;
      case 5: color = staticColors[STAT_COL_GREEN]; break;
      case 6: color = staticColors[STAT_COL_YELLOW]; break;
    }
  }
  else { // Sound fade
    static uint32_t fadeCnt = 0;
    static uint8_t fadeVal = 0;
    static uint8_t fadeState = 0;
    if (digitalRead(mic) == HIGH) {
      if (fadeCnt > (7*255)) { 
        fadeCnt = 0; 
      } else {
        fadeCnt += 8;
      }
      fadeVal = fadeCnt % 255;
      fadeState = fadeCnt / 255.;
    }
    switch (fadeState) {
      case 0: color.red = 255;            color.green = fadeVal;        color.blue = 0;             color.white = 0; break;
      case 1: color.red = (255-fadeVal);  color.green = 255;            color.blue = 0;             color.white = 0; break;
      case 2: color.red = 0;              color.green = 255;            color.blue = fadeVal;       color.white = 0; break;
      case 3: color.red = 0;              color.green = (255-fadeVal);  color.blue = 255;           color.white = 0; break;
      case 4: color.red = fadeVal;        color.green = 0;              color.blue = 255;           color.white = 0; break;
      case 5: color.red = 255;            color.green = 0;              color.blue = (255-fadeVal); color.white = 0; break;
      case 6: color.red = 255;            color.green = 0,              color.blue = 0;             color.white = 0; break;
    }
  }

  dmx8.red = color.red;
  dmx8.green = color.green;
  dmx8.blue = color.blue;
  dmx8.white = color.white;
}

// ----- Update LEDs -----
void updateLEDs() {
  if ((state == STATE_ADDR) && (dmxChMode == MODE_4CH)) {
    analogWrite(ledR, dmx8.red);
    analogWrite(ledG, dmx8.green);
    analogWrite(ledB, dmx8.blue);
    analogWrite(ledW, dmx8.white);
  } else {
    // Apply main dimming
    uint8_t r = (dmx8.red   * dmx8.main) / 255;
    uint8_t g = (dmx8.green * dmx8.main) / 255;
    uint8_t b = (dmx8.blue  * dmx8.main) / 255;
    uint8_t w = (dmx8.white * dmx8.main) / 255;

    // Handle macros first (can override color)
    handleMacro();

    // ----- Strobe handling -----
    if (dmx8.strobe <= 15) {
      // Strobe off — show steady light
    } else {
      // Strobe active
      float normalized = (dmx8.strobe - 16) / 239.0;  // 0.0–1.0
      unsigned long strobeInterval = 1000 - (unsigned long)(950 * pow(normalized, 0.6));

      if (millis() - lastStrobeTime >= strobeInterval) {
        lastStrobeTime = millis();
      } else if (millis() - lastStrobeTime >= 10) {
        r = 0; g = 0; b = 0; w = 0;
      }
    }
    analogWrite(ledR, r);
    analogWrite(ledG, g);
    analogWrite(ledB, b);
    analogWrite(ledW, w);
  }
}

// ----- Loop -----
void loop() {
  Button btn = checkBtn();

  // MENU button: always changes state
  if(btn == BTN_MENU){
      state = (State)((state+1)%6);
      substate = SUB_RED; // reset color substate
      EEPROM.write(EEPROM_STATE,state);
  }

  // ENTER button: only in color state
  if(btn == BTN_ENTER && state == STATE_COLOR){
      substate = (SubState)((substate+1)%4); // cycle RGBW
  }

  // UP/DOWN button: adjust values depending on state
  if(btn == BTN_UP || btn == BTN_DOWN){
      int delta = (btn==BTN_UP)?1:-1;
      switch(state){
          case STATE_ADDR:
              addr = constrain(addr+delta, 1, 512);
              EEPROM.write(EEPROM_ADDR_LOW, addr & 0xFF);
              EEPROM.write(EEPROM_ADDR_HIGH, addr >> 8);
              break;

          case STATE_MODE:
              dmxChMode = (DMXChMode)((dmxChMode + delta + 2)%2);
              EEPROM.write(EEPROM_MODE, dmxChMode);
              break;

          case STATE_COLOR:
              switch(substate){
                  case SUB_RED:   dmx8.red   = constrain(dmx8.red+delta,0,255); EEPROM.write(EEPROM_VALUE_R, dmx8.red); break;
                  case SUB_GREEN: dmx8.green = constrain(dmx8.green+delta,0,255); EEPROM.write(EEPROM_VALUE_G, dmx8.green); break;
                  case SUB_BLUE:  dmx8.blue  = constrain(dmx8.blue+delta,0,255); EEPROM.write(EEPROM_VALUE_B, dmx8.blue); break;
                  case SUB_WHITE: dmx8.white = constrain(dmx8.white+delta,0,255); EEPROM.write(EEPROM_VALUE_W, dmx8.white); break;
              }
              break;

          case STATE_STROBE: {
              strobeDisplay = constrain(strobeDisplay + delta, 0, 16);
              dmx8.strobe = (strobeDisplay == 0) ? 0 : map(strobeDisplay, 1, 16, 16, 255); 
              EEPROM.write(EEPROM_STROBE, dmx8.strobe);
              break;
          }

          case STATE_SPEED: {
              int speedDisplay = round(map(dmx8.speed, 0, 255, 1, 16));
              speedDisplay = constrain(speedDisplay + delta, 1, 16);
              dmx8.speed = map(speedDisplay, 1, 16, 0, 255);
              EEPROM.write(EEPROM_SPEED, dmx8.speed);
              break;
          }

          case STATE_MACRO: {
              int macroIndex = 0;

              // Map DMX value to macro index 0–11
              if (dmx8.macro <= 3) macroIndex = 0;
              else if (dmx8.macro <= 19) macroIndex = 1;
              else if (dmx8.macro <= 35) macroIndex = 2;
              else if (dmx8.macro <= 51) macroIndex = 3;
              else if (dmx8.macro <= 67) macroIndex = 4;
              else if (dmx8.macro <= 83) macroIndex = 5;
              else if (dmx8.macro <= 99) macroIndex = 6;
              else if (dmx8.macro <= 107) macroIndex = 7;
              else if (dmx8.macro <= 127) macroIndex = 8;
              else if (dmx8.macro <= 169) macroIndex = 9;
              else if (dmx8.macro <= 210) macroIndex = 10;
              else if (dmx8.macro <= 233) macroIndex = 11;
              else macroIndex = 12;

              // Adjust macro index with buttons
              macroIndex += delta;
              if (macroIndex < 0) macroIndex = 12;
              if (macroIndex > 12) macroIndex = 0;

              // Map index back to DMX value range midpoint
              const uint8_t macroValues[13] = {2,12,28,44,60,76,92,105,118,149,190,222,245};
              dmx8.macro = macroValues[macroIndex];
              EEPROM.write(EEPROM_MACRO, dmx8.macro);
              break;
          }
      }
  }

  if (state == STATE_ADDR) {
    readDMX();
  } else {
    dmx8.main = 255;
  }

  updateLEDs();

  // Display
  char disp[5]="    ";
  switch(state){
    case STATE_ADDR: 
      sprintf(disp,"d%03d",addr); break;
    case STATE_MODE: 
      sprintf(disp,"%s",dmxChMode==MODE_4CH?"4CH":"8CH"); break;
    case STATE_COLOR:
      switch(substate){
        case SUB_RED:   sprintf(disp,"r%03d",dmx8.red); break;
        case SUB_GREEN: sprintf(disp,"G%03d",dmx8.green); break;
        case SUB_BLUE:  sprintf(disp,"b%03d",dmx8.blue); break;
        case SUB_WHITE: sprintf(disp,"u%03d",dmx8.white); break;
      }
      break;
    case STATE_MACRO:
      if (dmx8.macro <= 210) {
        int idx = 0;
        if (dmx8.macro <= 3) idx = 0;
        else if (dmx8.macro <= 19) idx = 1;
        else if (dmx8.macro <= 35) idx = 2;
        else if (dmx8.macro <= 51) idx = 3;
        else if (dmx8.macro <= 67) idx = 4;
        else if (dmx8.macro <= 83) idx = 5;
        else if (dmx8.macro <= 99) idx = 6;
        else if (dmx8.macro <= 109) idx = 7;
        else if (dmx8.macro <= 127) idx = 8;
        else if (dmx8.macro <= 169) idx = 9;
        else idx = 10;
        sprintf(disp, "Au%02d", idx);
      } else {
        int idx = (dmx8.macro <= 233) ? 1 : 2;
        sprintf(disp, "So%02d", idx);
      }
      break;
    case STATE_STROBE:
      sprintf(disp, "St%02d", strobeDisplay); break;
    case STATE_SPEED: {
      int speedDisplay = round(map(dmx8.speed, 0, 255, 1, 16));
      sprintf(disp,"SP%02d",speedDisplay);
      break;
    }
  }
  displayStringOrNumber(disp);
  refreshDisplayNonBlocking();
}
