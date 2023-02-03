#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Octoliner.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int RED = 4;
const int BLUE = 10;
const int BUZZ = 7;
const int BUTTON1 = 2;
const int BUTTON2 = 12;
const int SW1 = 8;
const int SW2 = 9;
const int MOTORL1 = 3;
const int MOTORL2 = 5;
const int MOTORR1 = 6;
const int MOTORR2 = 11;

int TH_LINE = 450;
const int TH_RING = 450;
const int TH_BRICK = 170;
const int TH_Jar = 130;

byte DL = 0;

const int numSens = 8;

const int Sharp_L = A6;
const int Sharp_C = A1;
const int Sharp_R = A0;
int Iter_Sharp = 3;
const int TH_Side_Wall_1 = 350;  //350
const int TH_Side_Wall_2 = 350;  //350
const int TH_Center_Wall = 350;  //380
int target = 4500;
int LastPos = target;
const int MaxValue = 9000;
const int MinValue = 0;
int maxSpeed = 255;
int minSpeed = -51;
int MinTemp = minSpeed;

int TimeLine = 400;

int minWhite[numSens];
int maxWhite[numSens];

int IR_L;
int IR_C;
int IR_R;
int DIP = 0;
const int IRON = 13;
int sens[numSens];

int BSpeed = 80;  //220
int SpeedTemp = BSpeed;
const float Kp = 0.05;  //0.08
const float Kd = 0.5;   //0.8
int WheelL = BSpeed;
int WheelR = BSpeed;
const int BrickKP = 55;
int TH_CANonFinish = 200;
int TH_CANonStart = 200;

enum LINE_FOLLOW : byte { CROSS,
                          BRICK,
                          TIME,
                          LINE };

enum CMD : byte { MASTER = 0,
                  SLAVE,
                  SENSOR,
                  LINEFOLLOW,
                  ROWLINE };
enum compare : byte { more,
                      less };

const byte sizeMenu = 5;
String menu[sizeMenu] = {
  "Go master",
  "Go slave",
  "Sensors",
  "Line Test",
  "RowLine"
};

byte active = 0;

int Ws[numSens] = { 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000 };

Octoliner octoliner(42);

void readSensors() {
  for (int i = 0; i < numSens; i++) {
    sens[i] = octoliner.analogRead(numSens - 1 - i);
  }
}

void printSensors() {

  for (int i = 0; i < numSens; i++) {
    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(10, 0);
    display.print(sens[i]);
    display.print(' ');
  }
  display.println();
}

void drive(int left, int right) {
  left = constrain(left, minSpeed, maxSpeed);
  right = constrain(right, minSpeed, maxSpeed);
  if (left >= 0) {
    digitalWrite(MOTORL1, HIGH);
    analogWrite(MOTORL2, 255 - left);
  } else {
    digitalWrite(MOTORL2, HIGH);
    analogWrite(MOTORL1, 255 + left);
  }
  if (right >= 0) {
    digitalWrite(MOTORR1, HIGH);
    analogWrite(MOTORR2, 255 - right);
  } else {
    digitalWrite(MOTORR2, HIGH);
    analogWrite(MOTORR1, 255 + right);
  }
}
void getJar() {
  delay(1000);  // симуляция манипулятора
}
void putJar() {
  delay(1000);  // симуляция манипулятора
}
void DIPcheck() {
  digitalWrite(BLUE, !digitalRead(SW2));
  digitalWrite(RED, !digitalRead(SW1));
}

void beep(int i) {
  digitalWrite(BUZZ, HIGH);
  delay(300);
  digitalWrite(BUZZ, LOW);
}

int readLinePDA() {
  static const int TH_LINE = 400;
  static const int Noise = 120;

  bool online = false;
  long SumWV = 0;
  int SumV = 0;
  byte weight = 128;

  DL = 0;

  for (int i = 0; i < numSens; i++) {
    sens[i] = octoliner.analogRead(numSens - 1 - i);
    if (sens[i] > TH_LINE) {
      online = true;
      DL = DL + weight;
    }
    weight /= 2;
    if (sens[i] > Noise) {
      SumWV = SumWV + Ws[i] * (long)sens[i];
      SumV = SumV + sens[i];
    }
  }

  if (online == false) {
    if (LastPos < target) {
      return MinValue;
    } else {
      return MaxValue;
    }
  }
  LastPos = SumWV / SumV;
  return LastPos;
}

void LineFollow(byte type) {

  long Start;
  int error = 0;
  int lastError = 0;
  int deltaError = 0;
  int delta = 0;

  LastPos = readLinePDA();
  //DL = readLine();
  if (type == TIME) {
    Start = millis();
  }

  while (1) {

    if (type == BRICK) {
      if (readSharp(Sharp_C) > TH_BRICK) {
        /*drive(0, 0);
          display.clearDisplay();
          display.setCursor(0, 0);
          display.print(readSharp(Sharp_C));
          display.display();
          while(1);*/
        delay(40);  // const int DELAY_BRICK = 40 !!!
        if (readSharp(Sharp_C) > TH_BRICK) {
          drive(0, 0);
          return;
        }
      }
    } /* else if(type == RELOADED_BRICK) {
      if (readSharp(Sharp_C) > TH_BRICK) {
        return;
    }*/
    else if (type == CROSS) {
      if ((DL & B11111111) == B11111111) {  //DL == B11111111 / B01111110
        drive(0, 0);
        return;
      }
    } else if (type == TIME) {
      if (millis() - Start > TimeLine) {
        return;
      }
    } else if (type == LINE) {
      if ((digitalRead(BUTTON2)) == 0) {
        return;
      }
    }
    //=====================================================================================================//
    error = target - readLinePDA();
    deltaError = error - lastError;
    lastError = error;
    delta = error * Kp + deltaError * Kd;
    drive(BSpeed - delta, BSpeed + delta);
    //=====================================================================================================//
  }
}

void showALLSensors() {
  while (1) {
    readSensorsMaze();
    display.clearDisplay();
    display.setTextSize(2);

    display.setCursor(45, 0);
    display.print(IR_C);

    display.setTextSize(1);
    for (int i = 0; i < numSens; i++) {
      int value = octoliner.analogRead(8 - i - 1);
      display.setCursor(20 + (i - 1) * 15, 25);
      if (value > TH_LINE) {
        display.print(1);
      } else {
        display.print(0);
      }
    }

    display.display();
    if (digitalRead(BUTTON2) == 0) {
      delay(200);
      return;
    }
    delay(250);
  }  // while
}

int readSharp(byte pin) {
  int res = 0;
  for (int i = 0; i < Iter_Sharp; i++) {
    res += analogRead(pin);
  }
  return res / Iter_Sharp;
}

void readSensorsMaze() {
  IR_L = readSharp(Sharp_L);
  IR_C = readSharp(Sharp_C);
  IR_R = readSharp(Sharp_R);
}

void showRawLine() {
  const byte shiftX = 32;
  const byte shiftY = 15;
  display.setTextSize(1);
  while (1) {
    if (digitalRead(BUTTON2) == 0) {
      return;
    }
    display.clearDisplay();
    for (int i = 0; i < numSens; i++) {
      int value = octoliner.analogRead(numSens - i - 1);
      display.setCursor(shiftX * (i % 4), 10 + (i / 4) * shiftY);
      display.print(value);
    }
    display.display();
    delay(400);
  }
}

byte readLine() {
  byte DL = 0;
  byte w = 128;
  for (int i = 0; i < numSens; i++) {
    int value = octoliner.analogRead(numSens - 1 - i);
    if (value > TH_LINE) {
      DL = DL + w;
    }
    w = w / 2;
  }  // for
  return DL;
}

void readread() {
  Serial.begin(9600);
  while (1) {
    int L = readSharp(Sharp_L);
    int C = readSharp(Sharp_C);
    int R = readSharp(Sharp_R);
    Serial.print(L);
    Serial.print(' ');
    Serial.print(C);
    Serial.print(' ');
    Serial.print(R);
    Serial.println();
    delay(200);
  }
}


void showMsg(const String& msg) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(msg);
  display.display();
}

void updateMenu(byte i) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(15, 6);
  display.print(menu[i]);
  display.display();
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Start");
  display.setCursor(100, 25);
  display.print("Next");
  display.display();
}

void test() {
  for (int i = 0; 0 <= 10; i++) {
    drive(100, 100);
    delay(2000);
    drive(0, 0);
    delay(100);
    drive(-100, -100);
    delay(2000);
    drive(0, 0);
    delay(100);
  }
}
void goingMaster() {
  goingToJar();
}

void goingSlave() {
  WaitOnStart();
  goingToStart();
  WaitOnFinish();
}
void goingToStart() {
  drive(50, 50);
  while (1) {
    byte DL = readLine();
    if (DL == B11111111) {
      drive(0, 0);
      break;
    }
  }
  drive(50, -50);
  byte DL = readLine();
  while (1) {
    if (DL == B01111110) {
      drive(0, 0);
      break;
    }
  }
}
void WaitOnStart() {
  while (!CheckJar(TH_CANonStart, less)) {}
}
//ждем пока робот уедет
void WaitOnFinish() {
  while (!CheckJar(TH_CANonFinish, more)) {}
}
//ждем пока робот приедет
bool CheckJar(int Th, compare sign) {
  switch (sign) {
    case more:
      return (readSharp(Sharp_C) > Th);
      break;
    case less:
      return (readSharp(Sharp_C) < Th);
      break;
  }
}
void goingToJar() {
  delay(1000);
  drive(50, 50);
  while (!CheckJar(TH_CANonStart, less))
    drive(0, 0);
  getJar();
  drive(-50, 50);
  while (!StopIfDetectCenterLine) {}
  lineFolow();
}
void lineFolow() {

  delay(100);
  TimeLine = 800;
  LineFollow(TIME);
  LineFollow(CROSS);
  TimeLine = 500;
  LineFollow(TIME);
  drive(0, 0);
  delay(1000);
  drive(-50, -50);
  delay(1000);
  drive(0, 550);
  TimeLine = 250;
  LineFollow(TIME);
  drive(0, 0);
  while (!StopIfDetectHorisontLine()) {}
  goingSlave();
}
bool StopIfDetectHorisontLine() {
  byte DL = readLine();
  if (DL == B11111111) {
    drive(0, 0);
    return true;
  }
  return false;
}
bool StopIfDetectCenterLine() {
  byte DL = readLine();
  if (DL == B01111110) {
    drive(0, 0);
    return true;
  }
  return false;
}



void stop() {
  drive(0, 0);
  while (1)
    ;
}

void showLogo() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 5);
  display.print("Dual Berettas");
  display.display();
  delay(2000);
}

void setup() {
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  octoliner.begin();
  octoliner.setSensitivity(200);
  //octoliner.setBrightness(255);

  // Set Timer2 PWM phase correct(pin 11, pin 3) :
  // 16 000 000 / (8 * 510) = 3921.57Hz
  // CS22 CS21 CS20  010 - pescale 8
  //TCCR2B |= (1 << CS21 );
  //TCCR2B &= ~(1 << CS22 | 1 << CS20);

  // speed up ADC
  ADCSRA |= (1 << ADPS2);
  ADCSRA &= ~(1 << ADPS1);
  ADCSRA |= (1 << ADPS0);

  showLogo();

  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(IRON, OUTPUT);
  pinMode(MOTORL1, OUTPUT);
  pinMode(MOTORL2, OUTPUT);
  pinMode(MOTORR1, OUTPUT);
  pinMode(MOTORR2, OUTPUT);
  //Serial.begin(9600);
  //digitalWrite(IRON, HIGH);
  //DIP = !digitalRead(SW2);
  //DIP = DIP + 2 * (!digitalRead(SW1));
  //DIPcheck();

  //showALLSensors();

  digitalWrite(BUZZ, HIGH);
  delay(500);
  digitalWrite(BUZZ, LOW);

  /*  digitalWrite(BLUE, HIGH);
    delay(500);
    digitalWrite(BLUE, LOW);
    delay(1000);*/
}


void loop() {
  updateMenu(active);
  while (1) {
    if (digitalRead(BUTTON2) == 0) {
      active = active + 1;
      if (active == 5) {
        active = 0;
      }
      digitalWrite(BLUE, HIGH);
      updateMenu(active);
      delay(200);
      digitalWrite(BLUE, LOW);

    } else if (digitalRead(BUTTON1) == 0) {
      digitalWrite(RED, HIGH);
      delay(200);
      digitalWrite(RED, LOW);
      break;
    }
  }  // while

  if (active == MASTER) {
    showMsg("Drive master");
    goingMaster();
  }
  if (active == SLAVE) {
    showMsg("Drive slave");
    goingSlave();
  }
  if (active == SENSOR) {
    showALLSensors();
    return;
  }
  if (active == LINEFOLLOW) {
    showMsg("23th mart");
    LineFollow(LINE);
  }
  if (active == ROWLINE) {
    showRawLine();
  }
}