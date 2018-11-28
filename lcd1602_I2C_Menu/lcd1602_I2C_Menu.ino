/* Ноябрь 2018
   lcd1602 menu by three buttons and interrupts
*/
// -------------------- подключение библиотек ---------------------------------------

// -------------------- LCD1602 -----------------------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f, 16, 2);
#define printByte(args) write(args);

// ------------------------------------- СИМВОЛЫ -------------------------------------
byte pacman1[8] = {B00000, B01110, B10100, B11000, B11100, B01110, B00000, B00000};  // pacman1
// ----------------------- полоски ---------------------------------------------------
/*byte v1[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B11111};
  byte v2[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B11111, B00000};
  byte v3[8] = {B00000, B00000, B00000, B00000, B00000, B11111, B00000, B00000};
  byte v4[8] = {B00000, B00000, B00000, B00000, B11111, B00000, B00000, B00000};
  byte v5[8] = {B00000, B00000, B00000, B11111, B00000, B00000, B00000, B00000};
  byte v6[8] = {B00000, B00000, B11111, B00000, B00000, B00000, B00000, B00000};
  byte v7[8] = {B00000, B11111, B00000, B00000, B00000, B00000, B00000, B00000};
  byte v8[8] = {B11111, B00000, B00000, B00000, B00000, B00000, B00000, B00000};*/
// ------------------------ снежинка -------------------------------------------------
byte s1[8] = {0b00100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte s2[8] = {0b10101, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte s3[8] = {0b01110, 0b10101, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte s4[8] = {0b10101, 0b01110, 0b10101, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000};
byte s5[8] = {0b00100, 0b10101, 0b01110, 0b10101, 0b00100, 0b00000, 0b00000, 0b00000};
byte s6[8] = {0b00000, 0b00100, 0b10101, 0b01110, 0b10101, 0b00100, 0b00000, 0b00000};
byte s7[8] = {0b00000, 0b00000, 0b00100, 0b10101, 0b01110, 0b10101, 0b00100, 0b00000};
byte s8[8] = {0b00000, 0b00000, 0b00000, 0b00100, 0b10101, 0b01110, 0b10101, 0b00100};
// ------------------------------------- СИМВОЛЫ -------------------------------------
// -------------------- LCD1602 ------------------------------------------------------
byte butCentrePin = 4;                   // пин подключения центральной кнопки
byte butLeftPin = 2;                     // пин подключения левой кнопки
byte butRightPin = 3;                    // пин подключения правой кнопки
unsigned long startButPressTimer = 0;    // стартовое значение таймера удержания
unsigned long butPressTimer = 0;         // таймер удержания
bool butIsShort = false;                 // флаг короткого нажатия кнопки
bool butIsLong = false;                  // флаг длинного нажатия кнопки
volatile bool butLeftIsPress = false;    // флаг нажатия левой кнопки
volatile bool butRightIsPress = false;   // флаг нажатия правой кнопки
unsigned int butCentreHoldTime = 1000;   // время удержания для длинного нажатия
unsigned int shortButCount = 0;          // счетчик коротких нажатий
unsigned int longButCount = 0;           // счетчик длинных нажатий
bool butTimerIsCount = false;            // флаг счета таймера кнопки, если true - таймер считает
bool lcdClear = false;                   // флаг очистки экрана, если true - надо очистить
byte lcdBackLightPin = 5;                // пин управления подсветкой экрана
byte lcdBrightness = 50;                 // яркость lcd дисплея в процентах 0-100%
byte setDistance = 10;                   // переменная для настройки расстояния до препятствия
byte setMotorSpeed = 50;                 // переменная для настройки мощности мотора 0-100% - регулируется из меню робота
byte startValuePwmMotor = 50;            // начальная мощность моторов для П-регулятора
float koefProp = 0.1;                    // коэффициент пропорциональности для П-регулятора
bool robotStartFlag = false;             // флаг запуска какого-либо робота

byte sensorLabels[] = {0b01111111, 0b01011110, 0b01111110};   // значки для указания датчика - левый/центр/правый
int lineSensorNum = 0;                                        // переменная для выбора датчика в режиме "Linetracer 1"
//----------------------- описание меню ----------------------------------------------
String mainMenuItems[] = {"Left turn", "Right turn", "Circle turn", \
                          "Forward(sec)", "Linetracer 1", "Linetracer 2", \
                          "Linetracer 3", "Around wall", "Bluetooth robot", \
                          "Cal.lineSensor", "Set M speed", "Set distance", "Set brightness", \
                          "Set k filter"
                         };                                                       // пункты основного меню
byte mainMenuSize = sizeof(mainMenuItems) / sizeof(String) - 1;                   // размер (кол-во строк) основного меню (-1 так как счет с 0)
bool mainMenuFlag = true;                                                         // флаг основного меню
bool lcdDrawFlag = true;                                                          // флаг отрисовки дисплея

unsigned int refreshLcdTime = 500;                                                // период обновления дисплея
unsigned int lastRefreshLcd = 0;                                                  // время предыдущего обновления
int mainMenuCounter = 0;                                                          // счетчик позиции основного меню
// ------------------------- Подменю Calibration lineSensor --------------------------
String lineSensorMenuItems[] = {"Auto mode", "Manual mode", "Delay"};             // пункты меню Calibration lineSensor
byte lineSensorMenuSize = sizeof(lineSensorMenuItems) / sizeof(String) - 1;       // размер (кол-во строк) меню Calibration lineSensor (-1 так как счет с 0)
// counter = 0 -> auto mode, counter = 1 -> (manual mode)
int lineSensorMenuCounter = 0;                                                    // счетчик позиций подменю "Calibration lineSensor"
//настройку уровней белого и черного будем производить по левому сенсору
unsigned long whiteValue = 900;                                                   // уровень отраженного света от белой поверхности
unsigned long blackValue = 125;                                                   // уровень отраженного света от черной поверхности
byte lineSensorOffset = 25;                                                       // добавка к уровням белого и черного
long lineSensorThresholdValue = 0;                                                // порог принятия решения белый/черный для движения по линии
byte minlineSensorThresholdValue = 0;                                             // нижняя граница диапазона "серого"
byte maxlineSensorThresholdValue = 0;                                             // верхняя граница диапазона "серого"
byte thresholdPwmValue = 75;                                                      // пороговое значение ШИМ сигнала для пропорционального регулятора с одним датчиком
bool manualModeLineSensorMenuFlag = false;                                        // флаг ручной настройки порога белый/черный
bool delayBetweenActionLineSensorFlag = false;                                    // флаг настройки интервала опроса датчиков
byte lineSensorMask = 0b00000000;                                                 // маска значений датчиков линии

// флаги для всех подпунктов меню корневого уровня
bool leftTurnMenuFlag = false;
bool rightTurnMenuFlag = false;
bool circleTurnMenuFlag = false;
bool forwardMenuFlag = false;
bool line1MenuFlag = false;
bool line2MenuFlag = false;
bool line3MenuFlag = false;
bool wallMenuFlag = false;
bool bluetoothMenuFlag = false;
bool setLineSensorMenuFlag = false;
bool setSpeedMenuFlag = false;
bool setDistMenuFlag = false;
bool setBrightnessMenuFlag = false;         // флаг меню установки яркости дисплея
bool setKFilterMenuFlag = false;
//----------------------- описание меню -----------------------------------------------
//----------------------- заставки дисплея --------------------------------------------
int stringSnow[16];                         // массив снежинок
int SNOW_SPEED = 150;                       // скорость снегопада для заставки
unsigned long lastActionMillis = 0;         // время последнего действия для запуска заставки
//----------------------- заставки дисплея --------------------------------------------

void setup() {
  pinMode(butCentrePin, INPUT);
  pinMode(butLeftPin, INPUT);
  pinMode(butRightPin, INPUT);
  pinMode(lcdBackLightPin, OUTPUT);

  Serial.begin(115200);
  lcd.init();
  lcdChars();
  lcd.backlight();
  analogWrite(lcdBackLightPin, map(lcdBrightness, 0, 100, 0, 255));
  lcd.setCursor(2, 0);
  lcd.print("lcd1602 Menu");
  lcd.setCursor(10, 1);
  lcd.print("v0.1");
  attachInterrupt(0, butLeftRead, FALLING);
  attachInterrupt(1, butRightRead, FALLING);
  attachPCINT(digitalPinToPCINT(butCentrePin), butCentreRead, CHANGE);
  delay(2000);
  // заполняем массив снежинок случайным образом
  for (int i = 0; i < sizeof(stringSnow) / sizeof(int); i++) {
    stringSnow[i] = random(0, 16);
  }
}


void loop() {
  while (1) {
    //------------------ заставка -----------------------------------
    if (millis() - lastActionMillis > 10000) {
      screenSaver();
    }
    //------------------ заставка -----------------------------------
    // отрисовка основного меню
    if (mainMenuFlag and lcdDrawFlag) {
      mainMenuDraw();
      lcdDrawFlag = false;
    }

    // кнопки лево/право = движение вверх/вних по меню или настройка параметра меньше/больше
    //------------ КНОПКА ЛЕВО/МЕНЬШЕ/ВНИЗ -------------------------------
    if (butLeftIsPress) {
      lastActionMillis = millis();
      butLeftIsPress = false;
      // действия в основном меню
      if (mainMenuFlag) {
        lcdDrawFlag = true;
        if (mainMenuCounter == 0) {
          mainMenuCounter = mainMenuSize;
        } else {
          mainMenuCounter--;
        }
      }

      // действия в меню "Calibration lineSensor"
      if (setLineSensorMenuFlag and !manualModeLineSensorMenuFlag and !delayBetweenActionLineSensorFlag) {
        if (lineSensorMenuCounter == 0) {
          lineSensorMenuCounter = lineSensorMenuSize;
        } else {
          lineSensorMenuCounter--;
        }
        lcdDrawFlag = true;
      }

      // действия в подменю "Calibration lineSensor: Manual mode"
      if (setLineSensorMenuFlag and manualModeLineSensorMenuFlag) {
        lineSensorThresholdValue -= 5;
        lcdDrawFlag = true;
      }

      // действия в подменю "Calibration lineSensor: Delay"
      if (setLineSensorMenuFlag and delayBetweenActionLineSensorFlag) {
        delayBetweenActionLineSensor -= 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Установка яркости дисплея"
      if (setBrightnessMenuFlag) {
        lcdBrightness -= 5;
        lcdBrightness = constrain(lcdBrightness, 0, 100);
        analogWrite(lcdBackLightPin, map(lcdBrightness, 0, 100, 0, 255));
        lcdDrawFlag = true;
      }

      // действия в меню "Настройка скорости моторов"
      if (setSpeedMenuFlag) {
        setMotorSpeed -= 5;
        setMotorSpeed = constrain(setMotorSpeed, 0, 100);
        lcdDrawFlag = true;
      }

      // действия в меню "Настройка расстояния до препятствий"
      if (setDistMenuFlag) {
        setDistance -= 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Forward (sec)"
      if (forwardMenuFlag) {
        setForwardSec -= 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Set k filter"
      if (setKFilterMenuFlag) {
        k -= 0.01;
        lcdDrawFlag = true;
      }

      // действия в меню "Linetracer 1"
      if (line1MenuFlag) {
        lineSensorNum -= 1;
        lineSensorNum = constrain(lineSensorNum, 0, 2);
        lcdDrawFlag = true;
      }
    }

    //------------ КНОПКА ВПРАВО/БОЛЬШЕ/ВВЕРХ -------------------------------
    if (butRightIsPress) {
      lastActionMillis = millis();
      butRightIsPress = false;
      // действия в основном меню
      if (mainMenuFlag) {
        lcdDrawFlag = true;
        if (mainMenuCounter == mainMenuSize) {
          mainMenuCounter = 0;
        } else {
          mainMenuCounter++;
        }
      }

      // действия в меню "Calibration lineSensor"
      if (setLineSensorMenuFlag and !manualModeLineSensorMenuFlag and !delayBetweenActionLineSensorFlag) {
        if (lineSensorMenuCounter == lineSensorMenuSize) {
          lineSensorMenuCounter = 0;
        } else {
          lineSensorMenuCounter++;
        }
        lcdDrawFlag = true;
      }

      // действия в подменю "Calibration lineSensor: Manual mode"
      if (setLineSensorMenuFlag and manualModeLineSensorMenuFlag) {
        lineSensorThresholdValue += 5;
        lcdDrawFlag = true;
      }

      // действия в подменю "Calibration lineSensor: Delay"
      if (setLineSensorMenuFlag and delayBetweenActionLineSensorFlag) {
        delayBetweenActionLineSensor += 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Установка яркости дисплея"
      if (setBrightnessMenuFlag) {
        lcdBrightness += 5;
        lcdBrightness = constrain(lcdBrightness, 0, 100);
        analogWrite(lcdBackLightPin, map(lcdBrightness, 0, 100, 0, 255));
        lcdDrawFlag = true;
      }

      // действия в меню "Настройка скорости моторов"
      if (setSpeedMenuFlag) {
        setMotorSpeed += 5;
        setMotorSpeed = constrain(setMotorSpeed, 0, 100);
        lcdDrawFlag = true;
      }

      // действия в меню "Настройка расстояния до препятствий"
      if (setDistMenuFlag) {
        setDistance += 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Forward (sec)"
      if (forwardMenuFlag) {
        setForwardSec += 1;
        lcdDrawFlag = true;
      }

      // действия в меню "Set k filter"
      if (setKFilterMenuFlag) {
        k += 0.01;
        lcdDrawFlag = true;
      }

      // действия в меню "Linetracer 1"
      if (line1MenuFlag) {
        lineSensorNum += 1;
        lineSensorNum = constrain(lineSensorNum, 0, 2);
        lcdDrawFlag = true;
      }
    }

    //------------------------ SHORT PRESS CENTRAL BUTTON -------------------------------------
    if (butIsShort) {
      lastActionMillis = millis();
      // действия основного меню
      if (mainMenuFlag) {
        takeSubMenu();
        mainMenuFlag = false;
        lcdDrawFlag = true;
        goto END;
      }

      // действия в меню "Calibration lineSensor"
      if (setLineSensorMenuFlag and !manualModeLineSensorMenuFlag) {
        switch (lineSensorMenuCounter) {
          case 0:
            autoModeLineSensor();
            break;
          case 1:
            manualModeLineSensorMenuFlag = true;
            //manualModeLineSensor();
            break;
          case 2:
            delayBetweenActionLineSensorFlag = true;
            //delayBetweenAction();
            break;
        }
        lcdDrawFlag = true;
        goto END;
      }

      // действия в подменю "Calibration lineSensor: Manual mode"
      if (setLineSensorMenuFlag and manualModeLineSensorMenuFlag) {
        saveSettings(addressLineSensorThresholdValue, lineSensorThresholdValue);
        lcdDrawFlag = true;
        goto END;
      }

      // действия в меню "Настройки яркости дисплея"
      if (setBrightnessMenuFlag) {
        saveSettings(addressBrightness, lcdBrightness);
        lcdDrawFlag = true;
        goto END;
      }

      // действия в меню "Настройка скорости моторов"
      if (setSpeedMenuFlag) {
        saveSettings(addressMotorSpeed, setMotorSpeed);
        lcdDrawFlag = true;
        goto END;
      }

      // действия в меню "Настройка расстояния до препятствий"
      if (setDistMenuFlag) {
        saveSettings(addressDistance, setDistance);
        lcdDrawFlag = true;
        goto END;
      }

      // действия в меня "Set k filter"
      if (setKFilterMenuFlag) {
        saveSettings(addressFilter, k);
        lcdDrawFlag = true;
        goto END;
      }

      if (leftTurnMenuFlag or rightTurnMenuFlag or circleTurnMenuFlag or forwardMenuFlag or line1MenuFlag or line2MenuFlag or line3MenuFlag or wallMenuFlag or bluetoothMenuFlag) {
        lcd.setCursor(0, 1);
        lcd.print("    working     ");
        delay(2000);                    // задержка, чтобы отойти от робота
        startForwardRobot = millis();
        robotStartFlag = true;
        lcdDrawFlag = false;
        goto END;
      }
END:
      butIsShort = false;
    }
    //------------------------ SHORT PRESS CENTRAL BUTTON -------------------------------------

    // отрисовка общего меню для элементов Left, Right, Circle, LinetracerN, Around wall and Bluetooth robot
    if ((leftTurnMenuFlag or rightTurnMenuFlag or circleTurnMenuFlag or line2MenuFlag or line3MenuFlag or wallMenuFlag or bluetoothMenuFlag) and lcdDrawFlag) {
      commonMenu();
      lcdDrawFlag = false;
    }

    // запуск определенной программы робота
    if (robotStartFlag) {
      if (leftTurnMenuFlag) {
        leftTurnRobot();
      }

      if (rightTurnMenuFlag) {
        rightTurnRobot();
      }

      if (circleTurnMenuFlag) {
        circleTurnRobot();
      }

      if (forwardMenuFlag) {
        forwardRobot();
      }

      if (line1MenuFlag) {
        //line1Robot();
        //line1RobotProp();
        line1RobotProp1();
      }

      if (line2MenuFlag) {
        line2Robot();
      }

      if (line3MenuFlag) {
        line3Robot();
      }

      if (wallMenuFlag) {
        wallRobot();
      }

      if (bluetoothMenuFlag) {
        bluetoothRobot();
      }
    }

    // отрисовка меню "Linetracer 1"
    if (line1MenuFlag and lcdDrawFlag) {
      line1Menu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Calibration lineSensor"
    if (setLineSensorMenuFlag and lcdDrawFlag and !manualModeLineSensorMenuFlag and !delayBetweenActionLineSensorFlag) {
      setLineSensorMenu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Calibration lineSensor: Manual mode"
    if (setLineSensorMenuFlag and lcdDrawFlag and manualModeLineSensorMenuFlag) {
      manualModeLineSensor();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Calibration lineSensor: Delay"
    if (setLineSensorMenuFlag and lcdDrawFlag and delayBetweenActionLineSensorFlag) {
      delayBetweenAction();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Настройка яркости дисплея"
    if (setBrightnessMenuFlag and lcdDrawFlag) {
      setBrightnessMenu();
      lcdDrawFlag = false;
    }

    // отрисовка меню "Настройка скорости моторов"
    if (setSpeedMenuFlag and lcdDrawFlag) {
      setSpeedMenu();
      lcdDrawFlag = false;
    }

    //отрисовка меню "Настройка расстояния до препятствий"
    if (setDistMenuFlag and lcdDrawFlag) {
      setDistMenu();
      lcdDrawFlag = false;
    }

    //отрисовка меню "Настройка времени движения вперед"
    if (forwardMenuFlag and lcdDrawFlag) {
      forwardMenu();
      lcdDrawFlag = false;
    }

    //отрисовка меню "Настройка коэффициента фильтрации"
    if (setKFilterMenuFlag and lcdDrawFlag) {
      kFilterMenu();
      lcdDrawFlag = false;
    }

    //------------------------ LONG PRESS CENTRAL BUTTON -------------------------------------
    if (butIsLong) {
      lastActionMillis = millis();  // обновляем таймер действия, чтобы не уйти в заставку
      butIsLong = false;
      robotStartFlag = false;       // выход в основное меню = остановка работы робота (надо не забыть остановить все моторы)
      lcdDrawFlag = true;
      mainMenuFlag = true;
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      manualModeLineSensorMenuFlag = false;
      delayBetweenActionLineSensorFlag = false;
      analogWrite(leftMotorPwmPin, 0);
      analogWrite(rightMotorPwmPin, 0);
    }
    //------------------------ LONG PRESS CENTRAL BUTTON -------------------------------------
  }
}


// функция обработки нажатий центральной кнопки
void butCentreRead() {
  bool butState = !digitalRead(butCentrePin);

  // кнопка нажата и была нажата = удерживаем = "count time"
  if (butState) {
    startButPressTimer = millis();
  }

  // кнопка не нажата и была не нажата = не нажимали = "update startTimer"
  if (!butState) {
    butPressTimer = millis() - startButPressTimer;
    if (butPressTimer < 500) {
      butIsShort = true;
      butIsLong = false;
    }
    if (butPressTimer > butCentreHoldTime) {
      butIsShort = false;
      butIsLong = true;
    }
  }

}

// функция обработки левой кнопки
void butLeftRead() {
  if (!butLeftIsPress) {
    butLeftIsPress = true;
  }
}

// функция обработки правой кнопки
void butRightRead() {
  if (!butRightIsPress) {
    butRightIsPress = true;
  }
}

// функция отрисовки элементов основного меню
void mainMenuDraw() {
  lcd.clear();
  lcd.setCursor(0, 0);
  //lcd.printByte(0);                         // если указатель меню в виде кастомного символа
  lcd.printByte(0b01111110);
  byte nextItem = 0;
  if (mainMenuCounter == mainMenuSize) {
    nextItem = 0;
  } else {
    nextItem = mainMenuCounter + 1;
  }
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print(mainMenuItems[nextItem]);
}

// функция автоматической настройки порога белый/черный
void autoModeLineSensor() {
  
  lastActionMillis = millis();
  setLineSensorMenuFlag = true;
  lcdDrawFlag = true;
}

// функция ручной настройки порога белый/черный
void manualModeLineSensor() {
  
}

/*
    функция "Left turn", "Right turn", "Circle turn", "Linetracer1"
    "Linetracer2", "Linetracer3", "Around wall", "Bluetooth robot"
*/
void commonMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
}

// робот "Левый поворот"
void leftTurnRobot() {
  
}

// робот "Правый поворот"
void rightTurnRobot() {
  
}

// робот "Разворот"
void circleTurnRobot() {
  
}

// робот "Движение вперед по времени"
void forwardRobot() {
  
}

// робот "Линия с 1 датчиком - Релейный регулятор"
void line1RobotRelay() {

}

// робот "Линия с 2 датчиками"
void line2Robot() {
  
}

// робот "Линия с 3 датчиками"
void line3Robot() {
  
  }
}

// робот "Объезд препятствий"
void wallRobot() {

}

// робот "Робот с управлением по Bluetooth"
void bluetoothRobot() {

}

// функция "Forward"
void forwardMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Seconds");
  lcd.setCursor(13, 1);
  lcd.print(setForwardSec);
}

// функция "Linetracer 1"
void line1Menu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  switch (lineSensorNum) {
    case 0:
      lcd.setCursor(1, 1);
      break;
    case 1:
      lcd.setCursor(8, 1);
      break;
    case 2:
      lcd.setCursor(15, 1);
      break;
  }
  lcd.printByte(sensorLabels[lineSensorNum]);
}

// функция отрисовки "Настройка порога белый/черный для датчиков линии"
void setLineSensorMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print(lineSensorMenuItems[lineSensorMenuCounter]);
}

void delayBetweenAction() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print(lineSensorMenuItems[lineSensorMenuCounter]);
  lcd.setCursor(12, 1);
  lcd.print(delayBetweenActionLineSensor);
}

// функция отрисовки "Настройка яркости дисплея"
void setBrightnessMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Brightness");
  lcd.setCursor(12, 1);
  lcd.print(lcdBrightness);
  lcd.setCursor(15, 1);
  lcd.print("%");
}

// функция отрисовки "Настройка скорости моторов"
void setSpeedMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Speed");
  lcd.setCursor(12, 1);
  lcd.print(setMotorSpeed);
  lcd.setCursor(15, 1);
  lcd.print("%");
}

// функция отрисовки "Настройка расстояния до препятствий"
void setDistMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(1, 1);
  lcd.print("Distance");
  lcd.setCursor(13, 1);
  lcd.print(setDistance);
}

// функция "Set k filter"
void kFilterMenu() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(mainMenuItems[mainMenuCounter]);
  lcd.setCursor(5, 1);
  lcd.print("k:");
  lcd.setCursor(7, 1);
  lcd.print(k);
}

// функция отрисовки "Сохранения настроек"
// передаем в функцию адрес в памяти EEPROM и значение которое нужно записать
void saveSettings(byte address, float value) {
  EEPROM.writeFloat(address, value);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Saving settings");
  for (int i = 0; i < 16; i++) {
    lcd.setCursor(i, 1);
    lcd.print(">");
    delay(125);
  }
}

// функция заставки
void screenSaver() {
  // отрисовываем снегопад
  for (int x = 0; x < sizeof(stringSnow) / sizeof(int); x++) {
    int num = 0;
    if (stringSnow[x] <= 7) {
      lcd.setCursor(x, 1);
      lcd.print(" ");
      num = stringSnow[x];
      lcd.setCursor(x, 0);
    } else {
      num = stringSnow[x] - 8;
      lcd.setCursor(x, 0);
      lcd.print(" ");
      lcd.setCursor(x, 1);
    }
    lcd.printByte(num);
  }
  // сдвигаем снегопад вниз, если элемент вышел за границы, то генерируем новую снежинку случайно
  for (int i = 0; i < sizeof(stringSnow) / sizeof(int); i++) {
    int newValue = stringSnow[i] + 1;
    if (newValue > 15) {
      stringSnow[i] = random(0, 8);
    } else {
      stringSnow[i] = newValue;
    }
  }
  delay(SNOW_SPEED);
}

// создание кастомных символов для дисплея
void lcdChars() {
  lcd.createChar(0, s1);
  lcd.createChar(1, s2);
  lcd.createChar(2, s3);
  lcd.createChar(3, s4);
  lcd.createChar(4, s5);
  lcd.createChar(5, s6);
  lcd.createChar(6, s7);
  lcd.createChar(7, s8);
}

// функция выбора подменю
void takeSubMenu() {
  switch (mainMenuCounter) {
    case 0:
      leftTurnMenuFlag = true;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 1:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = true;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 2:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = true;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 3:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = true;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 4:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = true;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 5:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = true;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 6:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = true;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 7:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = true;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 8:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = true;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 9:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = true;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 10:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = true;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 11:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = true;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = false;
      break;
    case 12:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = true;
      setKFilterMenuFlag = false;
      break;

    case 13:
      leftTurnMenuFlag = false;
      rightTurnMenuFlag = false;
      circleTurnMenuFlag = false;
      forwardMenuFlag = false;
      line1MenuFlag = false;
      line2MenuFlag = false;
      line3MenuFlag = false;
      wallMenuFlag = false;
      bluetoothMenuFlag = false;
      setLineSensorMenuFlag = false;
      setSpeedMenuFlag = false;
      setDistMenuFlag = false;
      setBrightnessMenuFlag = false;
      setKFilterMenuFlag = true;
      break;
  }
}

