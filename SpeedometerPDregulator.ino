// Importerer biblioteker
#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4Encoders.h>

// Instanser av det vi skal bruke
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
Zumo32U4LCD display;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];
const uint16_t maxSpeed = 400;
int16_t lastError = 0;

// speedometer
unsigned long lastTimeBeforeCount;

void calibrateSensors()
{
  display.clear();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for (uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

float speedometer() {
  float timeAfterCount = micros() - lastTimeBeforeCount;
  float leftcount = encoders.getCountsAndResetLeft();
  float rightcount = encoders.getCountsAndResetRight();
  float counts = (leftcount + rightcount) / 2;
  float lengthInMilliMeters = counts / 909.7 * 122.52211349;
  float verdi = (lengthInMilliMeters / timeAfterCount) * 100000;
  lastTimeBeforeCount = micros();
  return verdi;
}

//Visning av sensorverdier på LCD skjerm
void displayLCD () {
  lcd.clear();
  lcd.gotoXY(2, 0);
  lcd.print(lineSensorValues[1] % 100);
  lcd.gotoXY(0, 1);
  lcd.print(lineSensorValues[0] % 100);
  lcd.gotoXY(6, 1);
  lcd.print(lineSensorValues[4] % 100);
  lcd.gotoXY(5, 0);
  lcd.print(lineSensorValues[2] % 100);
  lcd.gotoXY(3, 1);
  lcd.print(lineSensorValues[3] % 100);
}

void setup()
{
  Serial.begin(9600);
  // Initialiserer 5-linjefølgersensor
  lineSensors.initFiveSensors();

  // Wait for button A to be pressed and released.
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  buttonA.waitForButton();

  // Kalibrerer linjefølgersensor
  calibrateSensors();

  // Play music and wait for it to finish before we start driving.
  display.clear();
  display.print(F("Go!"));
}

void loop()
{
  // Les linjefølgersensor
  int16_t position = lineSensors.readLine(lineSensorValues);

  // Speedometer
  lcd.gotoXY(0, 1);
  lcd.print(speedometer());
  
  // Finn avvik, og lagre i en ny variabel kalt error
  int16_t error = position - 2000;

  // Opprett en ny variabel, med et navn som «speed_difference»
  float Kp = 1;   // tuning konstant, P-leddet
  float Td = 0.3 * (error - lastError); // tuning konstant, D-delen
  int16_t speedDifference = (error * Kp) + Td;
  lastError = error;

  // Opprett to nye variabler, kalt noe som left_speed og right_speed.
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  // constrain(x, bunnverdi, toppverdi), tvinger verdien til å være 0-400
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

  // Setter motorhastigheter
  motors.setSpeeds(leftSpeed, rightSpeed);
}

/*
  KP TEST
  1-10:   test 6 - skarpe svinger, resten wiggly
        test 3 - wiggly
  10-25:  test 13 - verre enne uten Kp, mer wiggly,
        test 20 - greit rett, men saktere pga mer korrigereing / kjørte ikke i svinger
  0.5-1:  test 0.85 - drit bra på rett linje og iffy svinger
  0.2-0.5: test 0.3 - drit bra i svak sving, grei rett og iffy krapp sving

  Best krappe svinger - 1<
  Best slake svinger - 0,2-0,5
  Best rett strek - 0,5-1

  TD TEST
  1-10:   test 2 - god på rette strekninger, unøyaktig i svinger
  10-25:  test 19 - veldig mange små tilpasninger på rett strekning, unøyaktig i svinger, bommer på noen
  0.5-1:  test 0.7 - god på rette strekninger, litt unøyaktig i svinger
  0.2-0.5: test 0.3 - god på rette strekninger, bedre i svinger
*/
