// This #include statement was automatically added by the Particle IDE.
#include <SparkFunLSM9DS1.h>

// This #include statement was automatically added by the Particle IDE.
#include "IOT-ECOSYS_LSM9DS1_Acc_Gyr_Mag.h"
#include "IoT-Eco-NeoPIXEL-Ring.h"

/*
*  This file is a sample application, based
*  on the IoT Prototyping Framework (IoTPF)

This application and IoTPF is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

IoTPF is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU v3 General Public License for more details.

Released under GNU v3

You should have received a copy of the GNU General Public License
along with IoTPF.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
* This is a sample demonstrates Particle Photon with MAX4466, LSMD9S1, SEN09088, DHT11.
*
* Needed components
* Mic MAX4466 (MAX)
* Nine degrees of freedom sensor LSMD9S1 (LSM)
* Mini Photocell  SEN09088 (LIG)
* Humidity & Temperature Sensor DHT11 (DHT)
* Particle Photon (PHO)
*
* Wiring
* PHO (A0) - MAX (OUT)
* PHO (GND) - MAX (GND)
* PHO (3,3) - MAX (VCC)
*
* PHO (D0) - LSM (SDA)
* PHO (D1) - LSM (SCL)
* PHO (GND) - LSM (GND)
* PHO (3V3) - LSM (3V3)
*
* PHO (A1) - LIG (GND)
* RES (POSITIVE) - LIG (GND)
* PHO (GND) - RES (NEGATIVE)
* PHO (VIN) - LIG (VCC)
*
* PHO (D2) - DHT (DATA)
* PHO (GND) - DHT (GND)
* PHO (VIN) - DHT (+5V)
*
* @author Dionysios Satikidis (dionysios.satikidis@gmail.com)
* @version 1.0
*/


// This #include statement was automatically added by the Particle IDE.
#include "IOT-ECOSYS_LSM9DS1_Acc_Gyr_Mag.h"

#define CALIBRATION_DURATION	4000


// ditital pins D0 and D1 for SPI - see .cpp & .h for details. Debug  = true
IoTEcoSys_LSM9DS1_Acc_Gyr_Mag lsm(true);

uint avgMagMinAvg[] = {0,0,0};
uint avgMagMaxAvg[] = {0,0,0};

uint differenz[] = {0,0,0};

short highestDifferenz = -1;


uint avgMag[] = {0,0,0};



uint lastValue[3];

uint delayTime = 1;
uint publishDelay = 1000;
uint loopsToPublish = (uint) (publishDelay / delayTime);
uint avgFactor = 50;


unsigned long rpm = 0;


uint mag[3];
uint last[3];
// 0 --> No status until now -1 --> going down 1 --> going up
int status[3] = {0,0,0};
uint highestValue[3];
uint lowestValue[3];

//Init: -1, going up: 0, is in maxima: 1, going down: 2, is in minima: 3.
short rotationStatus[] = {-1,-1,-1};

float rotationPerMinute[] = {0.0,0.0,0.0};
unsigned long lastRotation[] = {0,0,0};
unsigned long lastRotationMain = 0;
int finished = false;

int startedWhen = 0;

int button = D4;

int calibrationCycles = 0;

unsigned long lastZeroCheck = 0;

bool leds = true;

bool isTurnedOff = false;

String deviceName = "Washinator";


IoTEcoSys_NeoPIXEL_Ring ring(16);

void setup() {

  lsm.init();
  ring.init();

  Particle.variable("rpm", rpm);
  Particle.variable("finished", finished);
  Particle.variable("calibratevar", calibrationCycles);
  Particle.variable("devicename", deviceName);
  Particle.function("calibrate", apiCalibrate);
  Particle.function("setled", apiSetLed);
  Particle.function("setname", apiSetName);
  Serial.begin(9600);
  pinMode(button, INPUT);
}



void loop() {

  // Nine degrees of freedom sensor LSMD9S1 read sensors
  lsm.read();


  mag[0] = lsm.getMagRaw_X();
  mag[1] = lsm.getMagRaw_Y();
  mag[2] = lsm.getMagRaw_Z();

  if(!leds && !isTurnedOff){
    ring.neoRingClockWise(1, 1, "off");
    isTurnedOff = true;
  }

  /*Calibration*/

  if(digitalRead(button) == HIGH){
    Serial.printlnf("Button");
    calibrationCycles = CALIBRATION_DURATION;
      if(leds){
    ring.neoRingFillClockWise(0, 50, "white");
}
  }
  if(leds){
ring.neoRingFillPercentage((float) calibrationCycles  / (float) CALIBRATION_DURATION, "white");
}
  if(calibrationCycles){
    if(calibrationCycles % 300 == 0){
      Serial.printlnf("calibrationCycles: %d", calibrationCycles);

    }
    if(calibrationCycles == CALIBRATION_DURATION){
      resetCalibration();
    }


    calculateAvg();

    for(int i=0; i<3; i++){
      //Wechsel nach oben
      if(last[i] <= avgMag[i] && mag[i] > avgMag[i]){
        calculateMinAvg(i);
        highestValue[i] = mag[i];
      }
      //Wechsel nach unten
      if(last[i] >= avgMag[i] && mag[i] < avgMag[i]){
        calculateMaxAvg(i);
        lowestValue[i] = mag[i];
      }

      if (highestValue[i] < mag[i]){
        highestValue[i] = mag[i];
      }
      if (lowestValue[i] > mag[i]){
        lowestValue[i] = mag[i];
      }


      differenz[i] = avgMagMaxAvg[i] - avgMagMinAvg[i];

    }

    if(differenz[0] >= differenz[1] && differenz[0] >= differenz[2]){
      highestDifferenz = 0;
    }else if(differenz[1] >= differenz[0] && differenz[1] >= differenz[2]){
      highestDifferenz = 1;
    }else if(differenz[2] >= differenz[0] && differenz[2] >= differenz[1]){
      highestDifferenz = 2;
    }
    //Substract One Cycle
    calibrationCycles--;

  }
  /*Calibration End*/
  else{

  /*Rotation Recognition */

  for(int i=0; i<3; i++){
    //Wechsel nach oben
    if(last[i] <= avgMag[i] && mag[i] > avgMag[i]){
      status[i] = 1;
      changeStatus(0,i);
    }
    //Wechsel nach unten
    if(last[i] >= avgMag[i] && mag[i] < avgMag[i]){
      status[i] = -1;
      changeStatus(2,i);
    }

    if (mag[i] + differenz[i] * 0.3 > avgMagMaxAvg[i]){
      changeStatus(1,i);
    }
    if (mag[i] - differenz[i] * 0.3  < avgMagMinAvg[i]){
      changeStatus(3,i);
    }

  }

  /*Rotation Recognition End*/
}



  /* Publish */

  if(loopsToPublish <= 0){
    loopsToPublish = (uint) (publishDelay / delayTime);


    Serial.printlnf("avg x: %d, min: %d, max: %d, dif: %d\navg y: %d, min: %d, max: %d dif: %d\navg z: %d, min: %d, max: %d dif: %d\n", avgMag[0], avgMagMinAvg[0], avgMagMaxAvg[0], differenz[0], avgMag[1], avgMagMinAvg[1], avgMagMaxAvg[1], differenz[1], avgMag[2], avgMagMinAvg[2], avgMagMaxAvg[2], differenz[2]);

    //Particle.publish(String("mag_sensor"), String(mag[0]) +":"+ String(mag[1]) +":"+ String(mag[2]));
    //Particle.publish(String("avg"), String(avgMag[0]) + ":" + String(avgMag[1]) + ":" + String(avgMag[2]));
    //Particle.publish(String("maxAvg"), String(avgMagMaxAvg[0]) + ":" + String(avgMagMaxAvg[1]) + ":" + String(avgMagMaxAvg[2]));
    //Particle.publish(String("minAvg"), String(avgMagMinAvg[0]) + ":" + String(avgMagMinAvg[1]) + ":" + String(avgMagMinAvg[2]));

  }
  else{
    loopsToPublish--;
  }

  /* Publish End*/
  for(int i=0; i<3; i++){
    last[i] = mag[i];
  }

  if(finished == false && !calibrationCycles){
      if(leds){
  ring.neoRingUsingNoDelayCounterClockWise(rpm, "blue");
}
  setRotationToZero();
  }


  delay(delayTime);
}

void changeStatus(short status, short direction){
  /*if((rotationStatus[direction] == 0 || rotationStatus[direction] == -1) && status == 2){
    rotationStatus[direction] = status;
  }
  if(rotationStatus[direction] == 2 && status == 0){
    rotationStatus[direction] = status;
    calculateRotationPerMinute(direction);
  }*/

  if(rotationStatus[direction] + 1 == status){
    rotationStatus[direction] = status;
  }
  else if(rotationStatus[direction] == 3 && status == 0){
    rotationStatus[direction] = status;
    calculateRotationPerMinute(direction);

  }

}

void setRpmVariable(){
    rpm = rotationPerMinute[highestDifferenz];
    lastRotationMain = lastRotation[highestDifferenz];
}


void setRotationToZero(){
  unsigned long now = millis();
  if(now - lastZeroCheck < 500){
    return;
  }

  if(rpm == 0 && (lastRotationMain + 60000) < now && finished == false && (startedWhen + 10000) < now){
    Particle.publish("finished", "done", PRIVATE);
      if(leds){
    ring.neoRingFillClockWise(0, 50, "green");
  }
    finished = true;
  }



  for(int i=0; i < 3; i++){
    if((now - lastRotation[i]) / 1000.0 > 1){
      rotationPerMinute[i] = (2 * rotationPerMinute[i]) / 5;
      setRpmVariable();
      Serial.printlnf("toZero %d rpm: %.2f now: %lu lastRotation: %lu", i, rotationPerMinute[i], now , lastRotation[i]);
    }
      if((now - lastRotation[i]) / 1000.0 > 7){
        rotationPerMinute[i] = 0.0;
        setRpmVariable();
        Serial.printlnf("zero %d rpm: %.2f now: %lu lastRotation: %lu", i, rotationPerMinute[i], now , lastRotation[i]);
      }
  }
  lastZeroCheck = now;
}



void calculateRotationPerMinute(short direction){

    unsigned long now = millis();
    if(lastRotation[direction] > now){
      lastRotation[direction] = 0.0;
    }

    if((60.0 * 1000.0 / (float) (now - lastRotation[direction])) - rotationPerMinute[direction] > 50){
      rotationPerMinute[direction] = (10 * rotationPerMinute[direction] + (60.0 * 1000.0 / (float) (now - lastRotation[direction]))) / 11;
      Serial.printlnf("Value to heigh");
    }else{

    rotationPerMinute[direction] = (2 * rotationPerMinute[direction] + (60.0 * 1000.0 / (float) (now - lastRotation[direction]))) / 3;
    }
    setRpmVariable();
    if(direction == highestDifferenz){
      finished = false;
      startedWhen = millis();
    Serial.printlnf("rotated %d rpm: %.2f now: %lu lastRotation: %lu", direction, rotationPerMinute[direction], now , lastRotation[direction]);
    }


    lastRotation[direction] = now;

}

void calculateAvg(){

  if(avgMag[0] == 0){
    for(int i=0; i < 3; i++){
      avgMag[i] = mag[i];
    }
  }
  else{
    for(int i=0; i < 3; i++){
      if(avgMagMaxAvg[i] == 0 || avgMagMinAvg[i] == 0  ){
        avgMag[i] = (avgMag[i] * avgFactor + mag[i] * 1) / (avgFactor + 1);
      }
      else{
        avgMag[i] = (avgMagMaxAvg[i] + avgMagMinAvg[i]) / 2;
      }
    }
  }
}

void calculateMaxAvg(int i){

  if(avgMagMaxAvg[i] == 0){
    avgMagMaxAvg[i] = highestValue[i];
  }
  else{
    avgMagMaxAvg[i] = (avgMagMaxAvg[i] * avgFactor + highestValue[i] * 1) / (avgFactor + 1);
  }
}

void calculateMinAvg(int i){

  if(avgMagMinAvg[i] == 0){
    avgMagMinAvg[i] = lowestValue[i];
  }
  else{
    avgMagMinAvg[i] = (avgMagMinAvg[i] * avgFactor + lowestValue[i] * 1) / (avgFactor + 1);
  }
}

void resetCalibration(){
  for(int i=0; i<3; i++){
    avgMagMinAvg[i] = 0;
    avgMag[i] = 0;
    avgMagMaxAvg[i] = 0;
  }
}


int apiCalibrate(String extra) {

  calibrationCycles = CALIBRATION_DURATION;
  return 0;
}

int apiSetLed(String status){
  if(status == "on"){
    leds = true;
  }
  if(status == "off"){
    leds = false;
    isTurnedOff = false;
  }
}

int apiSetName(String name){
  deviceName = name;
}
