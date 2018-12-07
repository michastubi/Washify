// This #include statement was automatically added by the Particle IDE.
#include <SparkFunLSM9DS1.h>

// This #include statement was automatically added by the Particle IDE.
#include "IOT-ECOSYS_LSM9DS1_Acc_Gyr_Mag.h"

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



// ditital pins D0 and D1 for SPI - see .cpp & .h for details. Debug  = true
IoTEcoSys_LSM9DS1_Acc_Gyr_Mag lsm(true);

uint avgMagMinAvg[] = {0,0,0};
uint avgMagMaxAvg[] = {0,0,0};


uint avgMag[] = {0,0,0};



uint lastValue[3];

uint delayTime = 1;
uint publishDelay = 1000;
uint loopsToPublish = (uint) (publishDelay / delayTime);
uint avgFactor = 10000;

unsigned long nMeasure = 0;


uint mag[3];
uint last[3];
// 0 --> No status until now -1 --> going down 1 --> going up
int status[3] = {0,0,0};
uint highestValue[3];
uint lowestValue[3];




void setup() {

  lsm.init();
  Particle.variable("nMeasure", nMeasure);
  Serial.begin(9600);
}



void loop() {
  // Nine degrees of freedom sensor LSMD9S1 read sensors
  lsm.read();

  mag[0] = lsm.getMagRaw_X();
  mag[1] = lsm.getMagRaw_Y();
  mag[2] = lsm.getMagRaw_Z();

  nMeasure++;

  calculateAvg();

  for(int i=0; i<3; i++){
    //Wechsel nach oben
    if(last[i] <= avgMag[i] && mag[i] > avgMag[i]){
      status[i] = 1;
      calculateMinAvg(i);
      highestValue[i] = mag[i];
      Serial.printlnf("UP %d Last: %d Now: %d", i, last[i], mag[i]);
    }
    //Wechsel nach unten
    if(last[i] >= avgMag[i] && mag[i] < avgMag[i]){
      status[i] = -1;
      calculateMaxAvg(i);
      lowestValue[i] = mag[i];
        Serial.printlnf("DOWN %d Last: %d Now: %d", i, last[i], mag[i]);
    }



  }


  if(loopsToPublish <= 0){
    loopsToPublish = (uint) (publishDelay / delayTime);


    Serial.printlnf("avg %d, %d, %d", avgMag[0], avgMag[1], avgMag[2]);

    //Particle.publish(String("mag_sensor"), String(mag[0]) +":"+ String(mag[1]) +":"+ String(mag[2]));
    //Particle.publish(String("avg"), String(avgMag[0]) + ":" + String(avgMag[1]) + ":" + String(avgMag[2]));
    //Particle.publish(String("maxAvg"), String(avgMagMaxAvg[0]) + ":" + String(avgMagMaxAvg[1]) + ":" + String(avgMagMaxAvg[2]));
    //Particle.publish(String("minAvg"), String(avgMagMinAvg[0]) + ":" + String(avgMagMinAvg[1]) + ":" + String(avgMagMinAvg[2]));

  }
  else{
    loopsToPublish--;
  }

  for(int i=0; i<3; i++){
    last[i] = mag[i];
  }


  delay(delayTime);
}

void calculateAvg(){

  if(avgMag[0] == 0){
    for(int i=0; i < 3; i++){
      avgMag[i] = mag[i];
    }
  }
  else{
    for(int i=0; i < 3; i++){
      avgMag[i] = (avgMag[i] * avgFactor + mag[i] * 1) / (avgFactor + 1);
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
  Serial.printlnf("maxAvg %d, %d, %d", avgMagMaxAvg[0], avgMagMaxAvg[1], avgMagMaxAvg[2]);
}

void calculateMinAvg(int i){

  if(avgMagMinAvg[i] == 0){
    avgMagMinAvg[i] = lowestValue[i];
  }
  else{
    avgMagMinAvg[i] = (avgMagMinAvg[i] * avgFactor + lowestValue[i] * 1) / (avgFactor + 1);
  }
  Serial.printlnf("minAvg %d, %d, %d", avgMagMinAvg[0], avgMagMinAvg[1], avgMagMinAvg[2]);
}
