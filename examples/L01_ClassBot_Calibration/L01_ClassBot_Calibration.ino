/*
 * L01_ClassBot_Calibration.ino
 * 
 * Author: Andrew Gafford
 * email: agafford@spacetrek.com
 * Date: Feb. 25th, 2023
 * 
 * This is an example program to calibrate the motor power settings and 
 * encoder pulses per distance for the Space Trek Class Bot 2.0.
 */

#include <SpaceTrek_ClassBot2.h>                //include the classbot2 library so we have the commands to use the robot

const uint32_t displayTime = 1000;              //how often in ms to send the motor power levels to the serial port
uint32_t displayTimer = 0;                      //a variable to store processor clock count used to control the display timer

void setup() {                                  //the setup() funtion runs once when the program starts
  Serial.begin(115200);                         //Start the serial connection to the PC
  delay(500);                                   //wait 0.5 seconds to allow serial to connect

  classBot.begin();                             //start the classbot object.  Sets pinModes and sets up sensors 

//-------------------------------------- Drive Setup --------------------------------------
  //Power Settings
  classBot.setPowerForward(255, 255);           //set the power level for the motors when moving forward (left side, right side)
  classBot.setPowerReverse(255, 255);           //set the power level for the motors when moving in reverse (left side, right side)
  classBot.setPowerPivotRight(255, 255);        //set the power level for the motors when pivoting right
  classBot.setPowerPivotLeft(255, 255);         //set the power level for the motors when pivoting left
  classBot.setBiasForward(0);                   //set the forward bias. Negative goes more left, positive goes more right
  classBot.setBiasReverse(0);                   //set the reverse bias. Negative goes more left, positive goes more right

  //calibration settings
  classBot.setMeterDistanceForward(1500);       //set how many encoder pulses are needed to drive forward 1 meter
  classBot.setMeterDistanceReverse(1500);       //set how many encoder pulses are needed to drive reverse 1 meter
  classBot.setPivotRight90(275);                //set how many encoder pulses are needed to pivot right 90 degrees
  classBot.setPivotLeft90(275);                 //set how many encoder pulses are needed to pivot left 90 degrees

  // classBot.setSpeed(75);                        //set the speed for forward and reverse as percent of power

  delay(1000);                                  //wait for 1 second so the robot doesn't move as soon as you turn it on.
//-----------------------------------------------------------------------------------------



//----------------------------------- Power Calibration -----------------------------------
  //use these four commands to get the left and right power values for
  //each of the four movment types of the robot.

  // classBot.goForward();
  // classBot.goReverse();
  // classBot.goPivotRight();
  // classBot.goPivotLeft();
//-----------------------------------------------------------------------------------------


//------------------------------ Distance & Angle Calibration -----------------------------
  //once you have the power values for each movment type you will need
  //to calibrate distance and angles.  Use these four commands to 
  //calbrate for 1 meter forward and reverse asnd 360 degrees for both
  //pivot directions

  // classBot.forward(1);
  // classBot.reverse(1);
  // classBot.pivotRight(360);
  // classBot.pivotLeft(360);
//-----------------------------------------------------------------------------------------


//------------------------------------ Test Calibration -----------------------------------
  //once you have the power, distance and angles calibrated you will want
  //to test that your values work well.  Uncomment these 7 commands and
  //your robot should end at the same place it started at.

  // classBot.forward(1);
  // delay(250);
  // classBot.pivotRight(90);
  // delay(250);
  // classBot.pivotLeft(90);
  // delay(250);
  // classBot.reverse(1);
//-----------------------------------------------------------------------------------------
}//end setup() function



//-----------------------------------------------------------------------------------------
//DO NOT MAKE ANY CHANGES IN THIS FUNCTION
void loop() {                                   //the loop() function runs repeatedly forever
  classBot.run();                               //refreshes sensor readings and various other things running on the robot

  if(millis() - displayTimer >= displayTime){
    displayTimer = millis();                    //reset the display timer
    Serial.print("Left Power: "); Serial.print(classBot.getLeftPower());                          //displays the left power
    Serial.print("\tRight Power: "); Serial.println(classBot.getRightPower());                    //displays the right power
    // Serial.print("Left Count: "); Serial.print(classBot.leftCount());                             //displays the left encoder count
    // Serial.print("\tRight Count: "); Serial.println(classBot.rightCount());Serial.println();      //displays the right encoder count
  }
}
//-----------------------------------------------------------------------------------------