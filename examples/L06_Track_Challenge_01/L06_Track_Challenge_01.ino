/*
 * L06_Track_Challenge_01.ino
 * 
 * Author: Andrew Gafford
 * email: agafford@spacetrek.com
 * Date: Feb. 25th, 2023
 * 
 * Use this program to complete the track challenge
 *
 * You will need to use the calibration values from the previous
 * example.
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

  classBot.forward(1);                            //If your power and calibration values are corect this will make the robot drive forward 1 meter.
  delay(250);
  classBot.pivotRight(90);                        //pivot to the right 90 degrees
  delay(250);
  classBot.forward(0.5);                          //If your power and calibration values are corect this will make the robot drive forward 0.5 meter.
  delay(250);

  //Use what you learned in the rectangle challenge to complete the track challenge.
  //You need to keep the robot over the line at all times or you will lose points.
  //Make your robot finish in the crater.  You want the center of your robot to
  //end over the center of the cross mark in the crater.

}//end setup() function



//-----------------------------------------------------------------------------------------
//DO NOT MAKE ANY CHANGES IN THIS FUNCTION
void loop() {                                   //the loop() function runs repeatedly forever
  classBot.run();                               //refreshes sensor readings and various other things running on the robot
}
//-----------------------------------------------------------------------------------------