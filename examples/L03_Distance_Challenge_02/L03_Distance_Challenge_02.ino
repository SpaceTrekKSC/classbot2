/*
 * L03_Distance_Challenge_02.ino
 * 
 * Author: Andrew Gafford
 * email: agafford@spacetrek.com
 * Date: Feb. 25th, 2023
 * 
 * Use this program to complete the advanced distance challenge
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
  
  //sets the LED strip to green
  for(int i = 0; i < NUMPIXELS; i++){
    pixels.setPixelColor(i, pixels.Color(0, 150, 0));
  }
  pixels.show();

  

//-------------------------------------- Drive Setup --------------------------------------
  //Power Settings
  classBot.setPowerForward(255, 255);           //set the power level for the motors when moving forward (left side, right side)
  classBot.setPowerReverse(255, 255);           //set the power level for the motors when moving in reverse (left side, right side)
  classBot.setPowerPivotRight(255, 255);        //set the power level for the motors when pivoting right
  classBot.setPowerPivotLeft(255, 255);         //set the power level for the motors when pivoting left              

  //calibration settings
  classBot.setMeterDistanceForward(1400);       //set how many encoder pulses are needed to drive forward 1 meter
  classBot.setMeterDistanceReverse(1400);       //set how many encoder pulses are needed to drive reverse 1 meter
  classBot.setPivotRight90(275);                //set how many encoder pulses are needed to pivot right 90 degrees
  classBot.setPivotLeft90(275);                 //set how many encoder pulses are needed to pivot left 90 degrees

  // classBot.setSpeed(75);                        //set the speed for forward and reverse as percent of power

  delay(1000);                                  //wait for 1 second so the robot doesn't move as soon as you turn it on.
//-----------------------------------------------------------------------------------------

  classBot.forward(1);                            //If your power and calibration values are corect this will make the robot drive forward 1 meter.
  classBot.pivotRight(90);                        //pivot to the right 90 degrees
  classBot.forward(0.5);                          //If your power and calibration values are corect this will make the robot drive forward 0.5 meter.

  //add more code to complete a 1 meter by 0.5 meter rectangle.
  //don't forget the final pivot so the robot is facing the correct way when done.

}//end setup() function



//-----------------------------------------------------------------------------------------
//DO NOT MAKE ANY CHANGES IN THIS FUNCTION
void loop() {                                   //the loop() function runs repeatedly forever
  classBot.run();                               //refreshes sensor readings and various other things running on the robot
}
//-----------------------------------------------------------------------------------------