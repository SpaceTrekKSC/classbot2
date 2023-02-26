/*
 * L03_LED_Colors_01.ino
 * 
 * Author: Andrew Gafford
 * email: agafford@spacetrek.com
 * Date: Feb. 25th, 2023
 * 
 * This program shows you how to change the LED color
 */

#include <SpaceTrek_ClassBot2.h>                //include the classbot2 library so we have the commands to use the robot

const uint32_t displayTime = 1000;              //how often in ms to send the motor power levels to the serial port
uint32_t displayTimer = 0;                      //a variable to store processor clock count used to control the display timer

void setup() {                                  //the setup() funtion runs once when the program starts
  Serial.begin(115200);                         //Start the serial connection to the PC
  delay(500);                                   //wait 0.5 seconds to allow serial to connect

  classBot.begin();                             //start the classbot object.  Sets pinModes and sets up sensors

  //valid colors are RED, GREEN, BLUE, YELLOW, ORANGE, PURPLE, WHITE
  classBot.setColor(RED);                       //sets the LED strip to red
  delay(1000);                                  //waits for 1 second
  classBot.setColor(GREEN);                     //sets the LED strip to green
  delay(1000);                                  //waits for 1 second
  classBot.setColor(BLUE);                      //sets the LED strip to blue
  delay(1000);                                  //waits for 1 second
  classBot.setColor(YELLOW);                    //sets the LED strip to yellow
  delay(1000);                                  //waits for 1 second
  classBot.setColor(ORANGE);                    //sets the LED strip to orange
  delay(1000);                                  //waits for 1 second
  classBot.setColor(PURPLE);                    //sets the LED strip to purple
  delay(1000);                                  //waits for 1 second
  classBot.setColor(WHITE);                     //sets the LED strip to white
  delay(1000);                                  //waits for 1 second
  classBot.setColor(OFF);                       //turns the LED strip off

}//end setup() function

//-----------------------------------------------------------------------------------------
//DO NOT MAKE ANY CHANGES IN THIS FUNCTION
void loop() {                                   //the loop() function runs repeatedly forever
  classBot.run();                               //refreshes sensor readings and various other things running on the robot
}
//-----------------------------------------------------------------------------------------