/*
 * L10_Random_Explorer_01.ino
 * 
 * Author:  Andrew Gafford
 * email:   agafford@spacetrek.com
 * Date:    Feb. 25th, 2023
 * 
 * This is an example program to make the ClassBot 2.0 randomly explor using its distance sensor.
 */


//INCLUDES
#include <SpaceTrek_ClassBot2.h>

const uint32_t displayTime = 1000;
uint32_t displayTimer = 0;

float blockedDistance = 150.0;

void setup() {
  Serial.begin(115200);
  delay(500);

  classBot.begin();

  //Drive Setup
  classBot.setPowerForward(255, 255);           //set the power level for the motors when moving forward (right side, left side)
  classBot.setPowerReverse(255, 255);           //set the power level for the motors when moving fin reverse (right side, left side)
  classBot.setPowerPivotRight(255, 255);        //set the power level for the motors when pivoting left or right
  classBot.setPowerPivotRight(255, 255);        //set the power level for the motors when pivoting left or right
  classBot.setBiasForward(0);                   //set the forward bias. Negative goes more left, positive goes more right
  classBot.setBiasReverse(0);                   //set the reverse bias. Negative goes more left, positive goes more right

  classBot.setMeterDistanceForward(1500);       //set how many encoder pulses are needed to drive forward 1 meter
  classBot.setMeterDistanceReverse(1500);       //set how many encoder pulses are needed to drive in reverse 1 meter
  classBot.setPivotRight90(275);                //set how many encoder pulses are needed to pivot right 90 degrees
  classBot.setPivotLeft90(275);                 //set how many encoder pulses are needed to pivot left 90 degrees
  
  delay(1000);
}

void loop() {
  classBot.run();
  classBot.forwardRange(blockedDistance);
  delay(250);
  randomDirection();
}

void randomDirection(){
  uint32_t randomNumber = 0;             //a variable for a random number used to pick a random direction
  bool randomDirection = 0;              //a variable to store the choosen random direction
  randomNumber = random(2000);                  //generate a random number
  randomDirection = randomNumber % 2;           //use modulo operator to get the remainder of the random number divided by 2 (a way to check if odd or even)

  //we know there are only 4 possible directions and one of them is blocked by
  //an obstical and another is the direction we came from.  So we do not need
  //an endless loop making random turns.  We only want the first attempted turn
  //to be random.  After that if the first attempt is blocked we want to try the
  //other direction.  If it is also blocked then we will go back the way we came
  //from

  //first new direction option
  if(randomDirection){                          //the result of the modulo operation is a 1 or a 0, in other words true or false
    classBot.pivotRight(90);                       //if the result was 1 then pivot to the right
    delay(250);                                 //another short pause
  }
  else{                                         //if the result of the modulo operation was not 1 (it was 0)
    classBot.pivotLeft(90);                        //then pivot to the left
    delay(250);                                 //also another short pause
  }
  classBot.run();

  //second new direction option
  if(classBot.getDistance() < blockedDistance){         //if the new random direction is blocked then go back 180 degrees
    if(randomDirection){                        //we are still using the random direction, but now going the oposite
      classBot.pivotLeft(180);                     //way and for 180 degrees instead of only 90 degrees.
      delay(250);                               //
    }                                           //We could have just turned 180 degrees either left or right and
    else{                                       //regardless of what random way we originally went we would always
      classBot.pivotRight(180);                    //turn the oposite direction.  But if we needed a correction it would
      delay(250);                               //always be the same; either left or right
    }                                           //
  }                                             //end of the second direction option
  classBot.run();

  //no new direction possible...
  //go back the way we came from
  if(classBot.getDistance() < blockedDistance){         //if the second attempted new direction is still blocked then we know
    if(randomDirection){                        //that three of the possible four directions are blocked and our only
      classBot.pivotLeft(90);                      //option is to go back the way we came from.  So we turn an additional
      delay(250);                               //90 degreed to head back the way we came from
    }                                           //
    else{                                       //regardless if you use the randomDirection in the second new direction
      classBot.pivotRight(90);                     //attempt you will always go the oposite way from the first attempt on
      delay(250);                               //this final option.
    }                                           //   
  }

}
