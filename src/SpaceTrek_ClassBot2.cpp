/*
*
*
*/

#include <Arduino.h>
#include "SpaceTrek_ClassBot2.h"
#include <Wire.h>
#include <BQ25887.h>
#include <VL53L0X.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_NeoPixel.h>
#include <BNO055_support.h>

Classbot classBot = Classbot(Serial);
BQ25887 charger = BQ25887();
VL53L0X distanceTOF;
Adafruit_APDS9960 apds;
Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);


Classbot::Classbot(){
}

Classbot::Classbot(HardwareSerial &print){	
	printer = &print;			//Connection to Serial
	USE_SERIAL = true;
}

void Classbot::begin(){
	// setup all pin modes	
	// Motor Driver Pins
	pinMode(REAR_LEFT_ENCODER, INPUT);
	pinMode(REAR_RIGHT_ENCODER, INPUT);
	pinMode(FRONT_LEFT_ENCODER, INPUT);
	pinMode(FRONT_RIGHT_ENCODER, INPUT);
	pinMode(FRONT_RIGHT_PWM, OUTPUT);
	pinMode(FRONT_LEFT_PWM, OUTPUT);
	pinMode(REAR_LEFT_PWM, OUTPUT);
	pinMode(REAR_RIGHT_PWM, OUTPUT);
	pinMode(REAR_TB6612_EN, OUTPUT);
	pinMode(FRONT_TB6612_EN, OUTPUT);
	pinMode(REAR_A1, OUTPUT);
	pinMode(REAR_A2, OUTPUT);
	pinMode(REAR_B1, OUTPUT);
	pinMode(REAR_B2, OUTPUT);
	pinMode(FRONT_A1, OUTPUT);
	pinMode(FRONT_A2, OUTPUT);
	pinMode(FRONT_B1, OUTPUT);
	pinMode(FRONT_B2, OUTPUT);
	this-> standby();
	
	//6 volt power good pin
	pinMode(TPS54824_PG, INPUT);
	
	// Charger Pins
	pinMode(BQ25887_CD, OUTPUT);
	pinMode(BQ25887_PG, INPUT);
	pinMode(BQ25887_INT, INPUT);
	pinMode(BAT_VOLTAGE, INPUT);
	pinMode(BAT_DISCHARGE, OUTPUT);
	
	PCICR |= B00000100;               //activates PCIE2 interrupt for the motor encoders
	PCMSK2 |= B01010000;              //activates interrupt for PIN PCINT22-FRE and PCINT20-RRE (PCINT23-FLE and PCINT21-RLE not used)
	
	randomSeed(analogRead(ANALOG_RANDOM));    //read an empty analog pin to get random noise for the random number generator seed
	
	pixels.begin();
	for(int i = 0; i < NUMPIXELS; i++){
		pixels.setPixelColor(i, pixels.Color(150, 0, 0));
	}
	pixels.show();
	
	this->startBQ25887();		//includes Wire.begin();

	this->startVL53L0X();
	
	BNO_Init(&myBNO);
	bno055_set_operation_mode(OPERATION_MODE_NDOF);
	
	if (USE_APDS_COLOR){
		this->startAPDS9960();
	}
	
	if(USE_SERIAL){
		printer->println("ClassBot 2.0 started...");
	}
}

void Classbot::run(){
	//this->getUserInput();
	if(BALANCE_POWER){
		if(millis() - this->powerBalanceTimer >= POWER_BALANCE_TIME){
			
			this->previousLeftCount = this->currentLeftCount;
			this->previousRightCount = this->currentRightCount;
			PCICR &= B11111011;						//stop encoder int
			this->currentLeftCount = this->fleCounter;
			this->currentRightCount = this->freCounter;
			this->currentLeftDifference = this->currentLeftCount - this->previousLeftCount;
			this->currentRightDifference = this->currentRightCount - this->previousRightCount;
			
			if(this->currentLeftDifference < (this->currentRightDifference + this->currentBias)){		//reduce right power
				if(this->leftPower == this->targetPower){
					if(this->rightPower > MINIMUM_POWER){
						this->rightPower--;
					}
					//else can't adjust more
				}
				else if(this->leftPower < this->targetPower){
					this->leftPower++;
				}
			}
			else if((this->currentRightDifference + this->currentBias) < this->currentLeftDifference){
				if(this->rightPower == this->targetPower){
					if(this->leftPower > MINIMUM_POWER){
						this->leftPower--;
					}
					//else can't adjust more
				}
				else if(this->rightPower < this->targetPower){
					this->rightPower++;
				}
			}
			
			analogWrite(FRONT_RIGHT_PWM, this->leftPower);
			analogWrite(REAR_RIGHT_PWM, this->rightPower);
			
			//classBot.clearEncoders(); 				//clear encoder counts
			PCICR |= B00000100;						//start encoder int
			this->powerBalanceTimer = millis();
			
		}		
	}
	
	if(millis() - this->bqWdTimer >= BQ_WD_TIME){
		this->bqWdTimer = millis();
		//charger.wdReset();
		
		//Update Battery LEDs
		this->updateBatLED();
	}
	
	if(millis() - this->rangeTimer >= VL53L0X_RANGE_TIME){
		this->rangeTimer = millis();
		this->rangeDistance = distanceTOF.readRangeContinuousMillimeters();
		
		
	}
	
	if(millis() - this->bnoTimer >= BNO_UPDATE_TIME){
		bno055_read_euler_hrp(&eulerData);
	}

	
	if(USE_APDS_COLOR){
		if(millis() - this->colorTimer >= APDS_COLOR_TIME){
			this->colorTimer = millis();
			this->getColors();
		}
	}
	
	if(USE_SERIAL){
		if(DEBUG){
			if(millis() - this->debugBatTimer >= DEBUG_UPDATE_TIME){
				this->debugBatTimer = millis();
				charger.readADCVbatReg(); 
				printer->print(F("Battery Voltage: ")); printer->println(charger.getADC_VBAT(), 2);
				
				this->rangeDistance = distanceTOF.readRangeContinuousMillimeters();
				printer->print(F("Distance: ")); printer->print(distanceTOF.readRangeContinuousMillimeters()); printer->println(F(" mm"));
				
				if(USE_APDS_COLOR){
					printer->print(F("Color: C: ")); printer->println(this->color.clear);
				}
			}
		}
	}	
	
	// check sensors and act if needed
}

void Classbot::getUserInput(){
	static char inChar;
	
	if(USE_SERIAL){
		if(printer->available()){
			inChar = printer->read();

			switch(inChar){
				case 'U':                     //print all fields
					if(this->BQ25887_ACTIVATED){
						//printAllFields();
					}
					else{
						printer->println(F("BQ25887 not activated.  Start the BQ25887 first."));
					}
					break;

				case 'A':
					if(this->BQ25887_ACTIVATED){
						//printADCRegs();
					}
					else{
						printer->println(F("BQ25887 not activated.  Start the BQ25887 first."));
					}
					break;

				case 'F':                     //print faults
					if(this->BQ25887_ACTIVATED){
						//printFaults();
					}
					else{
						printer->println(F("BQ25887 not activated.  Start the BQ25887 first."));
					}
					break;

				case 'S':                     //print status
					if(this->BQ25887_ACTIVATED){
						//printStatFields();
					}
					else{
						printer->println(F("BQ25887 not activated.  Start the BQ25887 first."));
					}
					break;

				case 'I':
					if(this->BQ25887_ACTIVATED){
						charger.setFORCE_ICO(true);
					}
					else{
						printer->println(F("BQ25887 not activated.  Start the BQ25887 first."));
					}
					break;

				case 'D':
					this->DISCHARGE = true;
					if(this->BQ25887_ACTIVATED){
						charger.setEN_CHG(false);
					}
					digitalWrite(BAT_DISCHARGE, HIGH);
					printer->println(F("Draining Battery"));
					break;

				case 'd':
					this->DISCHARGE = false;
					digitalWrite(BAT_DISCHARGE, LOW);
					if(this->BQ25887_ACTIVATED){
						charger.setEN_CHG(true);
					}
					printer->println(F("NOT Draining Battery"));
					break;

				case 'B':
					if(!this->BQ25887_ACTIVATED){
						this->startBQ25887();
					}
					else{
						printer->println(F("BQ25887 already activated"));
					}
					break;

	//			case 'L':
	//				digitalWrite(BQ_PSEL_PIN, LOW);
	//				break;

	//			case 'l':
	//				digitalWrite(BQ_PSEL_PIN, HIGH);
	//				break;
					
				case '?':
					printer->println();
					printer->println(F("U - Print all register fields"));
					printer->println(F("A - Print ADC Values"));
					printer->println(F("F - Print Faults"));
					printer->println(F("S - Print Status Fields"));
					printer->println(F("I - Set BQ25887 FORCE_ICO register to true"));
					printer->println(F("D - Start discharging battery"));
					printer->println(F("d - Stop discharging battery"));
					printer->println(F("B - Start BQ25887"));
	//				printer->println(F("L - Set BQ25887 PSEL pin LOW"));
	//				printer->println(F("l - Set BQ25887 PSEL pin HIGH"));
					printer->println();
					break;

				default:
					//not a command message
					break;
			}
			
			while(printer->available()){      //empty any remaining characters in the serial buffer
				printer->read();
			}			
		}
	}
}

void Classbot::forward(){
	PORTC &= B11110011;						//sets PORTC 2 and 3 LOW (REAR_TB6612_EN and FRONT_TB6612_EN)
	
	//setup motor drivers to go forward
	PORTA |= B01010000;						//set PORTA pins 4 and 6 HIGH (FRONT_A1 and FRONT_B1)
	PORTA &= B01011111;						//set PORTA pins 5 and 7 LOW  (FRONT_A2 and FRONT_B2)
	PORTC |= B10100000;						//set PORTC pins 5 and 7 HIGH (REAR_B1 and REAR_A1)
	PORTC &= B10101111;						//set PORTC pins 4 and 6 LOW  (REAR_B2 and REAR_A2)	
	
	/*
	//for current classBot setup the rear is right and forward is left
	analogWrite(FRONT_RIGHT_PWM, this->powerLeftForward * this->driveSpeed);		//set all of the PWM power levels
	//analogWrite(FRONT_LEFT_PWM, this->powerLeftForward * this->driveSpeed);
	analogWrite(REAR_RIGHT_PWM, this->powerRightForward * this->driveSpeed);
	//analogWrite(REAR_LEFT_PWM, this->powerRightForward * this->driveSpeed);
	*/
	
	
	analogWrite(FRONT_RIGHT_PWM, this->leftPower);
	analogWrite(REAR_RIGHT_PWM, this->rightPower);
	

	PORTC |= B00001100;						//sets PORTC 2 and 3 HIGH (REAR_TB6612_EN and FRONT_TB6612_EN)
}

void Classbot::forward(float Distance){
	BALANCE_POWER = true;
	this->currentBias = this->forwardBias;
	this->leftPower = this->powerLeftForwardAdjusted;
	this->rightPower = this->powerRightForwardAdjusted;
	float metersToTravel = Distance * this->meterDistanceForward;
	uint32_t encoderCountsToTravel = metersToTravel;
	
	this->clearEncoders();
	this->powerBalanceTimer = millis();
	this->forward();	
	while(this->avgCount() < encoderCountsToTravel){
		this->run();
	}
	this->brake();		//once traveled specific distance hit the brakes
	//BALANCE_POWER = false;
}

void Classbot::goForward(){
	BALANCE_POWER = true;
	this->currentBias = this->forwardBias;
	this->leftPower = this->powerLeftForwardAdjusted;
	this->rightPower = this->powerRightForwardAdjusted;
	
	this->clearEncoders();
	this->powerBalanceTimer = millis();
	this->forward();	
}

void Classbot::forwardTime(float Time){
	BALANCE_POWER = true;
	this->currentBias = this->forwardBias;
	this->leftPower = this->powerLeftForwardAdjusted;
	this->rightPower = this->powerRightForwardAdjusted;
	float timeMilliseconds = Time * 1000.0;
	uint32_t timeToTravel = timeMilliseconds;
	this->clearEncoders();
	uint32_t startTravelTime = millis();
	this->forward();
	while(millis() < startTravelTime + timeToTravel){
		this->run();
	}
	this->brake();
	//BALANCE_POWER = false;
}

void Classbot::forwardRange(float Range){
	BALANCE_POWER = true;
	this->currentBias = this->forwardBias;
	this->leftPower = this->powerLeftForwardAdjusted;
	this->rightPower = this->powerRightForwardAdjusted;	
	this->rangeDistance = distanceTOF.readRangeContinuousMillimeters();
	float range = 0.0;
	if(Range <= 30.0){
		range = 30.0;
	}
	else if(Range >= 250.0){
		range = 250.0;
	}
	else{
		range = Range;
	}
	this->clearEncoders();
	this->forward();
	while(this->rangeDistance > range){
		this->run();
	}
	this->brake();
	//BALANCE_POWER = false;
}

void Classbot::reverse(){
	PORTC &= B11110011;						//sets PORTC 2 and 3 LOW (REAR_TB6612_EN and FRONT_TB6612_EN)
	
	//setup motor drivers to go reverse
	PORTA |= B10100000;						//set PORTA pins 5 and 7 HIGH (FRONT_A2 and FRONT_B2)
	PORTA &= B10101111;						//set PORTA pins 4 and 6 LOW  (FRONT_A1 and FRONT_B1)
	PORTC |= B01010000;						//set PORTC pins 4 and 6 HIGH (REAR_B2 and REAR_A2)
	PORTC &= B01011111;						//set PORTC pins 5 and 7 LOW  (REAR_B1 and REAR_A1)
	
	/*
	analogWrite(FRONT_RIGHT_PWM, this->powerLeftReverse * this->driveSpeed);		//set all of the PWM power levels
	//analogWrite(FRONT_LEFT_PWM, this->powerLeftReverse * this->driveSpeed);
	analogWrite(REAR_RIGHT_PWM, this->powerRightReverse * this->driveSpeed);
	//analogWrite(REAR_LEFT_PWM, this->powerRightReverse * this->driveSpeed);
	*/
	
	analogWrite(FRONT_RIGHT_PWM, this->leftPower);
	analogWrite(REAR_RIGHT_PWM, this->rightPower);

	PORTC |= B00001100;				//sets PORTC 2 and 3 HIGH (REAR_TB6612_EN and FRONT_TB6612_EN)
}

void Classbot::reverse(float Distance){
	BALANCE_POWER = true;
	this->currentBias = this->reverseBias;
	this->leftPower = this->powerLeftReverseAdjusted;
	this->rightPower = this->powerRightReverseAdjusted;
	float metersToTravel = Distance * this->meterDistanceReverse;
	uint32_t encoderCountsToTravel = metersToTravel;
	
	this->clearEncoders();
	this->powerBalanceTimer = millis();
	this->reverse();	
	while(this->avgCount() < encoderCountsToTravel){
		this->run();
	}
	this->brake();		//once traveled specific distance hit the brakes
	//BALANCE_POWER = false;
}

void Classbot::goReverse(){
	BALANCE_POWER = true;
	this->currentBias = this->reverseBias;
	this->leftPower = this->powerLeftReverseAdjusted;
	this->rightPower = this->powerRightReverseAdjusted;
	
	this->clearEncoders();
	this->powerBalanceTimer = millis();
	this->reverse();	
}

void Classbot::reverseTime(float Time){
	BALANCE_POWER = true;
	this->currentBias = this->reverseBias;
	this->leftPower = this->powerLeftReverseAdjusted;
	this->rightPower = this->powerRightReverseAdjusted;
	float timeMilliseconds = Time * 1000.0;
	uint32_t timeToTravel = timeMilliseconds;
	this->clearEncoders();
	uint32_t startTravelTime = millis();
	this->clearEncoders();
	this->reverse();
	while(millis() < startTravelTime + timeToTravel){
		this->run();
	}
	this->brake();
	//BALANCE_POWER = false;
}

void Classbot::pivotRight(){
	PORTC &= B11110011;						//sets PORTC 2 and 3 LOW (REAR_TB6612_EN and FRONT_TB6612_EN)
	
	//set motor driver to pivot right
	PORTA |= B10010000;						//set PORTA pins 4 and 7 HIGH (FRONT_A1 and FRONT_B2)
	PORTA &= B10011111;						//set PORTA pins 5 and 6 LOW  (FRONT_A2 and FRONT_B1)
	PORTC |= B10010000;						//set PORTC pins 4 and 7 HIGH (REAR_B2 and REAR_A1)
	PORTC &= B10011111;						//set PORTC pins 5 and 6 LOW  (REAR_B1 and REAR_A2)

	analogWrite(FRONT_RIGHT_PWM, this->leftPower);		//set all of the PWM power levels
	//analogWrite(FRONT_LEFT_PWM, this->leftPower);
	analogWrite(REAR_RIGHT_PWM, this->rightPower);
	//analogWrite(REAR_LEFT_PWM, this->rightPower);

	PORTC |= B00001100;				//sets PORTC 2 and 3 HIGH (REAR_TB6612_EN and FRONT_TB6612_EN)
}

void Classbot::pivotRight(float Angle){
	BALANCE_POWER = true;
	this->currentBias = 0;
	this->leftPower = this->powerLeftPivotRightAdjusted;
	this->rightPower = this->powerRightPivotRightAdjusted;
	float angleToTurn = (Angle * this->pivotRight90)/90.0;
	uint32_t encoderCountsToTurn = angleToTurn;
	
	this->clearEncoders();
	this->pivotRight();	
	while(this->avgCount() < encoderCountsToTurn){
		this->run();
	}
	this->brake();		//once traveled specific distance hit the brakes
	//BALANCE_POWER = false;
}

void Classbot::goPivotRight(){
	BALANCE_POWER = true;
	this->currentBias = 0;
	this->leftPower = this->powerLeftPivotRightAdjusted;
	this->rightPower = this->powerRightPivotRightAdjusted;
	
	this->clearEncoders();
	this->powerBalanceTimer = millis();
	this->pivotRight();	
}

void Classbot::pivotLeft(){
	PORTC &= B11110011;						//sets PORTC 2 and 3 LOW (REAR_TB6612_EN and FRONT_TB6612_EN)
	
	//set motor driver to pivot left
	PORTA |= B01100000;						//set PORTA pins 5 and 6 HIGH (FRONT_A2 and FRONT_B1)
	PORTA &= B01101111;						//set PORTA pins 4 and 7 LOW  (FRONT_A1 and FRONT_B2)
	PORTC |= B01100000;						//set PORTC pins 5 and 6 HIGH (REAR_B1 and REAR_A2)
	PORTC &= B01101111;						//set PORTC pins 4 and 7 LOW  (REAR_B2 and REAR_A1)
	
	analogWrite(FRONT_RIGHT_PWM, this->leftPower);		//set all of the PWM power levels
	//analogWrite(FRONT_LEFT_PWM, this->leftPower);
	analogWrite(REAR_RIGHT_PWM, this->rightPower);
	//analogWrite(REAR_LEFT_PWM, this->rightPower);

	PORTC |= B00001100;				//sets PORTC 2 and 3 HIGH (REAR_TB6612_EN and FRONT_TB6612_EN)
}

void Classbot::pivotLeft(float Angle){
	BALANCE_POWER = true;
	this->currentBias = 0;
	this->leftPower = this->powerLeftPivotLeftAdjusted;
	this->rightPower = this->powerRightPivotLeftAdjusted;
	float angleToTurn = (Angle * this->pivotLeft90)/90.0;
	uint32_t encoderCountsToTurn = angleToTurn;
	
	this->clearEncoders();
	this->pivotLeft();	
	while(this->avgCount() < encoderCountsToTurn){
		this->run();
		
	}
	this->brake();		//once traveled specific distance hit the brakes
	//BALANCE_POWER = false;
	
}

void Classbot::goPivotLeft(){
	BALANCE_POWER = true;
	this->currentBias = 0;
	this->leftPower = this->powerLeftPivotLeftAdjusted;
	this->rightPower = this->powerRightPivotLeftAdjusted;
	
	this->clearEncoders();
	this->powerBalanceTimer = millis();
	this->pivotLeft();	
}

void Classbot::brake(){
	PORTA |= B11110000;			//sets FRONT_A1 through _B2 HIGH)
	PORTC |= B11111100;			//sets REAR_A1 through REAR_B2 HIGH and sets FRONT and REAR TB6612_EN HIGH)
	BALANCE_POWER = false;
	this->currentBias = 0;
}

void Classbot::standby(){
	PORTC &= B11110011;				//sets PORTC pin 2 and 3 low (REAR_TB6612_EN and FRONT_TB6612_EN)
	BALANCE_POWER = false;
	this->currentBias = 0;
}

void Classbot::setPowerForward(uint16_t powerLeft, uint16_t powerRight){
	if(powerLeft > MAXIMUM_POWER) powerLeft = MAXIMUM_POWER;
	else if(powerLeft < MINIMUM_POWER) powerLeft = MINIMUM_POWER;
	if(powerRight > MAXIMUM_POWER) powerRight = MAXIMUM_POWER;
	else if(powerRight < MINIMUM_POWER) powerRight = MINIMUM_POWER;
	
	if((powerLeft != MAXIMUM_POWER) && (powerRight != MAXIMUM_POWER)){
		if(powerLeft > powerRight){
			powerRight = MAXIMUM_POWER * (float(powerRight) / float(powerLeft));
			powerLeft = MAXIMUM_POWER;
		}
		else if(powerRight > powerLeft){
			powerLeft = MAXIMUM_POWER * (float(powerLeft) / float(powerRight));
			powerRight = MAXIMUM_POWER;
		}
		else{
			powerLeft = MAXIMUM_POWER;
			powerRight = MAXIMUM_POWER;
		}
	}
	
	this->powerRightForward = powerRight;
	this->powerLeftForward = powerLeft;
	
	this->setSpeed(100);
}

void Classbot::setPowerReverse(uint16_t powerLeft, uint16_t powerRight){
	if(powerLeft > MAXIMUM_POWER) powerLeft = MAXIMUM_POWER;
	else if(powerLeft < MINIMUM_POWER) powerLeft = MINIMUM_POWER;
	if(powerRight > MAXIMUM_POWER) powerRight = MAXIMUM_POWER;
	else if(powerRight < MINIMUM_POWER) powerRight = MINIMUM_POWER;
	
	if((powerLeft != MAXIMUM_POWER) && (powerRight != MAXIMUM_POWER)){
		if(powerLeft > powerRight){
			powerRight = MAXIMUM_POWER * (float(powerRight) / float(powerLeft));
			powerLeft = MAXIMUM_POWER;
		}
		else if(powerRight > powerLeft){
			powerLeft = MAXIMUM_POWER * (float(powerLeft) / float(powerRight));
			powerRight = MAXIMUM_POWER;
		}
		else{
			powerLeft = MAXIMUM_POWER;
			powerRight = MAXIMUM_POWER;
		}
	}
	
	this->powerRightReverse = powerRight;
	this->powerLeftReverse = powerLeft;
	
	this->setSpeed(100);
}

void Classbot::setBiasForward(int8_t bias){
	this->forwardBias = bias;
}

void Classbot::setBiasReverse(int8_t bias){
	this->reverseBias = bias;
}

void Classbot::setPowerPivotRight(uint16_t powerLeft, uint16_t powerRight){
	if(powerLeft > MAXIMUM_POWER) powerLeft = MAXIMUM_POWER;
	else if(powerLeft < MINIMUM_POWER) powerLeft = MINIMUM_POWER;
	if(powerRight > MAXIMUM_POWER) powerRight = MAXIMUM_POWER;
	else if(powerRight < MINIMUM_POWER) powerRight = MINIMUM_POWER;
	
	if((powerLeft != MAXIMUM_POWER) && (powerRight != MAXIMUM_POWER)){
		if(powerLeft > powerRight){
			powerRight = MAXIMUM_POWER * (float(powerRight) / float(powerLeft));
			powerLeft = MAXIMUM_POWER;
		}
		else if(powerRight > powerLeft){
			powerLeft = MAXIMUM_POWER * (float(powerLeft) / float(powerRight));
			powerRight = MAXIMUM_POWER;
		}
		else{
			powerLeft = MAXIMUM_POWER;
			powerRight = MAXIMUM_POWER;
		}
	}
	
	this->powerRightPivotRight = powerRight;
	this->powerLeftPivotRight = powerLeft;
	
	this->setSpeed(100);
}

void Classbot::setPowerPivotLeft(uint16_t powerLeft, uint16_t powerRight){
	if(powerLeft > MAXIMUM_POWER) powerLeft = MAXIMUM_POWER;
	else if(powerLeft < MINIMUM_POWER) powerLeft = MINIMUM_POWER;
	if(powerRight > MAXIMUM_POWER) powerRight = MAXIMUM_POWER;
	else if(powerRight < MINIMUM_POWER) powerRight = MINIMUM_POWER;
	
	if((powerLeft != MAXIMUM_POWER) && (powerRight != MAXIMUM_POWER)){
		if(powerLeft > powerRight){
			powerRight = MAXIMUM_POWER * (float(powerRight) / float(powerLeft));
			powerLeft = MAXIMUM_POWER;
		}
		else if(powerRight > powerLeft){
			powerLeft = MAXIMUM_POWER * (float(powerLeft) / float(powerRight));
			powerRight = MAXIMUM_POWER;
		}
		else{
			powerLeft = MAXIMUM_POWER;
			powerRight = MAXIMUM_POWER;
		}
	}
	
	this->powerRightPivotLeft = powerRight;
	this->powerLeftPivotLeft = powerLeft;
	
	this->setSpeed(100);
}

void Classbot::setSpeed(float speed){
	if(speed < 0.0) speed = 0.0;
	else if(speed > 100.0) speed = 100.0;
	
	speed = speed/100.0;		//puts speed into percent
	
	//this->driveSpeed = (speed/100.0);
	this->targetPower = MINIMUM_POWER + (float(MAXIMUM_POWER - MINIMUM_POWER)*speed);
		
	//set the adjusted powers
	this->powerRightForwardAdjusted = this->powerRightForward * speed;
	this->powerLeftForwardAdjusted = this->powerLeftForward * speed;
	this->powerRightReverseAdjusted = this->powerRightReverse * speed;
	this->powerLeftReverseAdjusted = this->powerLeftReverse * speed;
	this->powerRightPivotRightAdjusted = this->powerRightPivotRight * speed;
	this->powerLeftPivotRightAdjusted = this->powerLeftPivotRight * speed;
	this->powerRightPivotLeftAdjusted = this->powerRightPivotLeft * speed;
	this->powerLeftPivotLeftAdjusted = this->powerLeftPivotLeft * speed;
	
	//make sure all adjusted powers are at least MINIMUM_POWER
	if(this->powerRightForwardAdjusted < MINIMUM_POWER) this->powerRightForwardAdjusted = MINIMUM_POWER;
	if(this->powerLeftForwardAdjusted < MINIMUM_POWER) this->powerLeftForwardAdjusted = MINIMUM_POWER;
	if(this->powerRightReverseAdjusted < MINIMUM_POWER) this->powerRightReverseAdjusted = MINIMUM_POWER;
	if(this->powerLeftReverseAdjusted < MINIMUM_POWER) this->powerLeftReverseAdjusted = MINIMUM_POWER;
	if(this->powerRightPivotRightAdjusted < MINIMUM_POWER) this->powerRightPivotRightAdjusted = MINIMUM_POWER;
	if(this->powerLeftPivotRightAdjusted < MINIMUM_POWER) this->powerLeftPivotRightAdjusted = MINIMUM_POWER;
	if(this->powerRightPivotLeftAdjusted < MINIMUM_POWER) this->powerRightPivotLeftAdjusted = MINIMUM_POWER;
	if(this->powerLeftPivotLeftAdjusted < MINIMUM_POWER) this->powerLeftPivotLeftAdjusted = MINIMUM_POWER;

}

uint8_t Classbot::getLeftPower(){
	return this->leftPower;
}
uint8_t Classbot::getRightPower(){
	return this->rightPower;
}

void Classbot::setMeterDistanceForward(uint32_t encoderCountMeter){
	this->meterDistanceForward = encoderCountMeter;
}

void Classbot::setMeterDistanceReverse(uint32_t encoderCountMeter){
	this->meterDistanceReverse = encoderCountMeter;
}

void Classbot::setPivotRight90(uint32_t encoderCountAngle90){
	this->pivotRight90 = encoderCountAngle90;
}

void Classbot::setPivotLeft90(uint32_t encoderCountAngle90){
	this->pivotLeft90 = encoderCountAngle90;
}

void Classbot::setColor(uint8_t color){
	switch(color){
		case RED:
			this->changeLED(150, 0, 0);			//set LED to red
			break;
		case GREEN:
			this->changeLED(0, 150, 0);			//set LED to green
			break;
		case BLUE:
			this->changeLED(0, 0, 150);			//set LED to blue
			break;
		case YELLOW:
			this->changeLED(120, 150, 0);		//set LED to YELLOW
			break;
		case ORANGE:
			this->changeLED(150, 60, 0);		//set color to orange
			break;
		case PURPLE:
			this->changeLED(150, 0, 150);		//set LED to purple
			break;
		case WHITE:
			this->changeLED(100, 100, 100);		//set LED to white
			break;
		default:
			this->changeLED(0, 0, 0);			//set to OFF
			break;
	}
}

void Classbot::changeLED(uint8_t r, uint8_t g, uint8_t b){
	for(int i = 0; i < NUMPIXELS; i++){
		pixels.setPixelColor(i, pixels.Color(r, g, b));
	}
	pixels.show();
}

uint32_t Classbot::avgCount(){
	float average = (this->fleCounter + this->freCounter) / 2.0;
	uint32_t avg = average;
	return avg;
}

void Classbot::clearEncoders(){
	this->fleCounter = 0;
	this->freCounter = 0;
}

//==========================SETTINGS=========================================

void Classbot::verbose(bool onOff){
	this->VERBOSE = onOff;
}

void Classbot::debug(bool onOff){
	this->DEBUG = onOff;
}


//==========================SHIFT REG LEDS===================================

void Classbot::updateBatLED(){
	float batVoltage = 0.008216031*analogRead(BAT_VOLTAGE);
	
	if(batVoltage >= 7.0){
		
	}
	if(batVoltage >= 7.3){
		
	}
	if(batVoltage >= 7.6){
		
	}
	if(batVoltage >= 7.9){
		
	}
	
}






//==========================VL53L0X==========================================

void Classbot::startVL53L0X(){
	distanceTOF.init();
	distanceTOF.setTimeout(500);
	distanceTOF.startContinuous(100);
}

uint16_t Classbot::getDistance(){
	return distanceTOF.readRangeContinuousMillimeters();
}

//void Classbot::setStopDistance(uint16_t Distance){
//	this->stopDistance = Distance;
//}



//==========================APDS9960==========================================
void Classbot::startAPDS9960(){
	if(USE_APDS_COLOR){
		USE_APDS_COLOR = apds.begin();
	}
	if(USE_APDS_COLOR){
		apds.enableColor(true);
		apds.setADCGain(APDS9960_AGAIN_16X);
		apds.setADCIntegrationTime(50);
		
		analogWrite(ANALOG4, 255);
	}
}

void Classbot::useColor(bool onOff){
	USE_APDS_COLOR = onOff;
}

void Classbot::getColors(){
	while(!apds.colorDataReady()){      //wait for color data to be ready
		delay(2);
	}
	apds.getColorData(&this->color.red, &this->color.green, &this->color.blue, &this->color.clear);
}



//==========================BQ25887==========================================

void Classbot::startBQ25887(){
	digitalWrite(BQ25887_CD, LOW);
	this->bqWdTimer = millis();

	if(charger.begin()){
		if(DEBUG){
			printer->println(F("BQ25887 is working..."));
		}
	}
	else{
		if(DEBUG){
			printer->println(F("BQ25887 is NOT working.  Check your wires..."));
			for(int i = 0; i > 20; i++){
				printer->print(".");
				delay(500);
			}
			printer->println();
		}
	}
	delay(100);

	if(DEBUG){
		printer->println(F("Resetting the BQ25887 registers..."));
	}
	charger.registerReset();
	delay(100);

  //getFirstFiveRegs();
  //printFirstFiveRegs();

	charger.setADC_EN(true);                                    //enable ADC control
	charger.setADC_ONE_SHOT(false);                             //enable ADC one-shot mode
	charger.setTCB_ACTIVE(0);                                   //time interval to stop charging and discharging for cell voltage meassurments as uint8_t, 0 is 4s, 1 is 32s, 2 is 2min, 3 is 4min, incorrect values go to default value of 2
	charger.setILIMPinFunction(false);                          //enable current limit pin as boolean true or false, default is true
	//charger.setInputVoltageLimit(4.5);                          //input voltage limit as floating point, Range 3.9 - 5.5, default is 4.3V

	BQ25887_ACTIVATED = true;
	
	
	if(DEBUG){
		printer->println(F("Finished starting BQ25887..."));
	}
}

uint32_t Classbot::rightCount(){
	return this->freCounter;
}
uint32_t Classbot::leftCount(){
	return this->fleCounter;
}

ISR (PCINT2_vect){
  classBot.processEncoders();
}
