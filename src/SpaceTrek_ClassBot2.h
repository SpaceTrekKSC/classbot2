/*
*
*SpaceTrek_ClassBot2.h
*
*Author:	Andrew Gafford
*Email:		agafford@spacetrek.com
*Date:		Feb. 25th, 2023
*
*This library is for controling the Space Trek Class Bot 2.0.  It provides
*control of the drive motors, sensors and RF comunication.
*
*/

#ifndef classbot2_h
#define classbot2_h

//includes
#include <Arduino.h>
#include <Wire.h>
#include <BQ25887.h>
#include <VL53L0X.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_NeoPixel.h>
#include <BNO055_support.h>

//defines
//PIN Definitions for the AES Robot Control Board
#define REAR_RIGHT_ENCODER		PIN_PK4
#define REAR_LEFT_ENCODER		PIN_PK5
#define FRONT_RIGHT_ENCODER		PIN_PK6
#define FRONT_LEFT_ENCODER		PIN_PK7
#define FRONT_RIGHT_PWM			PIN_PH3
#define FRONT_LEFT_PWM			PIN_PH4
#define	REAR_LEFT_PWM			PIN_PH5
#define REAR_RIGHT_PWM			PIN_PH6
#define REAR_TB6612_EN			PIN_PC2
#define FRONT_TB6612_EN			PIN_PC3
#define REAR_A1					PIN_PC7
#define REAR_A2					PIN_PC6
#define REAR_B1					PIN_PC5
#define REAR_B2					PIN_PC4
#define FRONT_A1				PIN_PA4
#define FRONT_A2				PIN_PA5
#define FRONT_B1				PIN_PA6
#define FRONT_B2				PIN_PA7

#define BLE_CS					PIN_PB0
#define BLE_IRQ					PIN_PE4

//battery charger
#define BQ25887_CD				PIN_PA1
#define BQ25887_PG				PIN_PA0
#define BQ25887_INT				PIN_PJ4
#define BAT_DISCHARGE			PIN_PA3
#define BAT_VOLTAGE				PIN_PF0

#define SD_CS					PIN_PG2
#define SD_DETECT				PIN_PJ6

#define BOARD_LED				PIN_PF5

#define TPS54824_PG				PIN_PJ3

//Stepper Driver
#define DRV8834_EN				PIN_PG1
#define DRV8834_M0				PIN_PC1
#define DRV8834_M1				PIN_PC0
#define DRV8834_STEP			PIN_PB4
#define DRV8834_DIR				PIN_PB5
#define DRV8834_FAULT			PIN_PJ5

//Expansion Analog
#define ANALOG1					PIN_PK0
#define ANALOG2					PIN_PK1
#define ANALOG3					PIN_PK2
#define ANALOG4					PIN_PK3

#define ANALOG_RANDOM			PIN_PF2

#define NEO_PIXEL_PIN			PIN_PG5
#define NUMPIXELS 				8

//==========END PIN DEFINITIONS

//Settings
#define BQ_WD_TIME				20000
#define VL53L0X_RANGE_TIME		100
#define APDS_COLOR_TIME			100
#define BNO_UPDATE_TIME			50

#define MINIMUM_POWER			40
#define MAXIMUM_POWER			255
#define POWER_BALANCE_TIME		1000

//#define DEBUG
#define DEBUG_UPDATE_TIME			5000


class Classbot{
	struct RGBC{
		uint16_t red;
		uint16_t green;
		uint16_t blue;
		uint16_t clear;
	};
	
	struct bno055_t myBNO;
	//struct bno055_euler eulerData;
	
	volatile uint8_t savePINK = 0;
	volatile uint32_t fleCounter = 0;
	volatile uint32_t freCounter = 0;
	volatile uint8_t fleState = PINK & B10000000;
	volatile uint8_t freState = PINK & B01000000;

	public:
		Classbot();
		Classbot(HardwareSerial &print);

		void processEncoders() __attribute__((always_inline)){
			this->savePINK = PINK;
			if((savePINK & B01000000) != this->fleState){
				this->fleState = savePINK & B01000000;
				if(this->fleState){
					this->fleCounter++;
				}
			}
			if((savePINK & B00010000) != this->freState){
				this->freState = savePINK & B00010000;
				if(this->freState){
					this->freCounter++;
				}
			}
		}
		
		
		
		
		
		
		void begin();
		void run();
		void getUserInput();
		
		void forward(float Distance);
		void goForward();
		void forwardTime(float Time);
		void forwardRange(float Range);
		
		void reverse(float Distance);
		void goReverse();
		void reverseTime(float Time);
		
		void pivotRight(float Angle);
		void goPivotRight();

		void pivotLeft(float Angle);
		void goPivotLeft();

		//void turnLeft(uint8_t Rate);
		//void turnRight(uint8_t Rate);

		void brake();
		void standby();
		
		
		void setPowerForward(uint16_t powerRight, uint16_t powerLeft);
		void setPowerReverse(uint16_t powerRight, uint16_t powerLeft);
		void setPowerPivotRight(uint16_t powerLeft, uint16_t powerRight);
		void setPowerPivotLeft(uint16_t powerLeft, uint16_t powerRight);
		void setSpeed(float speed);
		
		uint8_t getLeftPower();
		uint8_t getRightPower();
		
		void setMeterDistanceForward(uint32_t encoderCountMeter);
		void setMeterDistanceReverse(uint32_t encoderCountMeter);
		void setPivotRight90(uint32_t encoderCountAngle90);
		void setPivotLeft90(uint32_t encoderCountAngle90);
		
		uint32_t avgCount();
		void clearEncoders();	
		
		uint32_t rightCount();
		uint32_t leftCount();

		//Settings
		//void useSerial(bool onOff);
		void verbose(bool onOff);
		void debug(bool onOff);
		
		//Battery Charger
		//BQ25887 charger = BQ25887();
		void startBQ25887();
		
		//Time of Flight Distance Sensor
		uint16_t getDistance();
		//void setStopDistance(uint16_t Distance);
		//void useDistance(bool tof);
		//bool STOPPED_DISTANCE = false;
		
		//Color Sensor
		RGBC color = {0, 0, 0, 0,};
		void useColor(bool onOff);
		
		//BNO055
		struct bno055_euler eulerData;
		
	
	
	private:
		HardwareSerial* printer;
		
		void forward();
		void reverse();
		void pivotRight();
		void pivotLeft();

		void updateBatLED();
		
		uint8_t powerRightForward = MAXIMUM_POWER;
		uint8_t powerLeftForward = MAXIMUM_POWER;
		uint8_t powerRightForwardAdjusted = MAXIMUM_POWER;
		uint8_t powerLeftForwardAdjusted = MAXIMUM_POWER;
		
		uint8_t powerRightReverse = MAXIMUM_POWER;
		uint8_t powerLeftReverse = MAXIMUM_POWER;
		uint8_t powerRightReverseAdjusted = MAXIMUM_POWER;
		uint8_t powerLeftReverseAdjusted = MAXIMUM_POWER;
		
		uint8_t powerRightPivotRight = MAXIMUM_POWER;
		uint8_t powerLeftPivotRight = MAXIMUM_POWER;
		uint8_t powerRightPivotRightAdjusted = MAXIMUM_POWER;
		uint8_t powerLeftPivotRightAdjusted = MAXIMUM_POWER;
		
		uint8_t powerRightPivotLeft = MAXIMUM_POWER;
		uint8_t powerLeftPivotLeft = MAXIMUM_POWER;
		uint8_t powerRightPivotLeftAdjusted = MAXIMUM_POWER;
		uint8_t powerLeftPivotLeftAdjusted = MAXIMUM_POWER;
		
		
		//float driveSpeed = 1.0;
		uint8_t targetPower = MAXIMUM_POWER;
		uint8_t leftPower = MAXIMUM_POWER;
		uint8_t rightPower = MAXIMUM_POWER;
		
		
		bool BALANCE_POWER = false;
		
		
		
		uint32_t powerBalanceTimer = 0;
		uint32_t previousLeftCount = 0;
		uint32_t previousRightCount = 0;
		uint32_t currentLeftCount = 0;
		uint32_t currentRightCount = 0;
		uint32_t currentLeftDifference = 0;
		uint32_t currentRightDifference = 0;
		
		uint32_t meterDistanceForward = 1000;
		uint32_t meterDistanceReverse = 1000;
		uint32_t pivotRight90 = 200;
		uint32_t pivotLeft90 = 200;
		
		//Settings
		bool USE_SERIAL = false;
		bool VERBOSE = false;
		bool DEBUG = false;
		
		//Battery Charger
		volatile uint32_t bqWdTimer = 0;
		volatile uint32_t debugBatTimer = 0;
		bool BQ25887_ACTIVATED = false;
		bool DISCHARGE = false;
		
		//Time of Flight Distance Sensor
		//VL53L0X distanceTOF;
		//bool USE_TOF_DISTANCE = false;
		void startVL53L0X();
		//uint16_t stopDistance = 0;
		uint16_t rangeDistance = 0;
		volatile uint32_t rangeTimer = 0;
		
		
		//Color Sensor
		//Adafruit_APDS9960 apds;
		bool USE_APDS_COLOR = false;
		void startAPDS9960();
		uint16_t red = 0;
		uint16_t greed = 0;
		uint16_t blue = 0;
		uint16_t clear = 0;
		volatile uint32_t colorTimer = 0;
		void getColors();
		
		//BNO055
		uint32_t bnoTimer = 0;
		
		//motion control
		bool FORWARD = false;
		bool REVERSE = false;
		bool PIVOT_LEFT = false;
		bool PIVOT_RIGHT = false;
		bool TURN_LEFT = false;
		bool TURN_RIGHT = false;
		bool BRAKE = false;

};	


extern Classbot classBot;
extern Adafruit_NeoPixel pixels;
extern BQ25887 charger;
extern Adafruit_APDS9960 apds;
extern VL53L0X distanceTOF;

#endif