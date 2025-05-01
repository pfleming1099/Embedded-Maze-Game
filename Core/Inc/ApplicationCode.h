/*
 * Application_Code.h
 *
 *  Created on: Apr 5, 2024
 *      Author: pflem
 */

#ifndef INC_APPLICATION_CODE_H_
#define INC_APPLICATION_CODE_H_

#include "LCD_Driver.h"
#include "Gyro_Driver.h"
#include "RNG_Driver.h"

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define BUTTON_PRESSED		1
#define BUTTON_UNPRESSED	0

#define BUTTON_PIN			GPIO_PIN_0

#define SPEED_ZERO			0
#define NEAR_ZERO			100
#define VERY_SLOW			1000
#define SLOW				2000
#define FAST				3000

#define DIRECTION_THRESHOLD	5000

#define GREEN_LED_PIN 		GPIO_PIN_13
#define RED_LED_PIN			GPIO_PIN_14

#define QUEUE_SIZE			32

#define BUTTON_EVENT_FLAG	0x00000001U

#define ANGLE_TIMER_PERIOD	50

#define SCREEN_OFFSET		10 //maze offset from screen border draw from top left (px)

typedef enum {
	CLOCKWISE_SLOW,
	CLOCKWISE_FAST,
	COUNTER_CLOCKWISE_SLOW,
	COUNTER_CLOCKWISE_FAST,
	ZERO
} gyroDir;

/*
 * Config struct declaration below, values can be edited in the source file
 */
extern struct Config {
	int version;

	struct Physics {
		int gravity;			//[kg*cm/(s^2)]
		int updateFrequency;
		enum PinAtCenter{
			DRONE,
			MAZE
		}pinAtCenter;
		int angleGain;
	}physics;

	struct Drone {
		struct Disruptor {
			int maxTime;
			int power;
			int minActivationEnergy;
		}disruptor;

		struct EnergyStore {
			int maxEnergy;
			int rechargeRate;
		}energyStore;

		int diameter;
	}drone;

	struct Maze {
		int timeToComplete;
		int cellSize;

		struct Size {
			int width;
			int height;
		}size;

		struct ObstacleProbability {
			int wall;
			int hole;
		}obstacleProbability;

		int holeDiameter;
		bool hardEdged;

		struct Waypoints {
			int number;
			int diameter;
			bool reuse;
			int (*location)[2];
		}waypoints;
	}maze;
}config;

void GPIOinit();
void ApplicationInit();

void CreateMaze();
void DrawMaze();

void BallPosition();

void Disruptor();
void ledOutputGreen();
void ledOutputRed();

void GameOverTimer();
void GameOverFell();
void GameWin(int num_waypoints);

void EXTI0_IRQHandler();

#endif /* INC_APPLICATION_CODE_H_ */

