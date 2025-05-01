/*
 * ApplicationCode.c
 *
 *  Created on: Nov 14, 2023
 *      Author: xcowa
 */

#include "ApplicationCode.h"

/*
 * config struct definitions. change values to your desire! be aware that moving values outside limits will
 * break the program.
 */
struct Config config = {
	.version = 1,						//(int) version
	.physics = {
		.gravity = 980,					//(int) [kg*cm/(s^2)]
		.updateFrequency = 50,			//(int) [Hz]
		.pinAtCenter = DRONE,			//(enum) [DRONE, MAZE]
		.angleGain = 50				//(int) [-/1000], (PhysAngleRate = Gyro*500/1000)
	},
	.drone = {
		.disruptor = {
			.maxTime = 1000,				//(int) [ms]
			.power = 10000,				//(int) [mW]
			.minActivationEnergy = 6000	//(int) [mJ]
		},
		.energyStore = {
			.maxEnergy = 15000,			//(int) [mJ]
			.rechargeRate = 1000			//(int) [mW]
		},
		.diameter = 10					//(int) [mm]
	},
	.maze = {
		.timeToComplete = 10000,		//(int) [ms]
		.cellSize = 15,					//(int) [mm]
		.size = {
			.width = 15,				//(int) [cells]
			.height = 15				//(int)	[cells]
		},
		.obstacleProbability = {
			.wall = 100	,				//(int) [Pr * 1000]
			.hole = 200					//(int) [Pr * 1000]
		},
		.holeDiameter = 11,				//(int) [mm]
		.hardEdged = true,				//(bool) Does the maze have a border of walls around it
		.waypoints = {
			.number = 4,				//(int) number of waypoints
			.diameter = 90,				//(int) [mm]
			.reuse = false,				//(bool)
			.location = (int[][2]){{50,50}, {130,50}, {130,130}, {50, 130}} //#0: Start point, #n-1: goal if reuse is false
		}
	}
};

/*
 * Timer
 */
//static osTimerId_t angleTimerID;
//
//static StaticTimer_t timerTCB;
//
//const osTimerAttr_t timerAttr = {
//		.name = "angleTimer",
//		//.attr_bits = 0,
//		.cb_mem = &timerTCB,
//		.cb_size = sizeof(timerTCB)
//};

/*
 * Thread
 */
static osThreadId_t ballPositionID;
static osThreadId_t disruptorID;
static osThreadId_t ledOutputGreenID;
static osThreadId_t ledOutputRedID;
static osThreadId_t timerThreadID;

/*
 * Mutex
 */
//static osMutexId_t ballPositionMutex;
//static osMutexId_t angleMutex;
//static osMutexId_t ball_collision_mutex;
//static osMutexId_t energyStoreMutex;

/*
 * Semaphore
 */

/*
 * current ball position
 */
typedef struct {
	double x;
	double y;
	double angle_x;
	double angle_y;
}BallPosition_t;

typedef struct{
	bool top;
	int top_coord[2];
	bool left;
	int left_coord[2];
	bool right;
	int right_coord[2];
	bool bottom;
	int bottom_coord[2];
	bool hole;
	int hole_coord[2];

	bool empty;
}Cell;

//typedef struct {
//	Cell walls[config.maze.size.width][config.maze.size.height];
//}Walls_Info;

BallPosition_t ball_position;

bool ball_collision = true;
double energy_store;

bool game_over = false;

int time;

//Cell maze[config.maze.size.width][config.maze.size.height];

Cell maze[15][15];

/*
 * GPIO init interrupt: initializes the GPIOA clock and interrupt for button
 */
void GPIOinit(){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIOinit;
    GPIOinit.Pin = BUTTON_PIN;
    GPIOinit.Mode = GPIO_MODE_IT_FALLING;
    GPIOinit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIOinit);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/*
 *
 */
void ApplicationInit(){
	//GPIO
	//GPIOinit();

	//LCD
	LTCD__Init();
	LTCD_Layer_Init(0);
	LCD_Clear(0, LCD_COLOR_WHITE);
	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);

	//Gyro
	Gyro_Init();
	Gyro_Power_On();

	//RNG
	RNG_Init();

	//Maze
	CreateMaze();
	DrawMaze();

	//Thread
	ballPositionID = osThreadNew(BallPosition, NULL, NULL);

	if (ballPositionID == NULL){
		fprintf(stderr, "ballPositionID creation failed!\n");
		while(1);
	}

	disruptorID = osThreadNew(Disruptor, NULL, NULL);

	if (disruptorID == NULL){
		fprintf(stderr, "disruptorID creation failed!\n");
		while(1);
	}

	ledOutputGreenID = osThreadNew(ledOutputGreen, NULL, NULL);

	if (ledOutputGreenID == NULL){
		fprintf(stderr, "ledOutputGreenID creation failed!\n");
		while(1);
	}

	ledOutputRedID = osThreadNew(ledOutputRed, NULL, NULL);

	if (ledOutputRedID == NULL){
		fprintf(stderr, "ledOutputRedID creation failed!\n");
		while(1);
	}

	timerThreadID = osThreadNew(GameOverTimer, NULL, NULL);

	if (timerThreadID == NULL){
		fprintf(stderr, "timerThreadID creation failed!\n");
		while(1);
	}
	//Mutex
//	ball_collision_mutex = osMutexNew(NULL);
//
//	if (ball_collision_mutex == NULL){
//		fprintf(stderr, "ball_collision_mutex creation failed!\n");
//		while(1);
//	}
//
//	energyStoreMutex = osMutexNew(NULL);
//
//	if (energyStoreMutex == NULL){
//		fprintf(stderr, "energyStoreMutex creation failed!\n");
//		while(1);
//	}

	//initialize ball position struct
	ball_position.x = SCREEN_OFFSET;
	ball_position.y = SCREEN_OFFSET;
	ball_position.angle_x = 0;
	ball_position.angle_y = 0;
	//ball_position.angle = 0;
}

/*
 * Draw maze walls using random number generation
 */
void DrawMaze(){
	//int offset = 10;	//maze offset from screen border draw from top left (px)
	int cellSize = config.maze.cellSize;
	for (int i = 0; i < config.maze.size.height; i++){
		for (int j = 0; j < config.maze.size.width; j++){
//			LCD_DisplayNumber(cellSize * i + SCREEN_OFFSET, cellSize * j + SCREEN_OFFSET, i);
			if (maze[i][j].top){
				LCD_Draw_Horizontal_Line(cellSize * i + SCREEN_OFFSET, cellSize * j + SCREEN_OFFSET, cellSize, LCD_COLOR_BLACK);
				maze[i][j].top_coord[0] = cellSize * i + SCREEN_OFFSET;
				maze[i][j].top_coord[1] = cellSize * j + SCREEN_OFFSET;
			}
			if (maze[i][j].left){
				LCD_Draw_Vertical_Line(cellSize*i + SCREEN_OFFSET, cellSize*j + SCREEN_OFFSET, cellSize, LCD_COLOR_BLACK);
				maze[i][j].left_coord[0] = cellSize*i + SCREEN_OFFSET;
				maze[i][j].left_coord[1] = cellSize*j + SCREEN_OFFSET;
			}
			if (maze[i][j].right){
				LCD_Draw_Vertical_Line(cellSize*i + cellSize + SCREEN_OFFSET, cellSize*j + SCREEN_OFFSET, cellSize, LCD_COLOR_BLACK);
				maze[i][j].right_coord[0] = cellSize*i + cellSize + SCREEN_OFFSET;
				maze[i][j].right_coord[1] = cellSize*j + SCREEN_OFFSET;
			}
			if (maze[i][j].bottom){
				LCD_Draw_Horizontal_Line(cellSize*i + SCREEN_OFFSET, cellSize*j+cellSize + SCREEN_OFFSET, cellSize, LCD_COLOR_BLACK);
				maze[i][j].bottom_coord[0] = cellSize*i + SCREEN_OFFSET;
				maze[i][j].bottom_coord[1] = cellSize*j+cellSize + SCREEN_OFFSET;
			}
			if (maze[i][j].hole){
				LCD_Draw_Circle_Fill(cellSize*i + cellSize/2 + SCREEN_OFFSET, cellSize*j + cellSize/2 + SCREEN_OFFSET, config.maze.holeDiameter/2, LCD_COLOR_BLACK);
				maze[i][j].hole_coord[0] = cellSize*i + cellSize/2 + SCREEN_OFFSET;
				maze[i][j].hole_coord[1] = cellSize*j + cellSize/2 + SCREEN_OFFSET;
			}
		}
	}
}

/*
 * Create the logical maze of cells
 */
void CreateMaze(){
	for (int col = 0; col < config.maze.size.height; col++){
		for (int row = 0; row < config.maze.size.width; row++){
			maze[col][row].top = false;
			maze[col][row].bottom = false;
			maze[col][row].left = false;
			maze[col][row].right = false;
			maze[col][row].hole = false;
		}
	}

	for (int col = 0; col < config.maze.size.height; col++){
		for (int row = 0; row < config.maze.size.width; row++){
			//create cell

			if ((RNG_getNumber() % 1000) < config.maze.obstacleProbability.wall){
				maze[col][row].top = true;
				if (row != 0) maze[col][row - 1].bottom = true;
			}
//			else {
//				maze[col][row].top = false;
//			}
			if ((RNG_getNumber() % 1000) < config.maze.obstacleProbability.wall){
				maze[col][row].left = true;
				if (col != 0) maze[col - 1][row].right = true;
			}
//			else {
//				maze[col][row].left = false;
//			}
			if ((RNG_getNumber() % 1000) < config.maze.obstacleProbability.wall){
				maze[col][row].right = true;
				if (col != config.maze.size.width - 1) maze[col + 1][row].left = true;
			}
//			else {
//				maze[col][row].right = false;
//			}
			if ((RNG_getNumber() % 1000) < config.maze.obstacleProbability.wall){
				maze[col][row].bottom = true;
				if (row != config.maze.size.height - 1) maze[col][row + 1].top = true;
			}
//			else {
//				maze[col][row].bottom = false;
//			}
			if ((RNG_getNumber() % 1000) < config.maze.obstacleProbability.hole && !(row == 0 && col == 0)){
				maze[col][row].hole = true;
			}
//			else {
//				maze[col][row].hole = false;
//			}

			if (config.maze.hardEdged == true){
				if (row == 0){
					maze[col][row].top = true;
				}
				if (col == 0){
					maze[col][row].left = true;
				}
				if (row == config.maze.size.height - 1){
					maze[col][row].bottom = true;
				}
				if (col == config.maze.size.width - 1){
					maze[col][row].right = true;
				}
			}
		}
	}
}

/*
 * callback func for integrating angle at the discrete time
 */
void BallPosition(){
	//if hard edged maze boundries at
	double drone_radius = config.drone.diameter/2;
	while(1){

		double curr_x = ball_position.x;
		double curr_y = ball_position.y;
		double angle_x = ball_position.angle_x;
		double angle_y = ball_position.angle_y;

		//Cell curr_cell

		angle_y += (Gyro_Get_Velocity_Y() * (1.0 / (double)config.physics.updateFrequency)) * ((double)config.physics.angleGain / 1000.0);
		angle_x += (Gyro_Get_Velocity_X() * (1.0 / (double)config.physics.updateFrequency)) * ((double)config.physics.angleGain / 1000.0);

		if (angle_y >= 90 || angle_x >= 90){
			//game failure
			GameOverFell();
		}

		double magnitude = sqrt(angle_x*angle_x + angle_y*angle_y);	//magnitude of ball movement
		double direction = atan2(angle_y, angle_x); //direction of movement in rad

//			double delta_x = magnitude * cos(direction) * ((config.physics.gravity / 100.0) * sin(angle_x));
//			double delta_y = magnitude * sin(direction) * ((config.physics.gravity / 100.0) * sin(angle_y));

		double delta_x = magnitude * cos(direction);
		double delta_y = magnitude * sin(direction);

		double next_x = curr_x + delta_x;
		double next_y = curr_y + delta_y;

		//maze boundaries
		if (config.maze.hardEdged){
			if (next_x > (config.maze.cellSize * config.maze.size.width - drone_radius + SCREEN_OFFSET)){ //right side
				next_x = config.maze.cellSize * config.maze.size.width - drone_radius + SCREEN_OFFSET;
			} else if (next_x < (SCREEN_OFFSET + drone_radius)){
				next_x = drone_radius + SCREEN_OFFSET; //left side
			}
			if (next_y > (config.maze.cellSize * config.maze.size.height - drone_radius + SCREEN_OFFSET)){ //bottom
				next_y = config.maze.cellSize * config.maze.size.height - drone_radius + SCREEN_OFFSET;
			} else if (next_y < (SCREEN_OFFSET + drone_radius)){
				next_y = drone_radius + SCREEN_OFFSET; //top
			}
		}
		if (!config.maze.hardEdged){ //not hard edged
			if (next_x > (config.maze.cellSize * config.maze.size.width - drone_radius + SCREEN_OFFSET)){
				//game fail
				GameOverFell();
			}
			if (next_y > (config.maze.cellSize * config.maze.size.height - drone_radius + SCREEN_OFFSET)){
				//game fail
				GameOverFell();
			}
		}

		//osMutexAcquire(ball_collision_mutex, osWaitForever);
		bool ball_collision_loc = ball_collision;
		//osMutexRelease(ball_collision_mutex);

		if (ball_collision_loc){
			double hole_radius = config.maze.holeDiameter/2.0;
			//wall and hole collision
			//Cell current_cell = maze[(int)((curr_y - SCREEN_OFFSET) / config.maze.cellSize)][(int)((curr_x - SCREEN_OFFSET) / config.maze.cellSize)];
//			int index[2] = {(int)((next_y - SCREEN_OFFSET) / config.maze.cellSize), (int)((next_x - SCREEN_OFFSET) / config.maze.cellSize)};
//			//Cell next_cell = maze[next_cell_index[0]][next_cell_index[1]];
//	//
//	//		int curr_cell_index[2] = {(int)((curr_y - SCREEN_OFFSET) / config.maze.cellSize), (int)((curr_x - SCREEN_OFFSET) / config.maze.cellSize)};
//	//		Cell curr_cell = maze[curr_cell_index[0]][curr_cell_index[1]];
//
//			Cell maze_slice[3][3];
//			for (int i = 0; i < 3; i++) {
//			    for (int j = 0; j < 3; j++) {
//			        int row_index = index[0] - 1 + i;
//			        int col_index = index[1] - 1 + j;
//
//			        if (row_index < 0 || row_index > config.maze.size.height - 1 || col_index < 0 || col_index > config.maze.size.width - 1) {
//			            maze_slice[i][j].empty = true; // Out of bounds, set to NULL
//			        } else {
//			            maze_slice[i][j] = maze[col_index][row_index]; // Assign the corresponding cell from the maze
//			        }
//			    }
//			}
//
//			//DETECT BALL HITTING MAZE WALLS HERE
//			for (int i = 0; i < 3; i++) {
//			    for (int j = 0; j < 3; j++) {
//			        // Check if next_x and next_y intersect with any walls of the current cell
////			    	int next_cell_index[2] = {(int)((next_y - SCREEN_OFFSET) / config.maze.cellSize), (int)((next_x - SCREEN_OFFSET) / config.maze.cellSize)};
////			    	int i = next_cell_index[0];
////			    	int j = next_cell_index[1]
//			    	Cell cell = maze_slice[i][j];
//			    	if (!cell.empty){
//						int ball_x1 = next_x - 5;
//						int ball_y1 = next_y - 5;
//						int ball_x2 = next_x + 5;
//						int ball_y2 = next_y + 5;
//
//						if (cell.top){
//							if (ball_y1 < cell.top_coord[1] && ball_y2 > cell.top_coord[1]) next_y = cell.top_coord[1] + drone_radius;
//						}
//						if (cell.left){
//							if (ball_x1 < cell.left_coord[0] && ball_x2 > cell.left_coord[0]) next_x = cell.left_coord[0] + drone_radius;
//						}
//						if (cell.right){
//							if (ball_x2 > cell.right_coord[0] && ball_x1 < cell.right_coord[0]) next_x = cell.right_coord[0] - drone_radius;
//						}
//						if (cell.bottom){
//							if (ball_y2 > cell.bottom_coord[1] && ball_y1 < cell.bottom_coord[1]) next_y = cell.bottom_coord[1] - drone_radius;
//						}
//			    	}
//			    }
//			}

			//handle circle
			double distance;
			double sumRadii = (drone_radius + hole_radius) * 0.5;
			for (int col = 0; col < config.maze.size.width; col++){
				for (int row = 0; row < config.maze.size.width; row++){
					distance = sqrt(pow(next_y - maze[col][row].hole_coord[0], 2) + pow(next_x - maze[col][row].hole_coord[1], 2));
					if (distance < sumRadii) {
						// Ball has fallen into the hole
						GameOverFell();
					}
				}
			}
		}


		//check waypoint 1
		static int num_waypoints_hit = 0;
		if (num_waypoints_hit == 0){
			LCD_Draw_Circle_Fill(config.maze.waypoints.location[0][1]  + SCREEN_OFFSET, config.maze.waypoints.location[0][0] + SCREEN_OFFSET, config.maze.waypoints.diameter/2, LCD_COLOR_YELLOW);
		}
		double sumRadii = drone_radius + config.maze.waypoints.diameter/2;
		double distance = sqrt(pow(next_y - (config.maze.waypoints.location[0][0] + SCREEN_OFFSET), 2) + pow(next_x - (config.maze.waypoints.location[0][1]  + SCREEN_OFFSET), 2));
		if (distance < sumRadii && num_waypoints_hit == 0){
			LCD_DisplayNumber(10, 240, 1);
			num_waypoints_hit = 1;
			LCD_Draw_Circle_Fill(config.maze.waypoints.location[0][1]  + SCREEN_OFFSET, config.maze.waypoints.location[0][0] + SCREEN_OFFSET, config.maze.waypoints.diameter/2, LCD_COLOR_WHITE);
			LCD_Draw_Circle_Fill(config.maze.waypoints.location[1][0]  + SCREEN_OFFSET, config.maze.waypoints.location[1][1] + SCREEN_OFFSET, config.maze.waypoints.diameter/2, LCD_COLOR_YELLOW);
		}
		//check waypoint 2
		sumRadii = drone_radius + config.maze.waypoints.diameter/2;
		distance = sqrt(pow(next_y - (config.maze.waypoints.location[1][0] + SCREEN_OFFSET), 2) + pow(next_x - (config.maze.waypoints.location[1][1] + SCREEN_OFFSET), 2));
		if (distance < sumRadii && num_waypoints_hit == 1){
			LCD_DisplayNumber(30, 240, 2);
			num_waypoints_hit = 2;
			LCD_Draw_Circle_Fill(config.maze.waypoints.location[1][0]  + SCREEN_OFFSET, config.maze.waypoints.location[1][1] + SCREEN_OFFSET, config.maze.waypoints.diameter/2, LCD_COLOR_WHITE);
			LCD_Draw_Circle_Fill(config.maze.waypoints.location[2][0]  + SCREEN_OFFSET, config.maze.waypoints.location[2][1] + SCREEN_OFFSET, config.maze.waypoints.diameter/2, LCD_COLOR_YELLOW);

		}
		//check waypoint 3
		sumRadii = drone_radius + config.maze.waypoints.diameter/2;
		distance = sqrt(pow(next_y - (config.maze.waypoints.location[2][0] + SCREEN_OFFSET), 2) + pow(next_x - (config.maze.waypoints.location[2][1] + SCREEN_OFFSET), 2));
		if (distance < sumRadii && num_waypoints_hit == 2){
			LCD_DisplayNumber(50, 240, 3);
			num_waypoints_hit = 3;
			LCD_Draw_Circle_Fill(config.maze.waypoints.location[2][0]  + SCREEN_OFFSET, config.maze.waypoints.location[2][1] + SCREEN_OFFSET, config.maze.waypoints.diameter/2, LCD_COLOR_WHITE);
			LCD_Draw_Circle_Fill(config.maze.waypoints.location[3][0]  + SCREEN_OFFSET, config.maze.waypoints.location[3][1] + SCREEN_OFFSET, config.maze.waypoints.diameter/2, LCD_COLOR_YELLOW);
		}
		//check waypoint 4
		sumRadii = drone_radius + config.maze.waypoints.diameter/2;
		distance = sqrt(pow(next_y - (config.maze.waypoints.location[3][0] + SCREEN_OFFSET), 2) + pow(next_x - (config.maze.waypoints.location[3][1] + SCREEN_OFFSET), 2));
		if (distance < sumRadii && num_waypoints_hit == 3){
			LCD_DisplayNumber(70, 240, 4);
			num_waypoints_hit = 4;
			GameWin(num_waypoints_hit);
			//game win
		}

		//draw new ball to screen
		LCD_Draw_Circle_Fill(curr_y, curr_x, drone_radius, LCD_COLOR_WHITE);
		DrawMaze();
		LCD_Draw_Circle_Fill(next_y, next_x, drone_radius, LCD_COLOR_BLUE);

		//update struct values
		ball_position.x = next_x;
		ball_position.y = next_y;
		ball_position.angle_x = angle_x;
		ball_position.angle_y = angle_y;

		//update frequency delay
		osDelay(1000/config.physics.updateFrequency);
	}
}

void Disruptor(){
	while(1){
		if (HAL_GPIO_ReadPin(GPIOA, BUTTON_PIN) == GPIO_PIN_SET && energy_store >= config.drone.disruptor.minActivationEnergy){
			//osMutexAcquire(ball_collision_mutex, osWaitForever);
			ball_collision = false;
			//osMutexRelease(ball_collision_mutex);

			//osMutexAcquire(energyStoreMutex, osWaitForever);
			energy_store = 0;
			//osMutexRelease(energyStoreMutex);

			osDelay(config.drone.disruptor.maxTime);

		} else {
			//osMutexAcquire(ball_collision_mutex, osWaitForever);
			ball_collision = true;
			//osMutexRelease(ball_collision_mutex);
		}

		osDelay(1000);
		//osMutexAcquire(energyStoreMutex, osWaitForever);
		energy_store += config.drone.energyStore.rechargeRate;
		//osMutexRelease(energyStoreMutex);
	}
}

void ledOutputGreen(){
	int max_energy = config.drone.energyStore.maxEnergy;
	int energy_store_loc;
	double percent_energy;
	int delay;
	while(1){
		//osMutexAcquire(energyStoreMutex, osWaitForever);
		energy_store_loc = energy_store;
		//osMutexRelease(energyStoreMutex);

		if (energy_store_loc < config.drone.disruptor.minActivationEnergy){
			percent_energy = ((double)energy_store_loc / (double)max_energy) * 100;
			delay = (int)percent_energy;

			HAL_GPIO_WritePin(GPIOG, GREEN_LED_PIN, GPIO_PIN_SET);
			osDelay(delay);
			HAL_GPIO_WritePin(GPIOG, GREEN_LED_PIN, GPIO_PIN_RESET);
			osDelay(delay);
		} else {
			HAL_GPIO_WritePin(GPIOG, GREEN_LED_PIN, GPIO_PIN_SET);
		}
	}
}

void ledOutputRed(){
	int energy_store_loc;
	while(1){
		energy_store_loc = energy_store;

		if (energy_store_loc < config.drone.disruptor.minActivationEnergy){
			int time_remaining = (config.drone.disruptor.minActivationEnergy - energy_store_loc) / config.drone.energyStore.rechargeRate;
			int period = time_remaining / 10;
			if (period < 2) period = 2;

			HAL_GPIO_WritePin(GPIOG, RED_LED_PIN, GPIO_PIN_SET);
			osDelay(period / 2);

			HAL_GPIO_WritePin(GPIOG, RED_LED_PIN, GPIO_PIN_RESET);
			osDelay(period / 2);
		} else {
			HAL_GPIO_WritePin(GPIOG, RED_LED_PIN, GPIO_PIN_RESET);
		}
	}
}

void GameOverFell(){
	LCD_Clear(0, LCD_COLOR_RED);

	LCD_DisplayString(42, 120, "Game Over!");
	LCD_DisplayString(20, 150, "The Ball Fell");

	game_over = true;

	osThreadTerminate(ledOutputRedID);
	osThreadTerminate(ledOutputGreenID);
	osThreadTerminate(ballPositionID);
	osThreadTerminate(disruptorID);
	osThreadTerminate(timerThreadID);
}

void GameOverTimer(){
	LCD_DisplayString(10, 270, "Time:");
	for (int i = config.maze.timeToComplete; i >= 0; i-= 1000){
		time = i;
		LCD_DisplayNumber(90, 270, (i/1000));
		osDelay(1000);
		if (game_over == true) return;
		LCD_SetTextColor(LCD_COLOR_WHITE);
		LCD_DisplayNumber(90, 270, (i/1000));
		LCD_SetTextColor(LCD_COLOR_BLACK);
	}

	//timer expired
	LCD_Clear(0, LCD_COLOR_RED);

	LCD_DisplayString(42, 120, "Game Over!");
	LCD_DisplayString(20, 150, "Time Ran Out");

	osThreadTerminate(ledOutputRedID);
	osThreadTerminate(ledOutputGreenID);
	osThreadTerminate(ballPositionID);
	osThreadTerminate(disruptorID);
	osThreadTerminate(timerThreadID);
}

void GameWin(int num_waypoints){
	game_over = true;
	LCD_Clear(0, LCD_COLOR_WHITE);
	int elapsed_time_sec = (config.maze.timeToComplete - time) / 1000;
	uint16_t score = elapsed_time_sec;

	LCD_DisplayString(47, 120, "You Won!");
	LCD_DisplayString(45, 150, "Score:");
	LCD_DisplayNumber(135, 150, score);

	osThreadTerminate(ledOutputRedID);
	osThreadTerminate(ledOutputGreenID);
	osThreadTerminate(ballPositionID);
	osThreadTerminate(disruptorID);
	osThreadTerminate(timerThreadID);
}
