/*
 * motor.h
 *
 *  Created on: Mar 27, 2025
 *      Author: knn64
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define MAX_SPEED 255
#define MAX_ALPHA 220

#define DIRECTION_FORWARD "F"
#define DIRECTION_LEFT "L"
#define DIRECTION_RIGHT "R"
#define DIRECTION_BACKWARD "B"

#define POLARITY_POSITIVE 0
#define POLARITY_NEGATIVE 1

#include <stdio.h>
#include <stdint.h>
#include "tim.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"



typedef enum {
	FORWARD,
	LEFT,
	RIGHT,
	BACKWARD,
} DIRECTION;

typedef struct {
	uint8_t motor_id;
	TIM_HandleTypeDef *TIM_motor;
	uint32_t TIM_CH_motor_high;
	uint32_t TIM_CH_motor_low;
} motor_t;

typedef struct {
	float current_angle; // Orientation actuelle (0-360°)
	uint8_t target_direction; // Direction cible (0-255)
	uint8_t target_speed; // Vitesse cible (0-255)
	uint32_t step_counter; // Compteur d'itérations
	volatile uint8_t new_command; // Drapeau pour nouvelle commande
	int16_t current_left_speed; // Vitesse actuelle appliquée (gauche)
	int16_t current_right_speed; // Vitesse actuelle appliquée (droite)
} RobotState;


typedef struct {
	uint8_t direction;
	uint8_t speed;
	volatile uint8_t valid; // 1 si nouvelle commande valide
} CommandBuffer;

void motor_test(int delay,int speed);
void motor_setSpeed(motor_t motor,uint16_t speed,int polarity);
void motor_setVehiculeDirection(DIRECTION dir,int speed);
void control_wheels(uint8_t direction, uint8_t speed, int16_t* left_speed, int16_t* right_speed);
void set_robot_motion(uint8_t direction, uint8_t speed);
void control_wheels_with_time(RobotState* state, int16_t* left_speed, int16_t* right_speed);
float shortest_angle_diff(float target, float current);
float normalize_angle(float angle);




#endif /* INC_MOTOR_H_ */
