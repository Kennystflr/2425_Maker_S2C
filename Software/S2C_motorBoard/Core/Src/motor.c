/*
 * Motors.c
 *
 *  Created on: Mar 27, 2025
 *      Author: knn64
 */

#include "motor.h"
#include "stm32g4xx_hal_tim.h"
#include <stdint.h>
#include <math.h>

// Constants
#define PI 3.14159265359
#define MAX_ROTATION_SPEED 45.0 // Degrés par seconde (à ajuster)
#define CONTROL_PERIOD 0.01     // Période de contrôle (10 ms)
#define ADJUSTMENT_PERIOD 0.2   // Période d'ajustement (2 s)
#define STEPS_PER_ADJUSTMENT (uint32_t)(ADJUSTMENT_PERIOD / CONTROL_PERIOD)
#define RIGHT_SPEED_COMPENSATION 1.1 // Facteur pour corriger le virage à droite (ajuster après tests)
#define MAX_SPEED_CHANGE_PER_STEP 5 // Variation max de vitesse par 10 ms (500 unités/s)

static motor_t m1 ={1,&htim3,TIM_CHANNEL_2,TIM_CHANNEL_1};
static motor_t m2 ={2,&htim2,TIM_CHANNEL_2,TIM_CHANNEL_1};
static motor_t m3 ={3,&htim2,TIM_CHANNEL_3,TIM_CHANNEL_4};
static motor_t m4 ={4,&htim4,TIM_CHANNEL_2,TIM_CHANNEL_1};


RobotState robot_state = {0.0, 0, 0, 0, 0, 0, 0};
CommandBuffer command_buffer = {0, 0, 0};
extern TIM_HandleTypeDef htim7; // Timer pour 10 ms


float normalize_angle(float angle) {
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0) angle += 360.0;
    return angle;
}



float shortest_angle_diff(float target, float current) {
    float diff = target - current;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    return diff;
}


void motor_setSpeed(motor_t motor,uint16_t speed,int polarity){
	if((speed<=MAX_SPEED)&(speed!=0)){
		if(polarity==POLARITY_POSITIVE){
			__HAL_TIM_SET_COMPARE(motor.TIM_motor, motor.TIM_CH_motor_high, speed);
			__HAL_TIM_SET_COMPARE(motor.TIM_motor, motor.TIM_CH_motor_low, MAX_SPEED - speed);
		}
		if(polarity==POLARITY_NEGATIVE){
			__HAL_TIM_SET_COMPARE(motor.TIM_motor, motor.TIM_CH_motor_high, MAX_SPEED-speed);
			__HAL_TIM_SET_COMPARE(motor.TIM_motor, motor.TIM_CH_motor_low, speed);
		}
	}
	else {
		__HAL_TIM_SET_COMPARE(motor.TIM_motor, motor.TIM_CH_motor_high, 125);
		__HAL_TIM_SET_COMPARE(motor.TIM_motor, motor.TIM_CH_motor_low, 125);
	}

}


void motor_setVehiculeDirection(DIRECTION dir,int speed){
	if(speed<= MAX_SPEED){
		switch(dir){
		case(FORWARD):
					motor_setSpeed(m1,speed,POLARITY_POSITIVE);
		motor_setSpeed(m2,speed,POLARITY_POSITIVE);
		motor_setSpeed(m3,speed,POLARITY_POSITIVE);
		motor_setSpeed(m4,speed,POLARITY_POSITIVE);
		break;
		case(BACKWARD):
					motor_setSpeed(m1,speed,POLARITY_NEGATIVE);
		motor_setSpeed(m2,speed,POLARITY_NEGATIVE);
		motor_setSpeed(m3,speed,POLARITY_NEGATIVE);
		motor_setSpeed(m4,speed,POLARITY_NEGATIVE);
		break;
		case(LEFT):
					motor_setSpeed(m1,speed,POLARITY_NEGATIVE);
		motor_setSpeed(m2,speed,POLARITY_POSITIVE);
		motor_setSpeed(m3,speed,POLARITY_NEGATIVE);
		motor_setSpeed(m4,speed,POLARITY_POSITIVE);
		break;
		case(RIGHT):
					motor_setSpeed(m1,speed,POLARITY_POSITIVE);
		motor_setSpeed(m2,speed,POLARITY_NEGATIVE);
		motor_setSpeed(m3,speed,POLARITY_POSITIVE);
		motor_setSpeed(m4,speed,POLARITY_NEGATIVE);
		break;
		}

	}
}

void apply_motor_speeds(int16_t left_speed, int16_t right_speed) {
    float right_speed_compensated = (float)right_speed * RIGHT_SPEED_COMPENSATION;

    if (right_speed_compensated > MAX_SPEED) right_speed_compensated = MAX_SPEED;
    if (right_speed_compensated < -MAX_SPEED) right_speed_compensated = -MAX_SPEED;

    uint16_t left_speed_abs = (uint16_t)(left_speed >= 0 ? left_speed : -left_speed);
    uint16_t right_speed_abs = (uint16_t)(right_speed_compensated >= 0 ? right_speed_compensated : -right_speed_compensated);
    int left_polarity = (left_speed >= 0) ? POLARITY_POSITIVE : POLARITY_NEGATIVE;
    int right_polarity = (right_speed_compensated >= 0) ? POLARITY_POSITIVE : POLARITY_NEGATIVE;

    motor_setSpeed(m1, left_speed_abs, left_polarity);
    motor_setSpeed(m2, left_speed_abs, left_polarity);
    motor_setSpeed(m3, right_speed_abs, right_polarity);
    motor_setSpeed(m4, right_speed_abs, right_polarity);
}


//################
// Function to control differential drive with 4 wheels (tank-like)
// Input: direction (0-360 degrees, 8-bit), speed (0-255, 8-bit)
// Output: wheel speeds for left and right sides (positive for forward, negative for backward)
void control_wheels_with_time(RobotState* state, int16_t* left_speed, int16_t* right_speed) {
    // Convertir direction 8 bits (0-255) en angle (0-360°)
    float target_angle = ((float)state->target_direction * 360.0) / 256.0;
    float angle_diff = shortest_angle_diff(target_angle, state->current_angle);

    // Calculer la vitesse de rotation
    float rotation_speed = angle_diff / ADJUSTMENT_PERIOD;
    if (fabs(rotation_speed) > MAX_ROTATION_SPEED) {
        rotation_speed = (rotation_speed > 0) ? MAX_ROTATION_SPEED : -MAX_ROTATION_SPEED;
    }

    // Mettre à jour l'angle actuel
    state->current_angle = normalize_angle(state->current_angle + rotation_speed * CONTROL_PERIOD);

    // Si l'angle est presque atteint, avancer dans la direction cible
    if (fabs(angle_diff) < 5.0) {
        float rad = target_angle * PI / 180.0;
        float forward_component = cos(rad);
        float turn_component = sin(rad);

        float left = state->target_speed * (forward_component + turn_component);
        float right = state->target_speed * (forward_component - turn_component);

        *left_speed = (int16_t)(left * MAX_SPEED / 255.0);
        *right_speed = (int16_t)(right * MAX_SPEED / 255.0);
    } else {
        // Pivoter sur place
        float turn_speed = rotation_speed / MAX_ROTATION_SPEED * MAX_SPEED;
        *left_speed = (int16_t)(-turn_speed);
        *right_speed = (int16_t)(turn_speed);
    }

    // Clamp des vitesses
    if (*left_speed > MAX_SPEED) *left_speed = MAX_SPEED;
    if (*left_speed < -MAX_SPEED) *left_speed = -MAX_SPEED;
    if (*right_speed > MAX_SPEED) *right_speed = MAX_SPEED;
    if (*right_speed < -MAX_SPEED) *right_speed = -MAX_SPEED;
}
// Example usage

// Limiter les changements de vitesse, y compris les changements de sens
void limit_speed_ramp(RobotState* state, int16_t target_left_speed, int16_t target_right_speed,
                      int16_t* applied_left_speed, int16_t* applied_right_speed) {
    // Gauche
    int16_t left_diff = target_left_speed - state->current_left_speed;
    // Changement de sens si signes opposés
    if (state->current_left_speed * target_left_speed < 0) {
        // Aller vers 0
        if (fabs(state->current_left_speed) > MAX_SPEED_CHANGE_PER_STEP) {
            state->current_left_speed += (state->current_left_speed > 0) ? -MAX_SPEED_CHANGE_PER_STEP : MAX_SPEED_CHANGE_PER_STEP;
        } else {
            state->current_left_speed = 0; // Atteint 0, prêt pour le nouveau sens
        }
    } else {
        // Même sens ou à partir de 0
        if (fabs(left_diff) > MAX_SPEED_CHANGE_PER_STEP) {
            state->current_left_speed += (left_diff > 0) ? MAX_SPEED_CHANGE_PER_STEP : -MAX_SPEED_CHANGE_PER_STEP;
        } else {
            state->current_left_speed = target_left_speed;
        }
    }

    // Droite
    int16_t right_diff = target_right_speed - state->current_right_speed;
    // Changement de sens si signes opposés
    if (state->current_right_speed * target_right_speed < 0) {
        // Aller vers 0
        if (fabs(state->current_right_speed) > MAX_SPEED_CHANGE_PER_STEP) {
            state->current_right_speed += (state->current_right_speed > 0) ? -MAX_SPEED_CHANGE_PER_STEP : MAX_SPEED_CHANGE_PER_STEP;
        } else {
            state->current_right_speed = 0; // Atteint 0, prêt pour le nouveau sens
        }
    } else {
        // Même sens ou à partir de 0
        if (fabs(right_diff) > MAX_SPEED_CHANGE_PER_STEP) {
            state->current_right_speed += (right_diff > 0) ? MAX_SPEED_CHANGE_PER_STEP : -MAX_SPEED_CHANGE_PER_STEP;
        } else {
            state->current_right_speed = target_right_speed;
        }
    }

    *applied_left_speed = state->current_left_speed;
    *applied_right_speed = state->current_right_speed;
}

void set_robot_motion(uint8_t direction, uint8_t speed) {
    int16_t left_speed, right_speed;

    control_wheels(direction, speed, &left_speed, &right_speed);


    motor_setVehiculeDirection(direction,speed);
    /*motor_setSpeed(m3, left_speed,1);
    motor_setSpeed(m2, right_speed,0);
    motor_setSpeed(m4, right_speed,0);*/

}


// Callback du timer de 10ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        int16_t target_left_speed, target_right_speed;
        int16_t applied_left_speed, applied_right_speed;

        // Calculer les vitesses cibles
        control_wheels_with_time(&robot_state, &target_left_speed, &target_right_speed);

        // Appliquer la rampe de vitesse
        limit_speed_ramp(&robot_state, target_left_speed, target_right_speed,
                         &applied_left_speed, &applied_right_speed);

        // Appliquer les vitesses limitées
        apply_motor_speeds(applied_left_speed, applied_right_speed);

        robot_state.step_counter++;
        if (robot_state.step_counter >= STEPS_PER_ADJUSTMENT) {
            robot_state.step_counter = 0;
            if (command_buffer.valid) {
                robot_state.target_direction = command_buffer.direction;
                robot_state.target_speed = command_buffer.speed;
                command_buffer.valid = 0;
            }
        }
    }
}


void motor_test(int delay,int speed){
	motor_setVehiculeDirection(FORWARD,speed);
	HAL_Delay(delay);
	motor_setVehiculeDirection(FORWARD,0);
}


