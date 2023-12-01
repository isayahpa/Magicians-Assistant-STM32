#ifndef INC_SERVOCONTROLLER_H_
#define INC_SERVOCONTROLLER_H_

#include "stm32l4xx_hal.h"
#include "helpers.h"

#define SERVO_POS_LEFT -90
#define SERVO_POS_CENTER 0
#define SERVO_POS_RIGHT 90

typedef struct Servo Servo;
struct Servo {
	TIM_HandleTypeDef* pTIMHandle;
	uint32_t channel;
};

typedef struct ServoController ServoController;
struct ServoController{
	TIM_HandleTypeDef* pTIMHandle;
	Servo** servoList;
	HAL_StatusTypeDef status;
};

//Initializers
void initServoController(ServoController*, Servo*[]);
void resetAllServos(ServoController *pCtrl);
HAL_StatusTypeDef setServoPosition(Servo* pServo, int position);

#endif /* INC_SERVOCONTROLLER_H_ */
