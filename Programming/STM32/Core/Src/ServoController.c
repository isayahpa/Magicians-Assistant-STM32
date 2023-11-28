#include "ServoController.h"

void initServoController(ServoController *pCtrl, Servo *servoList[]){
	printf("Initializing Servo Controller\n");
	pCtrl->servoList = servoList;
	pCtrl->status = HAL_OK;

	//TODO:Will need to fix the indexing of the Servo list to be safer
	int i = 0;
	Servo *pServo = pCtrl->servoList[0];
	while(pServo != NULL){
		pServo = pCtrl -> servoList[i];
		pCtrl->status = HAL_TIM_PWM_Start(pServo->pTIMHandle, pServo->channel);
		if(pCtrl->status != HAL_OK){
			printf("Failed to Start PWM for Servo #%lu | Status : %d\n", pServo->channel, pCtrl->status);
			break;
		}
		i++;
	}
	resetAllServos(pCtrl);
}

void resetAllServos(ServoController *pCtrl){
	printf("Resetting Servos\n");

	int i = 0;
	Servo *pServo = pCtrl->servoList[0];
	while(pServo != NULL){
		Servo *pServo = pCtrl->servoList[i];
		pCtrl->status = setServoPosition(pServo, SERVO_POS_CENTER);
		if(pCtrl->status != HAL_OK){
			printf("Failed to Reset Servo #%lu\n | Status : %d", pServo->channel, pCtrl->status);
			break;
		}
		i++;
	}
}

//https://www.youtube.com/watch?v=AjN58ceQaF4
//http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf
//Position in degrees
HAL_StatusTypeDef setServoPosition(Servo* pServo, int position){
	// TODO: Learn the PWM part to change servo position
	//Just a guess
	printf("Setting Servo #%lu to %d\n", pServo->channel, position);
	TIM_OC_InitTypeDef sConfigOC = {0};
	uint32_t positionAsPulse = 0;
	if(position == SERVO_POS_LEFT){
		positionAsPulse = 2800;
	} else if(position == SERVO_POS_CENTER){
		positionAsPulse = 4200;
	} else {
		positionAsPulse = 5600;
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = positionAsPulse; //TODO: Period = 140000 rn, see if this makes Servo pulsewidth 1.5ms (0 position)
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	return HAL_TIM_PWM_ConfigChannel(pServo->pTIMHandle, &sConfigOC, pServo->channel);

}

