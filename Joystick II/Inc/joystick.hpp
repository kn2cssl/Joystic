//joystic 

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include "NRF24L01.hpp"
#include <stdbool.h>
#include <stdio.h>
#include "usbd_cdc_if.h"

#define OFFSET 2053
#define DOFFSET 300
#define DATANUMBER 15
#define SHOOTTIMER 100

//X = 8
//square = 7
//circle = 9
//triangle = 10
//right = 4
//left = 3
//down = 5
//up = 2
//L2 = 0
//L1 = 1
//R1 = 11
//R2 = 12
enum KEYS
{
	X_KEY = 8,
	SQUARE_KEY = 7,
	CIRCLE_KEY = 9,
	TRIANGLE_KEY = 10,
	RIGHT_KEY = 4,
	LEFT_KEY = 3,
	DOWN_KEY = 5,
	UP_KEY = 2,
	L2_KEY = 0,
	L1_KEY = 1,
	R1_KEY = 11,
	R2_KEY = 12
};
class Joystick
{
	private:
		NRF* hnrf;
		ADC_HandleTypeDef* hadc;
		TIM_HandleTypeDef* htim;
		unsigned long int XChannel;
		unsigned long int YChannel;
		unsigned long int WChannel;
		unsigned long int PowerChannel;
		signed long int XOffset;
		signed long int YOffset;
		signed long int WOffset;
		signed long int PowerOffset;
		unsigned long int shootUpdated;
		void NRFInit(void);
		void changeChannel(unsigned long int channel);
		void getOffsets(void);
		bool Data_Ready(void);
	public:
		unsigned char keys[13];
		signed char buffer[32];
		unsigned char response[32];
		float speedConstant;
		Joystick();
		Joystick(NRF* nrf,ADC_HandleTypeDef* ADC,TIM_HandleTypeDef* htim,unsigned long int x,unsigned long int y,unsigned long int w,unsigned long int p);
		void init(NRF* nrf,ADC_HandleTypeDef* ADC,TIM_HandleTypeDef* htim,unsigned long int x,unsigned long int y,unsigned long int w,unsigned long int p);
		void changeRobot(unsigned char robotID);
		signed int getX(void);
		signed int getY(void);
		signed int getW(void);
		signed int getPower(void);
		void setX(signed int speed);
		void setY(signed int speed);
		void setW(signed int speed);
		void setPower(signed int speed);
		void update(void);
		void Key_Changed(void);
		void Read_Keys(void);
		void Do_Keys(void);
		void Time_Passed(TIM_HandleTypeDef* htim);
};

#endif
