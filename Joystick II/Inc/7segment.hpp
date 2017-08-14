#ifndef __7SEGMENT_HPP__
#define __7SEGMENT_HPP__
#include "stm32f1xx_hal.h"
#include <stdbool.h>

class Segment
{
	public:
		GPIO_TypeDef* port;
		unsigned int pin;
		void on(void);
		void off(void);
};

class Display
{
	private:
		Segment segment[7];
		bool data[16][7];
		void setData(void);
	public:
		Display(void);
		void show(unsigned char number);
		void clear(void);
};

#endif
