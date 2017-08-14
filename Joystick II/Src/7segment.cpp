#include "7segment.hpp"

void Segment::on(void)
{
	HAL_GPIO_WritePin(Segment::port,Segment::pin,GPIO_PIN_SET);
}
void Segment::off(void)
{
	HAL_GPIO_WritePin(Segment::port,Segment::pin,GPIO_PIN_RESET);
}

Display::Display(void)
{
	//top segment
	Display::segment[0].port = GPIOB;
	Display::segment[0].pin = GPIO_PIN_6;
	//top-right segment
	Display::segment[1].port = GPIOC;
	Display::segment[1].pin = GPIO_PIN_14;
	//bottom-right segment
	Display::segment[2].port = GPIOC;
	Display::segment[2].pin = GPIO_PIN_15;
	//bottom segment
	Display::segment[3].port = GPIOC;
	Display::segment[3].pin = GPIO_PIN_13;
	//bottom-left segment
	Display::segment[4].port = GPIOB;
	Display::segment[4].pin = GPIO_PIN_9;
	//top-left segment
	Display::segment[5].port = GPIOB;
	Display::segment[5].pin = GPIO_PIN_7;
	//middle segment
	Display::segment[6].port = GPIOB;
	Display::segment[6].pin = GPIO_PIN_8;
	Display::clear();
	Display::setData();
}
void Display::clear(void)
{
	for (unsigned char i = 0;i < 7 ; i++)
		Display::segment[i].off();	
}
void Display::setData(void)
{
	for (unsigned char i = 0; i < 16; i++)
	{
		for (unsigned char j = 0; j < 8; j++)
		{
			Display::data[i][j] = false;
		}
	}
	//0
	Display::data[0][0] = true;
	Display::data[0][1] = true;
	Display::data[0][2] = true;
	Display::data[0][3] = true;
	Display::data[0][4] = true;
	Display::data[0][5] = true;
	//1
	Display::data[1][1] = true;
	Display::data[1][2] = true;
	//2
	Display::data[2][0] = true;
	Display::data[2][1] = true;
	Display::data[2][3] = true;
	Display::data[2][4] = true;
	Display::data[2][6] = true;
	//3
	Display::data[3][0] = true;
	Display::data[3][1] = true;
	Display::data[3][2] = true;
	Display::data[3][3] = true;
	Display::data[3][6] = true;
	//4
	Display::data[4][1] = true;
	Display::data[4][2] = true;
	Display::data[4][5] = true;
	Display::data[4][6] = true;
	//5
	Display::data[5][0] = true;
	Display::data[5][2] = true;
	Display::data[5][3] = true;
	Display::data[5][5] = true;
	Display::data[5][6] = true;
	//6
	Display::data[6][0] = true;
	Display::data[6][2] = true;
	Display::data[6][3] = true;
	Display::data[6][4] = true;
	Display::data[6][5] = true;
	Display::data[6][6] = true;
	//7
	Display::data[7][0] = true;
	Display::data[7][1] = true;
	Display::data[7][2] = true;
	//8
	Display::data[8][0] = true;
	Display::data[8][1] = true;
	Display::data[8][2] = true;
	Display::data[8][3] = true;
	Display::data[8][4] = true;
	Display::data[8][5] = true;
	Display::data[8][6] = true;
	//9
	Display::data[9][0] = true;
	Display::data[9][1] = true;
	Display::data[9][2] = true;
	Display::data[9][3] = true;
	Display::data[9][5] = true;
	Display::data[9][6] = true;
	//A
	Display::data[10][0] = true;
	Display::data[10][1] = true;
	Display::data[10][2] = true;
	Display::data[10][4] = true;
	Display::data[10][5] = true;
	Display::data[10][6] = true;
	//b
	Display::data[11][2] = true;
	Display::data[11][3] = true;
	Display::data[11][4] = true;
	Display::data[11][5] = true;
	Display::data[11][6] = true;
	//c
	Display::data[12][3] = true;
	Display::data[12][4] = true;
	Display::data[12][6] = true;
	//d
	Display::data[13][1] = true;
	Display::data[13][2] = true;
	Display::data[13][3] = true;
	Display::data[13][4] = true;
	Display::data[13][6] = true;
	//E
	Display::data[14][0] = true;
	Display::data[14][3] = true;
	Display::data[14][4] = true;
	Display::data[14][5] = true;
	Display::data[14][6] = true;
	//F
	Display::data[15][0] = true;
	Display::data[15][4] = true;
	Display::data[15][5] = true;
	Display::data[15][6] = true;
}
void Display::show(unsigned char number)
{
	if (number > 15)
		return;
	for (unsigned char i = 0; i < 7 ; i++)
	{
		if (Display::data[number][i])
			Display::segment[i].on();
		else
			Display::segment[i].off();
	}
}

