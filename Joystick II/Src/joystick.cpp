
#include "joystick.hpp"

Joystick::Joystick()
{
	return;
}
Joystick::Joystick(NRF* nrf,ADC_HandleTypeDef* ADC,TIM_HandleTypeDef* htim,unsigned long int x,unsigned long int y,unsigned long int w,unsigned long int p)
{
	init(nrf,ADC,htim,x,y,w,p);
}
void Joystick::init(NRF* nrf,ADC_HandleTypeDef* ADC,TIM_HandleTypeDef* htim,unsigned long int x,unsigned long int y,unsigned long int w,unsigned long int p)
{
	Joystick::hnrf = nrf;
	Joystick::htim = htim;
	Joystick::NRFInit();
	Joystick::hadc = ADC;
	Joystick::XChannel = x;
	Joystick::YChannel = y;
	Joystick::WChannel = w;
	Joystick::PowerChannel = p;
	Joystick::getOffsets();	
	Joystick::speedConstant = 0.25;
	Joystick::shootUpdated = 0;
	HAL_TIM_Base_Start_IT(htim);
}
void Joystick::NRFInit(void)
{
	char Address[5] = { 0x11,0x22,0x33,0x44,0x00};
	hnrf -> clear_Interrupts();
	hnrf -> flush_TX();
	hnrf -> flush_RX();
	hnrf -> init_milad(_TX_MODE,JOYSTIC_CHANNEL,_2Mbps,Address,_Address_Width,_Buffer_Size,RF_PWR_MAX);
	hnrf -> writeReg(W_REGISTER | DYNPD,0x01);
	hnrf -> writeReg(W_REGISTER | FEATURE,0x06);
	HAL_Delay(2);	
}
void Joystick::changeChannel(unsigned long int channel)
{
	ADC_ChannelConfTypeDef config;
	config.Channel = channel;
  config.Rank = 1;
  config.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(hadc, &config);
}
void Joystick::getOffsets(void)
{
	changeChannel(XChannel);
	signed long int temp = 0;
	for (unsigned int i = 0; i < DATANUMBER;i++)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc,100);
		temp += (HAL_ADC_GetValue(hadc) - OFFSET) / DATANUMBER;
		HAL_ADC_Stop(hadc);
	}
	if (temp > DOFFSET || temp < - DOFFSET)
		XOffset = OFFSET;
	else
		XOffset = temp + OFFSET;
	changeChannel(YChannel);
	temp = 0;
	for (unsigned int i = 0; i < DATANUMBER;i++)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc,100);
		temp += (HAL_ADC_GetValue(hadc) - OFFSET) / DATANUMBER;
		HAL_ADC_Stop(hadc);
	}
	if (temp > DOFFSET || temp < - DOFFSET)
		YOffset = OFFSET;
	else
		YOffset = temp + OFFSET;
	changeChannel(WChannel);
	temp = 0;
	for (unsigned int i = 0; i < DATANUMBER;i++)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc,100);
		temp += (HAL_ADC_GetValue(hadc) - OFFSET) / DATANUMBER;
		HAL_ADC_Stop(hadc);
	}
	if (temp > DOFFSET || temp < - DOFFSET)
		WOffset = OFFSET;
	else
		WOffset = temp + OFFSET;
	changeChannel(PowerChannel);
	temp = 0;
	for (unsigned int i = 0; i < DATANUMBER;i++)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc,100);
		temp += (HAL_ADC_GetValue(hadc) - OFFSET) / DATANUMBER;
		HAL_ADC_Stop(hadc);
	}
	if (temp > DOFFSET || temp < - DOFFSET)
		PowerOffset = OFFSET;
	else
		PowerOffset = temp + OFFSET;
	
}
signed int Joystick::getX(void)
{
	changeChannel(XChannel);
	signed long int temp = 0;
	for (unsigned int i = 0; i < DATANUMBER;i++)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc,100);
		temp += (HAL_ADC_GetValue(hadc) - XOffset) / DATANUMBER;
		HAL_ADC_Stop(hadc);
	}
	if (temp < DOFFSET && temp > -DOFFSET)
		temp = 0;
	if (temp < 5000 && temp > -5000)
		return temp;	
	else 
		return 0;
}
signed int Joystick::getY(void)
{
	changeChannel(YChannel);
	signed long int temp = 0;
	for (unsigned int i = 0; i < DATANUMBER;i++)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc,100);
		temp += (HAL_ADC_GetValue(hadc) - YOffset ) / DATANUMBER;
		HAL_ADC_Stop(hadc);
	}
	if (temp < DOFFSET && temp > -DOFFSET)
		temp = 0;
	if (temp < 5000 && temp > -5000)
		return temp;	
	else 
		return 0;
}
signed int Joystick::getW(void)
{
	changeChannel(WChannel);
	signed long int temp = 0;
	for (unsigned int i = 0; i < DATANUMBER;i++)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc,100);
		temp += (HAL_ADC_GetValue(hadc) - WOffset) / DATANUMBER;
		HAL_ADC_Stop(hadc);
	}
	if (temp < DOFFSET && temp > -DOFFSET)
		temp = 0;
	if (temp < 5000 && temp > -5000)
		return temp;	
	else 
		return 0;
}
signed int Joystick::getPower(void)
{
	changeChannel(PowerChannel);
	signed long int temp = 0;
	for (unsigned int i = 0; i < DATANUMBER;i++)
	{
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc,100);
		temp += (HAL_ADC_GetValue(hadc) - PowerOffset) / DATANUMBER;
		HAL_ADC_Stop(hadc);
	}
	if (temp < 5000 && temp > -5000)
		return temp;	
	else 
		return 0;
}
void Joystick::changeRobot(unsigned char robotID)
{
	char Address[5] = { 0x11,0x22,0x33,0x44,0x00};
	//Address[4] = (robotID << 4) | robotID;
	hnrf -> set_RX_Pipe(0,Address,5,32);
	hnrf -> set_TX_Address(Address,5);
	/*if (robotID < 6 )
		hnrf -> set_CH(_CH_1);
	else
		hnrf -> set_CH(_CH_0);*/
	buffer[0] = robotID;
}
void Joystick::setX(signed int speed)
{
	if (speed < DOFFSET && speed > -DOFFSET)
		speed = 0;
	buffer[1] = (speed & 0xFF00) >> 8;
	buffer[2] = (speed & 0x00FF);
}

void Joystick::setY(signed int speed)
{
	if (speed < DOFFSET && speed > -DOFFSET)
		speed = 0;
	buffer[3] = (speed & 0xFF00) >> 8;
	buffer[4] = (speed & 0x00FF);	
}

void Joystick::setW(signed int speed)
{
	if (speed < DOFFSET && speed > -DOFFSET)
		speed = 0;
	buffer[5] = (speed & 0xFF00) >> 8;
	buffer[6] = (speed & 0x00FF);	
}
void Joystick::setPower(signed int speed)
{

}

void Joystick::update()
{
/*	unsigned char flag = 0;
	for (unsigned char j = 1;j < 7;j++)
	{
		if (buffer[j] > DOFFSET || buffer[j] < - DOFFSET)
			flag = 1;
	}
	if (flag == 0)
	{
		buffer[1] = 1;
		buffer[2] = 2;
		buffer[3] = 3;
		buffer[4] = 4;
		
	}*/
	char buf[32];
	for (unsigned char j = 0;j < 32;j++)
		buf[j] = (char)buffer[j];
	hnrf -> clear_Interrupts();
	hnrf -> flush_TX();
	hnrf -> write_TX_Buf(buf,32);
	HAL_GPIO_WritePin(hnrf -> CEPort , hnrf -> CEPin , GPIO_PIN_SET);
	for (unsigned long int j = 0; j < 30000 ; j++);
	HAL_GPIO_WritePin(hnrf -> CEPort , hnrf -> CEPin , GPIO_PIN_RESET);
	if (Data_Ready())
	{
		hnrf -> read_RX_Buf(buf,32);
		hnrf -> clear_All();
		for (unsigned char j = 0;j < 32;j++)
			response[j] = buf[j];
	}
	//buffer[15] = 0;
}
bool Joystick::Data_Ready()
{
	unsigned char status = hnrf -> get_Status();
	if (((status & 0x40) == 0x40) && (status != 0xFF))
		return true;
	return false;
}
void Joystick::Key_Changed()
{
	Read_Keys();
	Do_Keys();
}
void Joystick::Read_Keys(void)
{
	keys[0] = (unsigned char) HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10);
	keys[1] = (unsigned char) HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);
	keys[2] = (unsigned char) HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
	keys[3] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
	keys[4] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
	keys[5] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
	//keys[6] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
	keys[6] = true;
	keys[7] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
	keys[8] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);
	keys[9] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
	keys[10] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
	keys[11] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
	keys[12] = (unsigned char) HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
}
void Joystick::Do_Keys()
{
	if (keys[X_KEY] == 0)
	{
	}
	if (keys[SQUARE_KEY] == 0)
	{
		buffer[15] = 100;
		shootUpdated = 0;
	}
	if (keys[CIRCLE_KEY] == 0)
	{
	}
	if (keys[TRIANGLE_KEY] == 0)
	{
	}
	if (keys[LEFT_KEY] == 0)
	{
	}
	if (keys[RIGHT_KEY] == 0)
	{
	}
	if (keys[UP_KEY] == 0)
	{
	}
	if (keys[DOWN_KEY] == 0)
	{
	}
	if (keys[L1_KEY] == 0)
	{
		speedConstant = 0.25;
	}
	if (keys[L2_KEY] == 0)
	{
	}
	if (keys[R1_KEY] == 0)
	{
		speedConstant = 2;
	}
	if (keys[R2_KEY] == 0)
	{
	}	
}
void Joystick::Time_Passed(TIM_HandleTypeDef* htim)
{
	if (htim != Joystick::htim)
		return;
	if (shootUpdated < SHOOTTIMER + 10)
		shootUpdated++;
	if (shootUpdated == SHOOTTIMER)
		buffer[15] = 0;
}
