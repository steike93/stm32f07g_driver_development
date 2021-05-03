
#include <stm32f407g.h>
#include <stm32f407g_gpio_driver.h>


void delay(void)
{
	for(uint32_t i=0; i < 500000/2; i++);
}



int main(void)
{

	GPIO_Handle_t LED_Toggle;
	GPIO_Handle_t Button;

	Button.pGPIOx = GPIOB;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	Button.GPIO_PinConfig.GPIO_PinMode = 0;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Button.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_IN;


	LED_Toggle.pGPIOx = GPIOD;
	LED_Toggle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	LED_Toggle.GPIO_PinConfig.GPIO_PinMode = 1;
	LED_Toggle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	LED_Toggle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	LED_Toggle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LED_Toggle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_OUT;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&LED_Toggle);

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&Button);


	while(1)
	{

		if((GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == 0))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

	}



	for(;;);
}

