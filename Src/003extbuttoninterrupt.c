#include <stm32f407g.h>
#include <stm32f407g_gpio_driver.h>
#include <string.h>


void delay(void)
{
	for(uint32_t i=0; i < 500000/2; i++);
}


void EXT_INTERRUPT_BUTTON();

int main()
{


	GPIO_Handle_t LED_Toggle;
	GPIO_Handle_t Button;
	memset(&LED_Toggle, 0, sizeof(LED_Toggle));
	memset(&Button, 0, sizeof(LED_Toggle));


	Button.pGPIOx = GPIOD;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Button.GPIO_PinConfig.GPIO_PinMode = 4;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Button.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_IN;


	LED_Toggle.pGPIOx = GPIOD;
	LED_Toggle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	LED_Toggle.GPIO_PinConfig.GPIO_PinMode = 1;
	LED_Toggle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	LED_Toggle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
	LED_Toggle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LED_Toggle.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_OUT;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&LED_Toggle);

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&Button);


	GPIO_IRQPriorityConfig(IRQ_EXT9_5, 15);
	GPIO_IRQConfig(IRQ_EXT9_5, ENABLE);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

}


void EXT_INTERRUPT_BUTTON()
{
	{
		GPIO_IRQHandler(GPIO_PIN_NO_5);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	}
}


