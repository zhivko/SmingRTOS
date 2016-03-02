/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 ****/

#include "Interrupts.h"
#include "Digital.h"
#include "../include/sming_global.h"
#include "espressif/esp8266/ets_sys.h"

#include "portmacro.h"

InterruptCallback _gpioInterruptsList[16] = {0};
Delegate<void()> _delegateFunctionList[16];
bool _gpioInterruptsInitialied = false;

void attachInterrupt(uint8_t pin, InterruptCallback callback, uint8_t mode)
{
	GPIO_INT_TYPE type = ConvertArduinoInterruptMode(mode);
	attachInterrupt(pin, callback, type);
}

void attachInterrupt(uint8_t pin, Delegate<void()> delegateFunction, uint8_t mode)
{
	GPIO_INT_TYPE type = ConvertArduinoInterruptMode(mode);
	attachInterrupt(pin, delegateFunction, type);
}

void attachInterrupt(uint8_t pin, InterruptCallback callback, GPIO_INT_TYPE mode)
{
	if (pin >= 16) return; // WTF o_O
	_gpioInterruptsList[pin] = callback;
	_delegateFunctionList[pin] = nullptr;
	attachInterruptHandler(pin, mode);
}

void attachInterrupt(uint8_t pin, Delegate<void()> delegateFunction, GPIO_INT_TYPE mode)
{
	if (pin >= 16) return; // WTF o_O
	_gpioInterruptsList[pin] = NULL;
	_delegateFunctionList[pin] = delegateFunction;
	attachInterruptHandler(pin, mode);
}

void attachInterruptHandler(uint8_t pin, GPIO_INT_TYPE mode)
{
	portENTER_CRITICAL();

	if (!_gpioInterruptsInitialied)
	{
		gpio_intr_handler_register((void*)interruptHandler, NULL); // Register interrupt handler
		_xt_isr_unmask(1<<ETS_GPIO_INUM);
		_gpioInterruptsInitialied = true;
	}

	pinMode(pin, INPUT);

	gpio_pin_intr_state_set(GPIO_ID_PIN(pin), mode); // Enable GPIO pin interrupt

	portEXIT_CRITICAL();
}

void detachInterrupt(uint8_t pin)
{
	_gpioInterruptsList[pin] = NULL;
	_delegateFunctionList[pin] = nullptr;
	attachInterruptHandler(pin, GPIO_PIN_INTR_DISABLE);
}

void interruptMode(uint8_t pin, uint8_t mode)
{
	GPIO_INT_TYPE type = ConvertArduinoInterruptMode(mode);
	interruptMode(pin, type);
}

void interruptMode(uint8_t pin, GPIO_INT_TYPE type)
{
	portENTER_CRITICAL();

	pinMode(pin, INPUT);
	gpio_pin_intr_state_set(GPIO_ID_PIN(pin), type);

	portEXIT_CRITICAL();
}

GPIO_INT_TYPE ConvertArduinoInterruptMode(uint8_t mode)
{
	switch (mode)
	{
	case LOW: // to trigger the interrupt whenever the pin is low,
		return GPIO_PIN_INTR_LOLEVEL;
	case CHANGE: // to trigger the interrupt whenever the pin changes value
		return (GPIO_INT_TYPE)3; // GPIO_PIN_INTR_ANYEDGE
	case RISING: // to trigger when the pin goes from low to high,
		return GPIO_PIN_INTR_POSEDGE;
	case FALLING: // for when the pin goes from high to low.
		return GPIO_PIN_INTR_NEGEDGE;
	case HIGH: // to trigger the interrupt whenever the pin is high.
		return GPIO_PIN_INTR_HILEVEL;
	default:
		return GPIO_PIN_INTR_DISABLE;
	}
}

void noInterrupts()
{
	portENTER_CRITICAL();
}
void interrupts()
{
	portEXIT_CRITICAL();
}

static void IRAM_ATTR interruptHandler(uint32 intr_mask, void *arg)
{
	boolean processed;
	uint32 gpio_status;

	do
	{
		gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
		processed = false;
		for (uint8 i = 0; i < ESP_MAX_INTERRUPTS; i++, gpio_status<<1)
		{
			if ((gpio_status & BIT(i)) && (_gpioInterruptsList[i]||_delegateFunctionList[i]))
			{
				//clear interrupt status
				GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(i));

				if (_gpioInterruptsList[i])
					_gpioInterruptsList[i]();
				else if (_delegateFunctionList[i])
					_delegateFunctionList[i]();

				processed = true;
			}

		}
	} while (processed);
}
