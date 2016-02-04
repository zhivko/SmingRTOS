#include <user_config.h>
#include <SmingCore.h>

#define LED_PIN 2 // GPIO2

Timer procTimer;
bool state = true;
Serial serial1 = Serial(UART_ID_1);

void blink()
{
	digitalWrite(LED_PIN, state);
	state = !state;
}

void init()
{
	pinMode(LED_PIN, OUTPUT);
	procTimer.initializeMs(1000, blink).start();
}
