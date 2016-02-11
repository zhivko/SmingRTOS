#include <user_config.h>
#include <SmingCore.h>

#define LED_PIN 2 // GPIO2

Timer procTimer;
bool state = true;
uint32_t i=0;

HardwareSerial serial1 = HardwareSerial(UART_ID_0);

void blink()
{
	i++;
	//digitalWrite(LED_PIN, state);
	state = !state;
	serial1.printf("This is my %d test 123.\n", i);
}

void init()
{
	//pinMode(LED_PIN, OUTPUT);
	procTimer.initializeMs(1000, blink).start();
}
