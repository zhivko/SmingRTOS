//#include "SerialReadingDelegateDemo.h"
#include <user_config.h>
#include <SmingCore.h>

#include "zmtp_classes.h"

/*
#include <machinetalk/protobuf/message.pb.h>
#include <machinetalk/protobuf/types.pb.h>
#include <google/protobuf/text_format.h>
*/
//#include "../sming/system/uart.h"

//#define DISABLE_SPIFFS true

HardwareTimer hardwareTimer;

// If you want, you can define WiFi settings globally in Eclipse Environment Variables
#ifndef WIFI_SSID
#define WIFI_SSID "Enter_wifi_ssid" // Put you SSID and Password here
#define WIFI_PWD "Enter_wifi_pwd"
#endif

// ltc2400 settings
#define PIN_DO 5	/* Master In Slave Out */
#define PIN_DI 4	/* Master Out Slave In */
#define PIN_CK 15	/* Serial Clock */
#define PIN_SS 12	/* Slave Select */

int currWifiIndex = 0;
SPISoft *Ltc2400Spi = NULL;

Vector<String> wifi_sid;
Vector<String> wifi_pass;

Timer reportTimer;
//SoftwareSerial softSerial;
rBootHttpUpdate* airUpdater;
float_t floatAnalog;
float_t floatEncoder;
String analogResult;
long longAnalog;
int cnt = 0;                  // counter

HttpServer server;
int totalActiveSockets = 0;
long nextPos[4];
long curPos[4];
uint8_t step[4];
uint8_t dir[4];
uint32_t deltat = 2000;
String lastPositionMessage = "";
bool steppersOn = false;

rBootHttpUpdate* otaUpdater;

String ROM_0_URL = "http://192.168.1.19/firmwareRtos/rom0.bin";
String SPIFFS_URL = "http://192.168.1.19/firmwareRtos/spiff_rom.bin";

uint8_t x = 0;
uint8_t y = 1;
uint8_t z = 2;
uint8_t e = 3;

//SerialReadingDelegateDemo delegateDemoClass;

int8_t encoder0PinA = 5;
int8_t encoder0PinB = 4;
long encoder0Pos=0;

char* deblank(char* input)                                                  /* deblank accepts a char[] argument and returns a char[] */
{
    char *output=input;
    for (int i = 0, j = 0; i<strlen(input); i++,j++)                        /* Evaluate each character in the input */
    {
        if (input[i]!=' ')                                                  /* If the character is not a space */
            output[j]=input[i];                                             /* Copy that character to the output char[] */
        else
            j--;                                                            /* If it is a space then do not increment the output index (j), the next non-space will be entered at the current index */
    }
    return output;                                                          /* Return output char[]. Should have no spaces*/
}


void incrementNextWifiIndex() {
	currWifiIndex++;
	if (currWifiIndex == (wifi_sid.size()))
		currWifiIndex = 0;
}

void readFromLTC2400() {
	float volt;
	float v_ref = 4.096; // Reference Voltage, 5.0 Volt for LT1021 or 3.0 for LP2950-3, 4.096Vs for REF3040
	long int ltw = 0;         // ADC Data ling int
	BYTE sig;                 // sign bit flag
	BYTE b0;                  //
	char buf1[10];
	char buf[60];

	digitalWrite(PIN_SS, 0);
	delayMicroseconds(1);
	if (digitalRead(PIN_DO) == 0) {   // ADC Converter ready ?

		ltw = 0;
		sig = 0;

		b0 = Ltc2400Spi->transfer(1);      // read 4 bytes adc raw data with SPI
		if ((b0 & 0x20) == 0)
			sig = 1;  // is input negative ?
		b0 &= 0x1F;                   // discard bit 25..31
		ltw |= b0;
		ltw <<= 8;
		b0 = Ltc2400Spi->transfer(1);
		ltw |= b0;
		ltw <<= 8;
		b0 = Ltc2400Spi->transfer(1);
		ltw |= b0;
		ltw <<= 8;
		b0 = Ltc2400Spi->transfer(1);
		ltw |= b0;

		delayMicroseconds(1);

		digitalWrite(PIN_SS, HIGH);      // LTC2400 CS HIGH
		delay(200);

		if (sig)
			ltw |= 0xf0000000;    // if input negative insert sign bit
		ltw = ltw / 16;   // scale result down , last 4 bits have no information
		volt = ltw * v_ref / 16777216; // max scale

		//Serial.printf("%d",cnt++);
		//Serial.printf(";  ");
		dtostrf(volt, 6, 6, buf1);
		//Serial.printf("%s",buf1);           // print voltage as floating number
		//Serial.println("  ");

		sprintf(buf, "Analogue: %s", buf1);
		String message = String(buf);

		if (!message.equals(lastPositionMessage)) {
			WebSocketsList &clients = server.getActiveWebSockets();
			for (int i = 0; i < clients.count(); i++) {
				clients[i].sendString(message);
			}
			lastPositionMessage = message;
		}

	}
	digitalWrite(PIN_SS, HIGH); // LTC2400 CS hi
	delay(5);
	reportTimer.startOnce();
}

void reportAnalogue() {
	char buf[60];
	char buf1[10];
	char data[4];

	/*
	 1 ... BROWN        ... 12V ... 24V +
	 2 ... WHITE        ... RXD
	 3 ... BLUE         ... GND
	 4...  BLACK        ... not used
	 5...  YELOW/GREEN  ... TXD
	 */

	floatAnalog = atof(analogResult.c_str()) / 10.0;
	dtostrf(floatAnalog, 7, 4, buf1);
	sprintf(buf, "Analogue: %s", deblank(buf1));
	String message = String(buf);

	if (!message.equals(lastPositionMessage)) {
		WebSocketsList &clients = server.getActiveWebSockets();
		for (int i = 0; i < clients.count(); i++) {
			clients[i].sendString(message);
		}
		lastPositionMessage = message;
		//Serial.printf("Analogue: %f", analogResult.c_str());
	}
}


void reportEncoderPosition() {
	char buf[60];
	char buf1[12];

	floatEncoder = encoder0Pos * (2.4/160.0);
	dtostrf(floatEncoder, 4, 2, buf1);
	sprintf(buf, "Encoder: %s", deblank(buf1));
	String message1 = String(buf);

	if (!message1.equals(lastPositionMessage)) {
		WebSocketsList &clients = server.getActiveWebSockets();
		for (int i = 0; i < clients.count(); i++) {
			clients[i].sendString(message1);
		}
		lastPositionMessage = message1;
	}


    zmtp_msg_test (false);
    zmtp_channel_test (false);

/*


    pb::Container container, got;


    pb::Pin *pin;
    pb::Value *value;

    // type-tag the container:
    container.set_type(pb::ContainerType::MT_HALUPDATE);
    container.set_serial(56789);
    container.set_rsvp(pb::ReplyType::NONE);


    // add repeated submessage(s)
    pin = container.add_pin();
    pin->set_type(pb::ValueType::HAL_S32);
    pin->set_name("foo.1.bar");
    pin->set_hals32(4711);

    value = container.add_value();
    value->set_type(pb::ValueType::DOUBLE);
    value->set_v_double(3.14159);

    //std::string json = pb2json(container);
*/

}


void sendToClients(String message) {
	WebSocketsList &clients = server.getActiveWebSockets();
	for (int i = 0; i < clients.count(); i++) {
		clients[i].sendString(message);
	}
}

void enableMotors() {
	steppersOn = true;
	digitalWrite(2, false);
}

void disableMotors() {
	steppersOn = false;
	digitalWrite(2, true);
}

void reportStatus() {
	char buf[30];
	sprintf(buf, "X%d Y%d Z%d E%d M%d", curPos[0], curPos[1], curPos[2],
			curPos[3], steppersOn);
	String message = String(buf);
	if (!message.equals(lastPositionMessage)) {
		sendToClients(message);
		lastPositionMessage = message;
	}
}

void IRAM_ATTR AnalogReadTimerInt() {
	/*
	 int maxTimeout = 100;
	 int sleeping = 0;

	 int averageLoopMax = 30;
	 int j = 0;
	 long analogSum = 0;
	 int added = 0;
	 float tempAnalog;

	 while (j < averageLoopMax) {
	 while (!hx711.is_ready() && sleeping < 100) {
	 delayMicroseconds(5);
	 sleeping = sleeping + 5;
	 }
	 if (hx711.is_ready()) {
	 hardwareTimer.startOnce();
	 long result = hx711.read();
	 analogSum = analogSum + result;
	 added++;
	 } else {
	 //Still not ready
	 //floatAnalog = -1.0;
	 }
	 j++;
	 }
	 floatAnalog = analogSum / added;
	 hardwareTimer.initializeUs(deltat, AnalogReadTimerInt);
	 */

}

void IRAM_ATTR StepperTimerInt() {
	hardwareTimer.initializeUs(deltat, StepperTimerInt);
	hardwareTimer.startOnce();

	if (steppersOn) {
		uint32_t pin_mask_steppers = 0;
		//set direction pins
		for (int i = 0; i < 4; i++) {
			if (curPos[i] != nextPos[i]) {
				int8_t sign = -1;
				if (nextPos[i] > curPos[i])
					sign = 1;
				if (sign > 0)
					digitalWrite(dir[i], false);
				else
					digitalWrite(dir[i], true);
				curPos[i] = curPos[i] + sign;
				pin_mask_steppers = pin_mask_steppers | (1 << step[i]);
			}
		}
		delayMicroseconds(3);
		GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, pin_mask_steppers);
		delayMicroseconds(5);
		GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, pin_mask_steppers); // Set pin a and b low
		delayMicroseconds(5);
	}
}

void OtaUpdate_CallBack(bool result) {

	Serial.println("In callback...");
	if (result == true) {
		// success
		uint8 slot;
		slot = rboot_get_current_rom();
		if (slot == 0)
			slot = 1;
		else
			slot = 0;
		// set to boot new rom and then reboot
		Serial.printf("Firmware updated, rebooting to rom %d...\r\n", slot);
		sendToClients("Firmware updated, rebooting...");
		rboot_set_current_rom(slot);
		System.restart();
	} else {
		// fail
		Serial.println("Firmware update failed!");
	}
}

void ShowInfo() {
	Serial.printf("\r\nSDK: v%s\r\n", system_get_sdk_version());
	Serial.printf("Free Heap: %d\r\n", system_get_free_heap_size());
	Serial.printf("CPU Frequency: %d MHz\r\n", system_get_cpu_freq());
	Serial.printf("System Chip ID: %x\r\n", system_get_chip_id());
	Serial.printf("SPI Flash ID: %x\r\n", spi_flash_get_id());
	//Serial.printf("SPI Flash Size: %d\r\n", (1 << ((spi_flash_get_id() >> 16) & 0xff)));

	Vector<String> files = fileList();
	if (files.count() > 0) {
		Serial.println("\n\rSpiff files:");
		Serial.println("----------------------------");
		{
			for (int i = 0; i < files.count(); i++) {
				Serial.println(files[i]);
			}
		}
		Serial.println("----------------------------");
	} else {
		Serial.println("Empty spiffs!");
	}

}

void Switch() {
	uint8 before, after;
	before = rboot_get_current_rom();
	if (before == 0)
		after = 1;
	else
		after = 0;
	Serial.printf("Swapping from rom %d to rom %d.\r\n", before, after);
	rboot_set_current_rom(after);
	Serial.println("Restarting...\r\n");
	System.restart();
}

void OtaUpdate() {

	hardwareTimer.stop();

	uint8 slot;
	rboot_config bootconf;

	Serial.println("Updating...");
	sendToClients("Firmware ota update started...");

	// need a clean object, otherwise if run before and failed will not run again
	if (otaUpdater)
		delete otaUpdater;
	otaUpdater = new rBootHttpUpdate();

	// select rom slot to flash
	bootconf = rboot_get_config();
	slot = bootconf.current_rom;
	if (slot == 0)
		slot = 1;
	else
		slot = 0;

#ifndef RBOOT_TWO_ROMS
	// flash rom to position indicated in the rBoot config rom table
	otaUpdater->addItem(bootconf.roms[slot], ROM_0_URL);
#else
	// flash appropriate rom
	if (slot == 0) {
		otaUpdater->addItem(bootconf.roms[slot], ROM_0_URL);
	} else {
		otaUpdater->addItem(bootconf.roms[slot], ROM_1_URL);
	}
#endif

#ifndef DISABLE_SPIFFS
	// use user supplied values (defaults for 4mb flash in makefile)
	if (slot == 0) {
		otaUpdater->addItem(RBOOT_SPIFFS_0, SPIFFS_URL);
	} else {
		otaUpdater->addItem(RBOOT_SPIFFS_1, SPIFFS_URL);
	}
#endif

	// request switch and reboot on success
	//otaUpdater->switchToRom(slot);
	// and/or set a callback (called on failure or success without switching requested)
	otaUpdater->setCallback(OtaUpdate_CallBack);

	// start update
	otaUpdater->start();
}

void parseGcode(String commandLine) {
	if (commandLine.equals("ota")) {
//server.enableWebSockets(false);
		OtaUpdate();
		return;
	} else if (commandLine.equals("restart")) {
		System.restart();
		return;
	} else if (commandLine.equals("pos")) {
		reportStatus();
		return;
	} else if (commandLine.equals("enable")) {
		enableMotors();
		return;
	} else if (commandLine.equals("disable")) {
		disableMotors();
		return;
	} else if (commandLine.equals("stop")) {
		for (int i = 0; i < 4; i++) {
			nextPos[i] = curPos[i];
		}
	} else if (commandLine.startsWith("reassign")) {
//sendToClients(message)
//reassign x=3 y=0 z=2 e=1
		Vector<String> commandToken;
		int numToken = splitString(commandLine, ' ', commandToken);
		for (int i = 1; i < numToken; i++) {
			Vector<String> axisIndex;
			String axisIndexStr = commandToken[i].c_str();
			splitString(axisIndexStr, '=', axisIndex);
			String axis = axisIndex[0].c_str();
			if (axis.equals("x"))
				x = atoi(axisIndex[1].c_str());
			else if (axis.equals("y"))
				y = atoi(axisIndex[1].c_str());
			else if (axis.equals("z"))
				z = atoi(axisIndex[1].c_str());
			else if (axis.equals("e"))
				e = atoi(axisIndex[1].c_str());
		}
		char buf[150];
		sprintf(buf, "Reassign: x=%d y=%d z=%d e=%d\r\n", x, y, z, e);
		String msgBack = String(buf);
		sendToClients(msgBack);
		return;
	}

	if (steppersOn) {
		Vector<String> commandToken;
		int numToken = splitString(commandLine, ' ', commandToken);
		for (int i = 0; i < numToken; i++) {
			Serial.printf("Command: %s\r\n", commandToken[i].c_str());
			String motor = commandToken[i].substring(0, 1);
			String sign = commandToken[i].substring(1, 2);
			String posStr = "";
			if (sign == "+" || sign == "-") {
				posStr = commandToken[i].substring(2, commandToken[i].length());
			} else {
				sign = "";
				posStr = commandToken[i].substring(1, commandToken[i].length());
			}
			int8_t index = -1;
			if (motor == "X")
				index = x;
			else if (motor == "Y")
				index = y;
			else if (motor == "Z")
				index = z;
			else if (motor == "E")
				index = e;
			else if (motor == "T") {
				deltat = atoi(posStr.c_str());
			}
			if (index > -1) {
				if (sign == "+")
					nextPos[index] = nextPos[index] + atol(posStr.c_str());
				else if (sign == "-")
					nextPos[index] = nextPos[index] - atol(posStr.c_str());
				else
					nextPos[index] = atol(posStr.c_str());
				Serial.printf("Set nextpos[%d] to %d\r\n", index,
						nextPos[index]);
			}
		}
	}
}


void serialCallBack(Stream& stream, char arrivedChar,
		unsigned short availableCharsCount) {
	int ia = (int) arrivedChar;
	if (ia == 13) {
		char str[availableCharsCount];
		for (int i = 0; i < availableCharsCount; i++) {
			str[i] = stream.read();
			if (str[i] == '\r' || str[i] == '\n') {
				str[i] = '\0';
			}
		}

		if (!strcmp(str, "connect")) {
// connect to wifi
			WifiStation.config(wifi_sid.get(currWifiIndex),
					wifi_pass.get(currWifiIndex));
			WifiStation.enable(true);
		} else if (!strcmp(str, "ip")) {
			Serial.printf("ip: %s mac: %s\r\n",
					WifiStation.getIP().toString().c_str(),
					WifiStation.getMAC().c_str());
		} else if (!strcmp(str, "ota")) {
			OtaUpdate();
		} else if (!strcmp(str, "restart")) {
			System.restart();
		} else if (!strcmp(str, "ls")) {
			Vector<String> files = fileList();
			Serial.printf("filecount %d\r\n", files.count());
			for (unsigned int i = 0; i < files.count(); i++) {
				Serial.println(files[i]);
			}
		} else if (!strcmp(str, "info")) {
			ShowInfo();
		} else if (!strcmp(str, "switch")) {
			Switch();
		} else if (!strcmp(str, "cat")) {
			Vector<String> files = fileList();
			if (files.count() > 0) {
				Serial.printf("dumping file %s:\r\n", files[2].c_str());
				Serial.println(fileGetContent(files[2]));
			} else {
				Serial.println("Empty spiffs!");
			}
		} else if (!strcmp(str, "pos")) {
			reportStatus();
		} else if (!strcmp(str, "move")) {
			Serial.println();
			nextPos[0] += 10000;
			nextPos[1] += 10000;
			nextPos[2] += 10000;
			nextPos[3] += 10000;
//procTimer.initializeUs(deltat, blink1).start(true);
		} else if (!strcmp(str, "help")) {
			Serial.println();
			Serial.println("available commands:");
			Serial.println("  help - display this message");
			Serial.println("  ip - show current ip address");
			Serial.println("  connect - connect to wifi");
			Serial.println("  restart - restart the esp8266");
			Serial.println("  switch - switch to the other rom and reboot");
			Serial.println("  ota - perform ota update, switch rom and reboot");
			Serial.println("  info - show esp8266 info");
#ifndef DISABLE_SPIFFS
			Serial.println("  ls - list files in spiffs");
			Serial.println("  cat - show first file in spiffs");
#endif
			Serial.println();
		} else {
			Serial.printf("Trying to parse as gCode: %s\n", str);
			parseGcode(str);
		}
	} else if (ia == 48) {
		Serial.println();
		stream.read();
		nextPos[0] += 100;
		nextPos[1] += 100;
		nextPos[2] += 100;
		nextPos[3] += 100;
//procTimer.initializeUs(deltat, blink1).start(true);
	} else if (ia == 49) {
		Serial.println();
		stream.read();
		nextPos[0] -= 100;
		nextPos[1] -= 100;
		nextPos[2] -= 100;
		nextPos[3] -= 100;
//procTimer.initializeUs(deltat, blink1).start(true);
	}
}


void onIndex(HttpRequest &request, HttpResponse &response) {
	TemplateFileStream *tmpl = new TemplateFileStream("index.html");
	auto &vars = tmpl->variables();
//vars["counter"] = String(counter);
	response.sendTemplate(tmpl); // this template object will be deleted automatically
}

void onFile(HttpRequest &request, HttpResponse &response) {
	String file = request.getPath();
	if (file[0] == '/')
		file = file.substring(1);

	if (file[0] == '.')
		response.forbidden();
	else {
		response.setCache(86400, true); // It's important to use cache for better performance.
		response.sendFile(file);
	}
}

void wsConnected(WebSocket& socket) {
	totalActiveSockets++;
	lastPositionMessage = "";
	// Notify everybody about new connection

	WebSocketsList &clients = server.getActiveWebSockets();
	for (int i = 0; i < clients.count(); i++) {
		clients[i].sendString(
				"Connected to station: " + wifi_sid.get(currWifiIndex)
						+ ", SDK version: " + system_get_sdk_version());
	}

}

void wsMessageReceived(WebSocket& socket, const String& message) {
	Serial.printf("WebSocket message received: %s\r\n", message.c_str());

	char buf[150];
	sprintf(buf, "WebSocket message received: %s\r\n", message.c_str());
	String msgBack = String(buf);
	sendToClients(msgBack);

	parseGcode(message.c_str());
}

void wsBinaryReceived(WebSocket& socket, uint8_t* data, size_t size) {
	Serial.printf("Websocket binary data receieved, size: %d\r\n", size);
}

void wsDisconnected(WebSocket& socket) {
	totalActiveSockets--;
}

void initPins() {
	Serial.println("Init pins");

//---------------------
	step[0] = 5;  //2
	dir[0] = 4;   //0

	step[1] = 14;  //4
	dir[1] = 12;   //5
//---------------------
	step[2] = 03;
	dir[2] = 01;

	step[3] = 15;
	dir[3] = 13;
//---------------------
//system_soft_wdt_feed();

	for (int i = 0; i < 4; i++) {
		pinMode(step[i], OUTPUT);
		pinMode(dir[i], OUTPUT);
		digitalWrite(step[i], true);
		digitalWrite(dir[i], true);
		curPos[i] = 0;
		nextPos[i] = 0;
	}

	pinMode(2, OUTPUT);
	enableMotors();
}

void startWebServer() {

	Serial.println("Starting web server...Phase1");
	server.listen(80);
	server.addPath("/", onIndex);
	server.setDefaultHandler(onFile);

// Web Sockets configuration
	server.enableWebSockets(true);
	server.setWebSocketConnectionHandler(wsConnected);
	server.setWebSocketMessageHandler(wsMessageReceived);
	server.setWebSocketBinaryHandler(wsBinaryReceived);
	server.setWebSocketDisconnectionHandler(wsDisconnected);

	Serial.println("\r\n=== WEB SERVER STARTED ===");
	Serial.println(WifiStation.getIP().toString());
	Serial.println("==============================\r\n");
}

void doEncoderA() {

	// look for a low-to-high on channel A
	if (digitalRead(encoder0PinA) == HIGH) {
		// check channel B to see which way encoder is turning
		if (digitalRead(encoder0PinB) == LOW) {
			encoder0Pos = encoder0Pos + 1;         // CW
		} else {
			encoder0Pos = encoder0Pos - 1;         // CCW
		}
	} else // must be a high-to-low edge on channel A
	{
		// check channel B to see which way encoder is turning
		if (digitalRead(encoder0PinB) == HIGH) {
			encoder0Pos = encoder0Pos + 1;          // CW
		} else {
			encoder0Pos = encoder0Pos - 1;          // CCW
		}
	}
}

void doEncoderB() {

	// look for a low-to-high on channel B
	if (digitalRead(encoder0PinB) == HIGH) {
		// check channel A to see which way encoder is turning
		if (digitalRead(encoder0PinA) == HIGH) {
			encoder0Pos = encoder0Pos + 1;         // CW
		} else {
			encoder0Pos = encoder0Pos - 1;         // CCW
		}
	}
	// Look for a high-to-low on channel B
	else {
		// check channel B to see which way encoder is turning
		if (digitalRead(encoder0PinA) == LOW) {
			encoder0Pos = encoder0Pos + 1;          // CW
		} else {
			encoder0Pos = encoder0Pos - 1;          // CCW
		}
	}
}

// Will be called when WiFi station was connected to AP
void connectOk() {
	Serial.println("I'm CONNECTED to AP_SSID=" + wifi_sid.get(currWifiIndex));
	Serial.println("IP: ");
	String ipString = WifiStation.getIP().toString();
	Serial.println(ipString);

	startWebServer();

	Serial.println("Init ended.");
	Serial.println("Type 'help' and press enter for instructions.");
	Serial.println();
	//Serial.setCallback(serialCallBack);

	if (ipString.equals("192.168.1.115") || ipString.equals("192.168.1.110")) {
// distance sensor
		Serial.println("MODE: LEUZE Distance sensor");

		Serial.begin(57600);
		deltat = 100000;
		system_uart_swap();
		//delegateDemoClass.begin();
		reportTimer.initializeMs(100, reportAnalogue).start();
	} else if (ipString.equals("192.168.1.111")
			|| ipString.equals("192.168.1.112")) {
// 4 axis stepper driver
		Serial.println("MODE: 4 Axis Stepper driver");

		deltat = 2000;

		if (ipString.equals("192.168.1.112"))
			parseGcode("reassign x=3 y=0 e=1 z=2");
		else if (ipString.equals("192.168.1.111"))
			parseGcode("reassign x=0 y=1 e=3 z=2");

		reportTimer.initializeMs(300, reportStatus).start();
		hardwareTimer.initializeUs(deltat, StepperTimerInt);
		hardwareTimer.startOnce();
		initPins();
	} else if (ipString.equals("192.168.1.116")) {
		Serial.println("MODE: Encoder driver");
		pinMode(encoder0PinA, INPUT);
		digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
		pinMode(encoder0PinB, INPUT);
		digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor

		attachInterrupt(encoder0PinA, doEncoderA, GPIO_PIN_INTR_ANYEDGE);
		attachInterrupt(encoder0PinB, doEncoderB, GPIO_PIN_INTR_ANYEDGE);

		reportTimer.initializeMs(100, reportEncoderPosition).start();
	} else if (ipString.equals("192.168.1.117")) {
		Ltc2400Spi = new SPISoft(PIN_DO, PIN_DI, PIN_CK, PIN_SS);
		Ltc2400Spi->begin();
		reportTimer.initializeMs(300, readFromLTC2400).startOnce();
	}
}


void connectNotOk() {

	WifiStation.enable(false);
	incrementNextWifiIndex();
	WifiStation.config(wifi_sid.get(currWifiIndex),
			wifi_pass.get(currWifiIndex), false);
	WifiStation.enable(true);
	WifiStation.waitConnection(connectOk, 12, connectNotOk);

}


void init() {
//ets_wdt_disable();
	Serial.begin(115200);
	WifiStation.enable(false);
	System.setCpuFrequency(eCF_160MHz);
	//Serial.systemDebugOutput(true);
	Serial.println("************************");
	Serial.println("***** Init running *****");
	Serial.println("************************");

// mount spiffs

	int slot = rboot_get_current_rom();
#ifndef DISABLE_SPIFFS
	if (slot == 0) {
#ifdef RBOOT_SPIFFS_0
		debugf("trying to mount spiffs at %x, length %d", RBOOT_SPIFFS_0 , SPIFF_SIZE);
		spiffs_mount_manual(RBOOT_SPIFFS_0, SPIFF_SIZE);
#else
		debugf("trying to mount spiffs at %x, length %d", 0x100000, SPIFF_SIZE);
		spiffs_mount_manual(0x100000, SPIFF_SIZE);
#endif
	} else {
#ifdef RBOOT_SPIFFS_1
		debugf("trying to mount spiffs at %x, length %d", RBOOT_SPIFFS_1 , SPIFF_SIZE);
		spiffs_mount_manual(RBOOT_SPIFFS_1, SPIFF_SIZE);
#else
		debugf("trying to mount spiffs at %x, length %d", SPIFF_SIZE);
		spiffs_mount_manual(0x300000, SPIFF_SIZE);
#endif
	}
#else
	debugf("spiffs disabled");
#endif

	ShowInfo();
	//wifi_set_opmode(STATION_MODE);

	wifi_sid.add("AsusKZ");
	wifi_sid.add("Sintex");
	wifi_pass.add("Doitman1");
	wifi_pass.add("sintex92");
	WifiStation.config(wifi_sid.get(currWifiIndex),
	wifi_pass.get(currWifiIndex), true);
	WifiAccessPoint.enable(false);
	WifiStation.enable(true);
	WifiStation.waitConnection(connectOk, 18, connectNotOk);

}
