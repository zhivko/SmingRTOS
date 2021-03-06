#include "SerialReadingDelegateDemo.h"
#include <user_config.h>
#include <SmingCore.h>

//#include "contiki-zmtp/zmtp_msg.h"

//#include "zmtp_classes.h"

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
#define WIFI_SSID "linksys" // Put you SSID and Password here
#define WIFI_PWD "Doitman1"
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

rBootHttpUpdate* otaUpdater = 0;

//#define ROM_0_URL  "http://192.168.43.21/firmwareRtos/samples/StepperDM542/out/firmware/rom0.bin"
//#define SPIFFS_URL  "http://192.168.43.21/firmwareRtos/samples/StepperDM542/out/firmware/spiff_rom.bin"
#define ROM_0_URL  "http://192.168.1.19/firmwareRtos/samples/StepperDM542/out/firmware/rom0.bin"
#define SPIFFS_URL  "http://192.168.1.19/firmwareRtos/samples/StepperDM542/out/firmware/spiff_rom.bin"

// UDP server
uint16_t udpServerPort = 1234;
String udpServerIP = "192.168.1.19";

uint8_t x = 0;
uint8_t y = 1;
uint8_t z = 2;
uint8_t e = 3;

int8_t x_dir = 1;
int8_t y_dir = 1;
int8_t z_dir = 1;
int8_t e_dir = 1;

SerialReadingDelegateDemo delegateDemoClass;

int8_t encoder0PinA = 5;
int8_t encoder0PinB = 4;
long encoder0Pos = 0;

Timer procTimer;
bool state = true;
#define LED_PIN 2 // GPIO2

// forward declaration
void STADisconnect(String ssid, uint8_t ssid_len, uint8_t bssid[6], uint8_t reason);
void onReceive(UdpConnection& connection, char *data, int size, IPAddress remoteIP, uint16_t remotePort);

UdpConnection udp(onReceive);

void initPins() {
	Serial.println("Init pins");

//---------------------
	step[0] = 5; //2
	dir[0] = 4; //0

	step[1] = 14; //4
	dir[1] = 12; //5
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
}

void enableMotors() {
	initPins();
	steppersOn = true;
	digitalWrite(2, false);
}

char* deblank(char* input) /* deblank accepts a char[] argument and returns a char[] */
{
	char *output = input;
	for (int i = 0, j = 0; i < strlen(input); i++, j++) /* Evaluate each character in the input */
	{
		if (input[i] != ' ') /* If the character is not a space */
			output[j] = input[i]; /* Copy that character to the output char[] */
		else
			j--; /* If it is a space then do not increment the output index (j), the next non-space will be entered at the current index */
	}
	return output; /* Return output char[]. Should have no spaces*/
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

		/*
		 union u
		 {
		 float f;
		 char s[sizeof(float)];
		 };

		 union u foo;
		 foo.f = floatAnalog;
		 */
		//udp.sendStringTo(IPAddress("192.168.1.19"), (uint16_t)1234, foo.s);
		udp.sendStringTo(IPAddress(udpServerIP), udpServerPort, analogResult);

		//Serial.printf("Analogue: %f", analogResult.c_str());
	}
}

void reportEncoderPosition() {
	char buf[60];
	char buf1[12];

	floatEncoder = encoder0Pos * (2.4 / 160.0);
	dtostrf(floatEncoder, 4, 2, buf1);
	sprintf(buf, "Encoder: %s", deblank(buf1));
	String message1 = String(buf);

	if (!message1.equals(lastPositionMessage)) {
		WebSocketsList &clients = server.getActiveWebSockets();
		for (int i = 0; i < clients.count(); i++) {
			clients[i].sendString(message1);
		}
	}

	//printf(" * zmtp_msg: ");
	//zmtp_msg_t *msg = zmtp_msg_from_const_data(0, "hello", 6);
	//zmtp_msg_destroy(&msg);

	//zmtp_msg_test (false);
	//zmtp_channel_test (false);

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

void disableMotors() {
	steppersOn = false;
	digitalWrite(2, true);
}

void reportStatus() {
	char buf[30];
	sprintf(buf, "X%d Y%d Z%d E%d M%d", curPos[x], curPos[y], curPos[z], curPos[e], steppersOn);
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
	sendToClients("OtaUpdate_CallBack");

	Serial.println("In callback...");
	if (result == true) {
		sendToClients("Ota update SUCCESS!");
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
		sendToClients("Ota update FAIL!");
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
	uint8 slot;
	rboot_config bootconf;

	Serial.println("Updating...");

	hardwareTimer.stop();
	reportTimer.stop();

	sendToClients("Firmware ota update started...");

	// need a clean object, otherwise if run before and failed will not run again
	if (otaUpdater)
		delete otaUpdater;
	otaUpdater = new rBootHttpUpdate();

	sendToClients("Firmware ota update started...1");

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

	sendToClients("Firmware ota update started...2");
	// request switch and reboot on success
	otaUpdater->switchToRom(slot);
	// and/or set a callback (called on failure or success without switching requested)
	otaUpdater->setCallback(OtaUpdate_CallBack);

	// start update
	sendToClients("Firmware ota update started...3");
	otaUpdater->start();
}

void parseGcode(String commandLine) {
	if (commandLine.equals("ota")) {
		//server.enableWebSockets(false);
		OtaUpdate();
		return;
	} else if (commandLine.equals("switch")) {
		Switch();
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
	} else if (commandLine.startsWith("udpServerIP")) {
		Vector<String> commandToken;
		int numToken = splitString(commandLine, ' ', commandToken);
		if (numToken == 2)
			udpServerIP = commandToken[1];
		if (numToken == 3) {
			udpServerIP = commandToken[1];
			udpServerPort = atoi(commandToken[2].c_str());
		}
		char buf[150];
		sprintf(buf, "Udp server port %s, port: %d", udpServerIP.c_str(), udpServerPort);
		String msgBack = String(buf);
		sendToClients(msgBack);
		return;

	} else if (commandLine.startsWith("reassign")) {
//sendToClients(message)
//reassign x=+3 y=+0 z=-2 e=-1
// - means direction is reverse for relative moves
		Vector<String> commandToken;
		int numToken = splitString(commandLine, ' ', commandToken);
		for (int i = 1; i < numToken; i++) {
			Vector<String> axisIndex;
			String axisIndexStr = commandToken[i].c_str();
			splitString(axisIndexStr, '=', axisIndex);
			String axis = axisIndex[0].c_str();

			if (axis.equalsIgnoreCase("x")) {
				{
					x = atoi(axisIndex[1].substring(1,2).c_str());
					if (axisIndex[1].substring(0,1).equalsIgnoreCase("+")) {
						x_dir = 1;
					} else {
						x_dir = -1;
					}
				}
			} else if (axis.equalsIgnoreCase("y")) {
				{
					y = atoi(axisIndex[1].substring(1,2).c_str());
					if (axisIndex[1].substring(0,1).equalsIgnoreCase("+")) {
						y_dir = 1;
					} else {
						y_dir = -1;
					}
				}
			} else if (axis.equalsIgnoreCase("z")) {
				{
					z = atoi(axisIndex[1].substring(1,2).c_str());
					if (axisIndex[1].substring(0,1).equalsIgnoreCase("+")) {
						z_dir = 1;
					} else {
						z_dir = -1;
					}
				}
			} else if (axis.equalsIgnoreCase("e")) {
				{
					e = atoi(axisIndex[1].substring(1,2).c_str());
					if (axisIndex[1].substring(0,1).equalsIgnoreCase("+")) {
						e_dir = 1;
					} else {
						e_dir = -1;
					}
				}
			}
		}
		char buf[150];
		sprintf(buf, "Reassign: x=%d %d y=%d %d z=%d %d e=%d %d\r\n", x_dir, x, y_dir, y, z_dir, z, e_dir, e);
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
			if (motor.equalsIgnoreCase("X")) {
				index = x;
				if (x_dir == -1)
				{
					sendToClients("x_dir = -1");
					if (sign == "+")
						sign = "-";
					else
						sign = "+";
				}
				else
				{
					sendToClients("x_dir = +1");
				}
			} else if (motor.equalsIgnoreCase("Y")) {
				index = y;
				if (y_dir == -1)
					if (sign == "+")
						sign = "-";
					else
						sign = "+";
			} else if (motor.equalsIgnoreCase("Z")) {
				index = z;
				if (z_dir == -1)
					if (sign == "+")
						sign = "-";
					else
						sign = "+";
			} else if (motor.equalsIgnoreCase("E")) {
				index = e;
				if (e_dir == -1)
					if (sign == "+")
						sign = "-";
					else
						sign = "+";
			} else if (motor.equalsIgnoreCase("T")) {
				deltat = atoi(posStr.c_str());
			}
			if (index > -1) {
				if (sign.equalsIgnoreCase("+"))
					nextPos[index] = nextPos[index] + atol(posStr.c_str());
				else if (sign.equalsIgnoreCase("-"))
					nextPos[index] = nextPos[index] - atol(posStr.c_str());
				else
					nextPos[index] = atol(posStr.c_str());

				char buf[150];
				sprintf(buf, "Set nextpos[%d] to %d\r\n", index, nextPos[index]);
				Serial.printf(buf);
				String msgBack = String(buf);
				sendToClients(msgBack);
			}
		}
	}
}

void serialCallBack(Stream& stream, char arrivedChar, unsigned short availableCharsCount) {

	/*
	 Serial.print("Class Delegate Demo Time = ");
	 Serial.print(micros());
	 Serial.print(" char = 0x");
	 Serial.print(String(arrivedChar, HEX)); // char hex code
	 Serial.print(" available = ");
	 Serial.println(availableCharsCount);
	 */
	int ia = (int) arrivedChar;
	if (arrivedChar == '\n') // Lets show data!
			{
		char str[availableCharsCount];
		Serial.println("<New line received>");
		int i = 0;
		while (stream.available()) {
			char cur = stream.read();
			if (((int) cur != 13) && ((int) cur != 10)) {
				str[i] = cur;
				Serial.print(cur);
			} else {
				str[i] = '\0';
			}
			i++;
		}
		Serial.println();
		//}

		/*
		 int ia = (int) arrivedChar;
		 if (ia == 13) {
		 char str[availableCharsCount];
		 for (int i = 0; i < availableCharsCount-1; i++) {
		 str[i] = stream.read();
		 Serial.printf("%c",str[i]);
		 if (str[i] == '\r' || str[i] == '\n') {
		 str[i] = '\0';
		 //break;
		 }
		 }
		 */
		Serial.printf("\nCommand: %s, length=%d %d %d\n", str, availableCharsCount, str[availableCharsCount - 2],
				str[availableCharsCount - 1]);
		if (!strcmp(str, "connect")) {
			// connect to wifi
			WifiStation.config(wifi_sid.get(currWifiIndex), wifi_pass.get(currWifiIndex));
			WifiStation.enable(true);
		} else if (!strcmp(str, "ip")) {
			Serial.printf("ip: %s mac: %s\r\n", WifiStation.getIP().toString().c_str(), WifiStation.getMAC().c_str());
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

	uint8 slotNo = rboot_get_current_rom();
	String slotNoStr = String(slotNo);

	uint32 heapSize = system_get_free_heap_size();
	String heapSizeStr = String(heapSize);

	WebSocketsList &clients = server.getActiveWebSockets();
	for (int i = 0; i < clients.count(); i++) {
		clients[i].sendString(
				"Connected to station: " + wifi_sid.get(currWifiIndex) + ", ROM:" + slotNoStr + ", heapSize: "
						+ heapSizeStr + ", appVer:1.27, SDK version: " + system_get_sdk_version());
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

void startWebServer() {

	Serial.println("Starting web server...Phase1");

	server.setTimeOut(2000);
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

void connectOk(IPAddress ip, IPAddress mask, IPAddress gateway) {
//void connectOk() {
	String ipString = WifiStation.getIP().toString();
	Serial.println("I'm CONNECTED to AP_SSID=" + wifi_sid.get(currWifiIndex) + " IP: " + ipString);
	//String ipString = ip.toString();
	Serial.println("IP: " + ipString);

	startWebServer();

	if (ipString.equals("192.168.1.115") || ipString.equals("192.168.1.110")) {
// distance sensor
		Serial.println("MODE: LEUZE Distance sensor");
		udp.listen(udpServerPort);

		Serial.begin(57600);
		deltat = 100000;
		system_uart_swap();
		delegateDemoClass.begin();
		reportTimer.initializeMs(100, reportAnalogue).start();

	} else if (ipString.equals("192.168.1.113") || ipString.equals("192.168.1.112") || ipString.equals("192.168.1.21")
			|| ipString.equals("192.168.43.154") || ipString.equals("192.168.43.34")) {    //
// 4 axis stepper driver
		Serial.setCallback(serialCallBack);
		Serial.println("MODE: 4 Axis Stepper driver");

		Serial.println("Init ended.");
		Serial.println("Type 'help' and press enter for instructions.");
		Serial.println();

		deltat = 2000;

		if (ipString.equals("192.168.1.112"))
			parseGcode("reassign x=+0 y=+3 e=+1 z=+2");
		else if (ipString.equals("192.168.1.113"))
			parseGcode("reassign x=+0 y=+1 e=+3 z=+2");

		reportTimer.initializeMs(300, reportStatus).start();
		hardwareTimer.initializeUs(deltat, StepperTimerInt);
		hardwareTimer.startOnce();
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
	} else {
		Serial.setCallback(serialCallBack);
	}

}

void checkConnection() {
	if (!WifiStation.isConnected()) {
		Serial.printf("Try connection to: %s\n", wifi_sid.get(currWifiIndex).c_str());
		WifiStation.config(wifi_sid.get(currWifiIndex), wifi_pass.get(currWifiIndex), false);
		incrementNextWifiIndex();
		WifiStation.connect();
		//WifiStation.restartWaitConnection(connectOk, 10, connectNotOk);
		//WifiStation.waitConnection(connectOk, 10, connectNotOk);
	}
}

void STADisconnect(String ssid, uint8_t ssid_len, uint8_t bssid[6], uint8_t reason) {
	checkConnection();
}

void blink() {
	Serial.printf("blink: %i\n", (int) state);
	digitalWrite(LED_PIN, state);
	state = !state;
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

	wifi_sid.add(WIFI_SSID);
	wifi_pass.add(WIFI_PWD);

	/*
	 wifi_sid.add("Sintex");
	 wifi_pass.add("sintex92");
	 wifi_sid.add("AsusKZ");
	 wifi_pass.add("Doitman1");
	 */

	WifiStation.config(wifi_sid.get(currWifiIndex), wifi_pass.get(currWifiIndex));
	WifiAccessPoint.enable(false);
	WifiStation.enable(true);

	checkConnection();
	procTimer.initializeMs(15000, checkConnection).start();

	WifiEvents.onStationGotIP(connectOk);

	//WifiStation.waitConnection(connectOk, 10, connectNotOk);

	/*


	 //wifi_set_opmode (STATION_MODE);
	 WifiAccessPoint.enable(false);

	 wifi_sid.add("AndroidAp");
	 wifi_pass.add("Doitman1");

	 //WifiStation.config(wifi_sid.get(currWifiIndex) , wifi_pass.get(currWifiIndex));
	 WifiStation.config(wifi_sid.get(currWifiIndex), wifi_pass.get(currWifiIndex));

	 WifiEvents.onStationDisconnect(STADisconnect);

	 checkConnection();
	 //WifiStation.connect();
	 //WifiStation.waitConnection(connectOk, 20, connectNotOk);

	 //pinMode(LED_PIN, OUTPUT);
	 procTimer.initializeMs(1000, blink).start();
	 */
}

void onReceive(UdpConnection& connection, char *data, int size, IPAddress remoteIP, uint16_t remotePort) {
	char buf[60];
	char buf1[12];

	sendToClients("UDP received");
	debugf("UDP Sever callback from %s:%d, %d bytes", remoteIP.toString().c_str(), remotePort, size);

	// We implement string mode server for example
	Serial.print(">\t");
	Serial.print(data);

	floatAnalog = atof(analogResult.c_str()) / 10.0;
	dtostrf(floatAnalog, 7, 4, buf1);
	sprintf(buf, "%s", deblank(buf1));
	String message = String(buf);

	udp.sendStringTo(remoteIP, udpServerPort, message);
}
