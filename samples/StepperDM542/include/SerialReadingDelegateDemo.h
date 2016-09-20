#include <SmingCore.h>

#ifndef INCLUDE_SERIALREADINGDELEGATEDEMO_H_
#define INCLUDE_SERIALREADINGDELEGATEDEMO_H_

extern String analogResult;
extern void sendToClients(String message);
String tempAnalogResult="";

//*** Example of global callback routine
void onDataCallback(Stream& stream, char arrivedChar, unsigned short availableCharsCount)
{
}

//*** Example of class callback processing
class SerialReadingDelegateDemo
{
public:
	void begin()
	{
		Serial.setCallback(StreamDataReceivedDelegate(&SerialReadingDelegateDemo::onData, this));
		debugf("hwsDelegateDemo instantiated, waiting for data");
	};

	void onData(Stream& stream, char arrivedChar, unsigned short availableCharsCount)
	{
		if (arrivedChar == 0x0D) // Lets show data!
		{
			//
			if(tempAnalogResult.length() == 5)
				analogResult = tempAnalogResult;
			tempAnalogResult = "";
		}
		else
		{
			tempAnalogResult += arrivedChar;
			//sendToClients(tempAnalogResult);
		}
	}

private:
	unsigned charReceived = 0;
	bool useRxFlag = true;
};


#endif /* INCLUDE_SERIALREADINGDELEGATEDEMO_H_ */
