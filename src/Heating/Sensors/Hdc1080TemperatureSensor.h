/*
 * Hdc1080TemperatureSensor.h
 *
 *  Created on:
 *      Author: J.Xin
 */

#ifndef SRC_HEATING_HDC1080TEMPERATURESENSOR_H_
#define SRC_HEATING_HDC1080TEMPERATURESENSOR_H_
#include "RepRapFirmware.h"



#if SUPPORT_HDC1080_SENSOR

#include "TemperatureSensor.h"
#include "RTOSIface/RTOSIface.h"

class Hdc1080HwInterface {
public:
	static void InitStatic();
	static bool Configure();

	static TemperatureError GetTemperature(float& t);
	static TemperatureError GetHumidity(float& h);

	static void SensorTask();
};

// This class represents a DHT temperature sensor
class Hdc1080TemperatureSensor: public TemperatureSensor {
public:
	Hdc1080TemperatureSensor(unsigned int channel);
	~Hdc1080TemperatureSensor();

	GCodeResult Configure(unsigned int mCode, unsigned int heater,
			GCodeBuffer& gb, const StringRef& reply) override;
	void Init() override;

protected:
	TemperatureError TryGetTemperature(float& t) override;
};

// This class represents a DHT humidity sensor
class Hdc1080HumiditySensor: public TemperatureSensor {
public:
	Hdc1080HumiditySensor(unsigned int channel);
	~Hdc1080HumiditySensor();

	GCodeResult Configure(unsigned int mCode, unsigned int heater,
			GCodeBuffer& gb, const StringRef& reply) override;
	void Init() override;

protected:
	TemperatureError TryGetTemperature(float& t) override;
};

#endif /*SUPPORT_HDC1080_SENSOR*/

#endif /* SRC_HEATING_HDC1080TEMPERATURESENSOR_H_ */
