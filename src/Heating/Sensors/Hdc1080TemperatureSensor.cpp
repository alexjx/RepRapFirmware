/*
 * Hdc1080TemperatureSensor.cpp
 *
 *  Created on:
 *      Author: J.Xin
 */

#include "Hdc1080TemperatureSensor.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/StepTimer.h"
#include "Hardware/I2C.h"

#if SUPPORT_HDC1080_SENSOR

constexpr uint32_t MinimumReadInterval = 2000;		// ms
constexpr uint32_t MeasurementDelay = 20; //ms

#include "Tasks.h"

constexpr uint16_t HDC1080_ADDRESS = 0b1000000;

constexpr uint8_t HDC1080_REG_TEMPERATURE = 0x00;
constexpr uint8_t HDC1080_REG_HUMIDITY = 0x01;
constexpr uint8_t HDC1080_REG_CONFIGURATION = 0x02;
constexpr uint8_t HDC1080_REG_SERIAL_ID_1 = 0xFB;
constexpr uint8_t HDC1080_REG_SERIAL_ID_2 = 0xFC;
constexpr uint8_t HDC1080_REG_SERIAL_ID_3 = 0xFD;
constexpr uint8_t HDC1080_REG_MANUFACTURE_ID = 0xFE;
constexpr uint8_t HDC1080_REG_DEVICE_ID = 0xFF;

constexpr uint8_t HDC1080_MODE_EITHER = 0b0;
constexpr uint8_t HDC1080_MODE_BOTH = 0b1;

constexpr uint8_t HDC1080_TRES_14BIT = 0b0;
constexpr uint8_t HDC1080_TRES_11BIT = 0b1;

constexpr uint8_t HDC1080_HRES_14BIT = 0b00;
constexpr uint8_t HDC1080_HRES_11BIT = 0b01;
constexpr uint8_t HDC1080_HRES_8BIT = 0b11;

constexpr uint16_t HDC1080_MANUFACTURE_ID = 0x5449;
constexpr uint16_t HDC1080_DEVICE_ID = 0x1050;

class Hdc1080Hardware {
public:
	bool Configure(uint8_t tRes = HDC1080_TRES_14BIT, uint8_t hRes = HDC1080_HRES_14BIT, bool heater = false) {
		uint16_t mid = readRegistor(HDC1080_REG_MANUFACTURE_ID);
		if (!mid) {
			return false;
		}

		uint16_t did = readRegistor(HDC1080_REG_DEVICE_ID);
		if (!did) {
			return false;
		}

		if (mid != HDC1080_MANUFACTURE_ID || did != HDC1080_DEVICE_ID) {
			return false;
		}

		_config = ((heater) ? 1 << 13 : 0) | (HDC1080_MODE_EITHER << 12) | (tRes << 10) | (hRes << 8);
		if (!writeRegistor(HDC1080_REG_CONFIGURATION, _config)) {
			return false;
		}

		return true;
	}

	TemperatureError GetTemperature(float& t) {
		TemperatureError err = TemperatureError::success;
		uint16_t r = triggerMeasurement(HDC1080_REG_TEMPERATURE, &err);
		if (r) {
			t = float(r) * (float) 165.0 / (float) 65536.0 - (float) 40.0;
		}
		return err;
	}

	TemperatureError GetHumidity(float& h) {
		TemperatureError err = TemperatureError::success;
		uint16_t r = triggerMeasurement(HDC1080_REG_HUMIDITY, &err);
		if (r) {
			h = float(r) * (float) 100.0 / (float) 65536.0;
		}
		return err;
	}

private:

	uint16_t readRegistor(uint8_t reg, TemperatureError* err = nullptr)
	{
		uint8_t data[3] = {0};
		data[0] = reg;

		auto t = I2C::Transfer(HDC1080_ADDRESS, data, 1, 2);
		if (t != 3) {
			if (err != nullptr) {
				*err = TemperatureError::badResponse;
				if (t == 0) {
					*err = TemperatureError::ioError;
				}
			}
			return 0;
		}
		return (data[1] << 8) | data[2];
	}

	uint16_t triggerMeasurement(uint8_t reg, TemperatureError* err = nullptr)
	{
		uint8_t data[3] = {0};

		if (I2C::Transfer(HDC1080_ADDRESS, &reg, 1, 0) != 1) {
			if (err != nullptr) {
				*err = TemperatureError::ioError;
			}
			return 0;
		}

		// delay per datasheet
		delay(MeasurementDelay);

		if (I2C::Transfer(HDC1080_ADDRESS, data, 0, 2) != 2) {
			if (err != nullptr) {
				*err = TemperatureError::notReady;
			}
			return 0;
		}

		return (data[0] << 8) | data[1];
	}

	bool writeRegistor(uint8_t reg, uint16_t val, TemperatureError* err = nullptr) {
		auto t = I2C::Transfer(HDC1080_ADDRESS, (uint8_t*) &val, 2, 0);
		if (err != nullptr) {
			if (t < 2) {
				*err = TemperatureError::ioError;
			} else {
				*err = TemperatureError::success;
			}
		}
		return t == 2;
	}

private:
	uint16_t _config;
};

//////////////////////////////////////////////
// Buffer Interface
//////////////////////////////////////////////

static Hdc1080Hardware* _sensor = nullptr;
static float sensorReadings[2] = { 0.0, 0.0 };
static TemperatureError sensorLastErrors[2] = { TemperatureError::success, TemperatureError::success };
static Mutex sensorMutex;
static constexpr unsigned int TaskStackWords = 150;
static Task<TaskStackWords>* sensorTask = nullptr;

extern "C" [[noreturn]] void Hdc1080TaskRoutine(void * pvParameters) {
	Hdc1080HwInterface::SensorTask();
}

void Hdc1080HwInterface::InitStatic() {
	sensorMutex.Create("HDC1080");
}

bool Hdc1080HwInterface::Configure() {
	MutexLocker lock(sensorMutex);

	if (_sensor == nullptr) {
		_sensor = new Hdc1080Hardware();
	}

	auto ok = _sensor->Configure();

	if (ok && sensorTask == nullptr) {
		sensorTask = new Task<TaskStackWords>;
		sensorTask->Create(Hdc1080TaskRoutine, "HDC1080", nullptr, TaskPriority::DhtPriority);
	}

	return ok;
}

[[noreturn]] void Hdc1080HwInterface::SensorTask() {
	for (;;) {
		{
			MutexLocker lock(sensorMutex);
			if (_sensor != nullptr) {
				sensorLastErrors[0] = _sensor->GetTemperature(sensorReadings[0]);
				sensorLastErrors[1] = _sensor->GetHumidity(sensorReadings[1]);
			}
		}
		delay(MinimumReadInterval);
	}
}

TemperatureError Hdc1080HwInterface::GetTemperature(float &t) {
	t = sensorReadings[0];
	return sensorLastErrors[0];
}

TemperatureError Hdc1080HwInterface::GetHumidity(float &h) {
	h = sensorReadings[1];
	return sensorLastErrors[1];
}

//////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////
// Class Hdc1080TemperatureSensor members
Hdc1080TemperatureSensor::Hdc1080TemperatureSensor(unsigned int channel) :
		TemperatureSensor(channel, "HDC1080-temperature") {
}

void Hdc1080TemperatureSensor::Init() {
	I2C::Init();
}

GCodeResult Hdc1080TemperatureSensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) {
	GCodeResult rslt = GCodeResult::ok;
	if (mCode == 305) {
		bool seen = false;
		TryConfigureHeaterName(gb, seen);

		if (!Hdc1080HwInterface::Configure()) {
			reply.copy("HDC1080 sensor could not be detected");
			rslt = GCodeResult::error;
		} else {
			reply.copy("HDC1080 sensor running");
		}

		if (!seen && !gb.Seen('X')) {
			CopyBasicHeaterDetails(heater, reply);
		}
	}
	return rslt;
}

Hdc1080TemperatureSensor::~Hdc1080TemperatureSensor() {
	// We don't delete the hardware interface object because the humidity channel may still be using it
}

TemperatureError Hdc1080TemperatureSensor::TryGetTemperature(float& t) {
	return Hdc1080HwInterface::GetTemperature(t);
}

// Class Hdc1080HumiditySensor members
Hdc1080HumiditySensor::Hdc1080HumiditySensor(unsigned int channel) :
		TemperatureSensor(channel, "HDC1080-humidity") {
}

void Hdc1080HumiditySensor::Init() {
	I2C::Init();
}

GCodeResult Hdc1080HumiditySensor::Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, const StringRef& reply) {
	GCodeResult rslt = GCodeResult::ok;
	if (mCode == 305) {
		bool seen = false;
		TryConfigureHeaterName(gb, seen);

		if (!Hdc1080HwInterface::Configure()) {
			reply.copy("HDC1080 sensor could not be detected");
			rslt = GCodeResult::error;
		} else {
			reply.copy("HDC1080 sensor running");
		}

		if (!seen && !gb.Seen('X')) {
			CopyBasicHeaterDetails(heater, reply);
		}
	}
	return rslt;
}

Hdc1080HumiditySensor::~Hdc1080HumiditySensor() {
	// We don't delete the hardware interface object because the temperature channel may still be using it
}

TemperatureError Hdc1080HumiditySensor::TryGetTemperature(float& t) {
	return Hdc1080HwInterface::GetHumidity(t);
}

#endif /*SUPPORT_HDC1080_SENSOR*/
