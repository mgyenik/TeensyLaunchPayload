#ifndef SERIAL_UTIL_H
#define SERIAL_UTIL_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <stdarg.h>

#define ARRAY_SIZE(__a) (int)(sizeof(__a)/sizeof((__a)[0]))

// Format printing template, specialized for format printing that is supported.
template<typename T>
static void pprintf(Print *p, T fmt, ... );

// Specialized for string literals.
template<>
void pprintf(Print *p, const char *fmt, ... ) {
	// Sting truncated at 128 chars, can be increased if stack depth not
	// concern.
	char buf[128];
	va_list args;

	va_start(args, fmt );
	vsnprintf(buf, 128, fmt, args);
	va_end(args);
	p->print(buf);
}

// Map from sensor type to a static string.
static inline const char *sensorTypeName(const sensor_t& sensor) {
	const char *type_name;
	switch (sensor.type) {
	case SENSOR_TYPE_ACCELEROMETER:
		type_name = "ACCELEROMETER";
		break;
	case SENSOR_TYPE_MAGNETIC_FIELD:
		type_name = "MAGNETOMETER";
		break;
	case SENSOR_TYPE_GYROSCOPE:
		type_name = "GYROSCOPE";
		break;
	case SENSOR_TYPE_PRESSURE:
		type_name = "BAROMETER";
		break;
	default:
		type_name = "UNKNOWN";
		break;
	}
	return type_name;
}

// Map form sensor type to static string for the units.
static inline const char *sensorTypeUnits(const sensor_t& sensor) {
	const char *unit_name;
	switch (sensor.type) {
	case SENSOR_TYPE_ACCELEROMETER:
		unit_name = "m/s^2";
		break;
	case SENSOR_TYPE_MAGNETIC_FIELD:
		unit_name = "uT";
		break;
	case SENSOR_TYPE_GYROSCOPE:
		unit_name = "rad/s";
		break;
	case SENSOR_TYPE_PRESSURE:
		unit_name = "hPa";
		break;
	default:
		unit_name = "unknown units";
		break;
	}
	return unit_name;
}

// Interface for SensorWriters, SensorWriters sample sensors and serialize the
// output to Print sinks. Classes that implement this interface help eliminate
// redundancy in printing.
class SensorWriter {
 public:
	SensorWriter() {}
	virtual ~SensorWriter() {}

	// Take sample from sensor so that all logging sources get the same value.
	virtual void Update() {};

 	// Log this sample to the given stream.
	virtual void Write(Print *p) const {}
};

// 
class UnifiedSensorWriter : public SensorWriter {
 public:
	UnifiedSensorWriter(Adafruit_Sensor *sensor);
	void Update();
 	void Write(Print *p);

 private:
	Adafruit_Sensor *sensor_;
	sensors_event_t event_;
	const char *name_;
	const char *unit_;
};

// Simple class to eliminate redundancy in printing.
class AltitudeSensorWriter : public SensorWriter {
 public:
	AltitudeSensorWriter(Adafruit_BMP085_Unified *bmp);
	void Update();
 	void Write(Print *p);

 private:
	Adafruit_BMP085_Unified *bmp_;
	sensors_event_t event_;
	const char *name_;
	const char *unit_;
};

void sensorDetailsOut(const sensor_t& sensor, Stream *serial);

#endif // SERIAL_UTIL_H
