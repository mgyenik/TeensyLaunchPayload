#include <Wire.h>
#include <Adafruit_Sensor.h>

#include "serial_util.h"

// Helper for dumping out details during initialization.
void sensorDetailsOut(const sensor_t& sensor, Stream *serial) {
	// Now write details to the serial output.
	pprintf(serial, "----------- %s ----------\n"
					"Sensor:       %s\n"
					"Driver Ver:   %d\n"
					"Unique ID:    %d\n"
					"------------------------------------\n\n",
					sensorTypeName(sensor), sensor.name, sensor.version, sensor.sensor_id);
}

static inline char *strf(char *buf, float f) {
	return dtostrf(f, 7, 4, buf);
}

static inline void pprint_sensor_vec(const sensors_vec_t& vec, Print *p) {
	char buf[16]; // No %f in sprintf? What the fuck.
	pprintf(p, "X: %s Y: %s Z: %s ", strf(buf, vec.x), strf(buf, vec.y), strf(buf, vec.z));
}

UnifiedSensorWriter::UnifiedSensorWriter(Adafruit_Sensor *sensor) : sensor_(sensor) {
	sensor_t s;
	sensor->getSensor(&s);
	name_ = sensorTypeName(s);
	unit_ = sensorTypeUnits(s);
}

void UnifiedSensorWriter::Update() {
	sensor_->getEvent(&event_);
}

void UnifiedSensorWriter::Write(Print *p) {
  sensor_t s;
  sensor_->getSensor(&s);

	// The name is always written.
	p->print(name_);

	// Write out body, format and source of data in event union depends on type...
	switch (s.type) {
	case SENSOR_TYPE_ACCELEROMETER:
		pprint_sensor_vec(event_.acceleration, p);
		break;
	case SENSOR_TYPE_MAGNETIC_FIELD:
		pprint_sensor_vec(event_.gyro, p);
		break;
	case SENSOR_TYPE_GYROSCOPE:
		pprint_sensor_vec(event_.magnetic, p);
		break;
	}

	// The unit is always written.
	pprintf(p, " %s\n", unit_);
}

AltitudeSensorWriter::AltitudeSensorWriter(Adafruit_BMP085_Unified *sensor) : bmp_(sensor) {
	sensor_t s;
	sensor->getSensor(&s);
	name_ = sensorTypeName(s);
	unit_ = sensorTypeUnits(s);
}

void AltitudeSensorWriter::Update() {
	bmp_->getEvent(&event_);
}

void AltitudeSensorWriter::Write(Print *p) {
	// TODO(mgyenik) Find clean way to update this.
	const float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
	float alt = bmp_->pressureToAltitude(seaLevelPressure, event_.pressure); 
	char buf[16]; // No %f in sprintf? What the fuck.
	pprintf(p, "%s %s %s\n", name_, strf(buf, alt), unit_);
}
