#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include <Arduino.h>
#include <pins_arduino.h>

// Probably shouldn't change these values unless you really know what you're doing.
#define NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance. Default=0
#define MAX_SENSOR_DELAY 5800   // Maximum uS it takes for sensor to start the ping. Default=5800
#define PING_OVERHEAD 5         // Ping overhead in microseconds (uS). Default=5
#define SPEED_OF_SOUND_CM      34 // 34cm/ms
#define SPEED_OF_SOUND_INCH    13.3858f // 13.4 inch/ms

class ultrasonic 
{
	public:
		ultrasonic(byte trigger, byte echo, int maxDistanceCm);
		int measureDistance(bool isCm);

	private:
		bool trigger();
		float microsecondsToCentimeters(long microseconds);
		float microsecondsToInches(long microseconds);
		uint8_t _triggerBit;
		uint8_t _echoBit;
		volatile uint8_t *_triggerOutput;
		volatile uint8_t *_echoInput;
		volatile uint8_t *_triggerMode;
		unsigned int _maxEchoTime;
		unsigned long _max_time;
};

#endif
