#include "ultrasonic.h"

// Optimize the NewPing library for HCSR04 for code size reduction and only used
// in arduino uno, do not use the standard library from Arduino due to bad review
// about performance and accuracy

// Constructor of the ultra sonic class, u
ultrasonic::ultrasonic(byte trigger, byte echo, int maxDistanceCm) 
{
	// Get the corresponding bit for trigger and echo pin
	_triggerBit = digitalPinToBitMask(trigger);
	_echoBit = digitalPinToBitMask(echo);      

	// Get the corresponding output port register for trigger and echo pin
	_triggerOutput = portOutputRegister(digitalPinToPort(trigger));
	_echoInput = portInputRegister(digitalPinToPort(echo));

	// Get the port mode register for the trigger pin
	_triggerMode = (uint8_t *) portModeRegister(digitalPinToPort(trigger));

	_maxEchoTime = (int)((maxDistanceCm * 2)/SPEED_OF_SOUND_CM) ;
}

int ultrasonic::measureDistance(bool isCm)
{
	int ret;
	if (!trigger()) return NO_ECHO; // Trigger a ping, if it returns false, return NO_ECHO to the calling function.

	while (*_echoInput & _echoBit)                // Wait for the ping echo.
		if (micros() > _max_time) return NO_ECHO;   // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.

	int duration = (micros() - (_max_time - _maxEchoTime) - PING_OVERHEAD);

	if(isCm)
	{
		ret = (int)microsecondsToCentimeters(duration);
	}
	else
	{
		ret = (int)microsecondsToInches(duration);
	}

	return ret;
}

// Time to distance convertor
float ultrasonic::microsecondsToInches(long microseconds) 
{
	return ((microseconds*SPEED_OF_SOUND_INCH)/2); 
}

float ultrasonic::microsecondsToCentimeters(long microseconds) 
{
	return ((microseconds*SPEED_OF_SOUND_CM)/2); 
}

bool ultrasonic::trigger() 
{
	*_triggerOutput &= ~_triggerBit; // Set the trigger pin low, should already be low, but this will make sure it is.
	delayMicroseconds(2);            // Wait for pin to go low.
	*_triggerOutput |= _triggerBit;  // Set trigger pin high, this tells the sensor to send out a ping.
	delayMicroseconds(10);           // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
	*_triggerOutput &= ~_triggerBit; // Set trigger pin back to low.

	if (*_echoInput & _echoBit) return false;               // Previous ping hasn't finished, abort.
	_max_time = micros() + _maxEchoTime + MAX_SENSOR_DELAY; // Maximum time we'll wait for ping to start (most sensors are <450uS, the SRF06 can take up to 34,300uS!)
	while (!(*_echoInput & _echoBit))                       // Wait for ping to start.
		if (micros() > _max_time) return false;               // Took too long to start, abort.

	_max_time = micros() + _maxEchoTime; // Ping started, set the time-out.
	return true;                         // Ping started successfully.
}
