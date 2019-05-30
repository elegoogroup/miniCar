#ifndef TICKER_H
#define TICKER_H

#include "Arduino.h"

enum resolution_t
{
	MICROS,
	MILLIS,
	MICROS_MICROS
};

enum status_t
{
	STOPPED,
	RUNNING,
	PAUSED
};

typedef void (*fptr)();

class Ticker
{

public:
	Ticker();

	~Ticker();

	void start(fptr callback, uint32_t timer, uint16_t repeat = 0, resolution_t resolution = MICROS);

	void resume();

	void pause();

	void stop();

	void update();

	void interval(uint32_t timer);

	uint32_t elapsed();

	status_t state();

	uint32_t counter();

private:
	bool tick();
	bool enabled;
	uint32_t timer;
	uint16_t repeat;
	resolution_t resolution = MICROS;
	uint32_t counts;
	status_t status;
	fptr callback;
	uint32_t lastTime;
	uint32_t diffTime;
};

#endif
