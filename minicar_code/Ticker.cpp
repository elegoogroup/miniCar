
#include "Ticker.h"

Ticker::Ticker() {}
Ticker::~Ticker() {}

void Ticker::start(fptr callback, uint32_t timer, uint16_t repeat, resolution_t resolution)
{
	this->resolution = resolution;
	if (this->resolution == MICROS)
		timer *= 1000;
	this->timer = timer;
	this->repeat = repeat;
	this->callback = callback;

	if (this->callback == NULL)
		return;
	if (this->resolution == MILLIS)
		lastTime = millis();
	else
		lastTime = micros();
	enabled = true;
	counts = 0;
	status = RUNNING;
}

void Ticker::resume()
{
	if (callback == NULL)
		return;
	if (resolution == MILLIS)
		lastTime = millis() - diffTime;
	else
		lastTime = micros() - diffTime;
	if (status == STOPPED)
		counts = 0;
	enabled = true;
	status = RUNNING;
}

void Ticker::stop()
{
	enabled = false;
	counts = 0;
	status = STOPPED;
}

void Ticker::pause()
{
	if (resolution == MILLIS)
		diffTime = millis() - lastTime;
	else
		diffTime = micros() - lastTime;
	enabled = false;
	status = PAUSED;
}

void Ticker::update()
{
	if (tick())
		callback();
}

bool Ticker::tick()
{
	if (!enabled)
		return false;
	if (resolution == MILLIS)
	{
		if ((millis() - lastTime) >= timer)
		{
			lastTime = millis();
			if (repeat - counts == 1)
				enabled = false;
			counts++;
			return true;
		}
	}
	else
	{
		if ((micros() - lastTime) >= timer)
		{
			lastTime = micros();
			if (repeat - counts == 1)
				enabled = false;
			counts++;
			return true;
		}
	}
	return false;
}

void Ticker::interval(uint32_t timer)
{
	if (resolution == MICROS)
		timer *= 1000;
	this->timer = timer;
}

uint32_t Ticker::elapsed()
{
	if (resolution == MILLIS)
		return millis() - lastTime;
	else
		return micros() - lastTime;
}

status_t Ticker::state()
{
	return status;
}

uint32_t Ticker::counter()
{
	return counts;
}