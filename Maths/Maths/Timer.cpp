#include "Timer.h"

void Timer::Start()
{
	startTime = time(0);
}

void Timer::End()
{
	startTime = time(0);
}

long Timer::GetMilliseconds()
{
	return 0;
}

long Timer::GetMicroseconds()
{
	return 0;
}

long Timer::GetNanoseconds()
{
	return 0;
}
