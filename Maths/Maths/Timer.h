#include <time.h>

class Timer {
public:
	void Start();
	void End();
	long GetMilliseconds();
	long GetMicroseconds();
	long GetNanoseconds();
private:
	long startTime;
};