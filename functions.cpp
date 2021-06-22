#ifndef SOURCE_USER_DEFINED_FUNCTIONS
#define SOURCE_USER_DEFINED_FUNCTIONS

#include <Windows.h>
#include <profileapi.h>

typedef double REAL;

double	GetWindowTime(void)
{
	LARGE_INTEGER	liEndCounter, liFrequency;

	QueryPerformanceCounter(&liEndCounter);
	QueryPerformanceFrequency(&liFrequency);

	return(liEndCounter.QuadPart / (REAL)(liFrequency.QuadPart) * 1000.0);

};

#endif