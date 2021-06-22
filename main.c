#include "include.h"

// define 필요한 변수
// define 필요한 array
// XsStatusFlag enum
// Callback Handler class
//
// Xsens 연결시키고 -> measurement mode 까지


void main(void) {

	//Callback();
	InitializeXsens();
	ConfigureSensor(); 

	while (!Manage.StopCond) {
		CheckTime();
		Xsens_Interface();
		Manage.StopCond = CheckStop();
	}

	Xsens_WriteData();
	printf("\n\t Task finish");

	return;
}