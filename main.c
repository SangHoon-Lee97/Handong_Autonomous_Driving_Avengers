#include "include.h"

// define �ʿ��� ����
// define �ʿ��� array
// XsStatusFlag enum
// Callback Handler class
//
// Xsens �����Ű�� -> measurement mode ����


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