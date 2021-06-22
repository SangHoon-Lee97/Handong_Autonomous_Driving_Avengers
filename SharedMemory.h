#ifndef SHAREDMEMORY
#define	SHAREDMEMORY

#include    <stdio.h>
#include    <stdlib.h>
#include    <math.h>
#include    <string.h>
#include    <memory.h>
#include    <conio.h>
#include    <time.h>
#include    <Windows.h>
#include    <tchar.h>
#include	<sys/types.h>


#define SERV_NAME1     TEXT("Xsens_smdat_ReadData")
#define SERV_NAME3     TEXT("Xsens_ReadData_Flag")

typedef struct Shared_Memeory
{
	int		SIM_count;
	double  SIM_Time;
	double  euler_x;
	double  euler_y;
	double  euler_z;
	double  acc_x;
	double  acc_y;
	double  acc_z;
	double  vel_x;
	double  vel_y;
	double  vel_z;
	double  rot_x;
	double  rot_y;
	double  rot_z;
}Xsens_smdat;

typedef struct Flag
{
	int		flag;
}ReadData_flag;

static HANDLE		hMampF_SER_Xsens; // handle, Mampping File, Server
static Xsens_smdat* SERV_smdat_Xsens;
static Xsens_smdat  dat_buf_Xsens;

static HANDLE		  hMampF_SER_Flag; // handle, Mampping File, Server
static ReadData_flag* SERV_smdat_Flag;
static ReadData_flag  dat_buf_Flag;

void CreateServerXsens(void);
void ClosedServerXsens(void);
void DataWrite(Xsens_smdat data);

void CreateServerFlag(void);
void ClosedServerFlag(void);
int  FlagRead(ReadData_flag* data);
#endif