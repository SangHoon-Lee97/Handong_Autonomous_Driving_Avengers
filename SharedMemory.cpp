//#include "SharedMemory.h"
//#include    <stdio.h>
//#include    <stdlib.h>
//#include    <math.h>
//#include    <string.h>
//#include    <memory.h>
//#include    <conio.h>
//#include    <time.h>
//#include    <Windows.h>
//#include    <tchar.h>
//#include	<sys/types.h>
//
//
//void CreateServerXsens(void)
//{
//	hMampF_SER_Xsens = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
//		0, sizeof(Xsens_smdat), SERV_NAME1);
//
//	if (hMampF_SER_Xsens == NULL)
//	{
//		printf("Could not open file mapping object (%d).\n", GetLastError());
//		return;
//	}
//
//	SERV_smdat_Xsens = (Xsens_smdat*)MapViewOfFile(hMampF_SER_Xsens, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(Xsens_smdat));
//}
//
//void ClosedServerXsens(void)
//{
//	if (SERV_smdat_Xsens == NULL) return;
//	UnmapViewOfFile(SERV_smdat_Xsens);
//	CloseHandle(hMampF_SER_Xsens);
//}
//
//void  DataWrite(Xsens_smdat data)
//{
//	if (SERV_smdat_Xsens == NULL) return;
//	*SERV_smdat_Xsens = data;
//
//	//printf("Write Data -> SIM_Count : %d\t SIM_Time : %f\t SIN_Wave : %f\n", data.SIM_count, data.SIM_Time, data.SIN_Wave);
//}
//
//void CreateServerFlag(void)
//{
//	hMampF_SER_Flag = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
//		0, sizeof(ReadData_flag), SERV_NAME3);
//
//	if (hMampF_SER_Flag == NULL)
//	{
//		//printf("Could not open file mapping object (%d).\n", GetLastError());
//		return;
//	}
//
//	SERV_smdat_Flag = (ReadData_flag*)MapViewOfFile(hMampF_SER_Flag, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(ReadData_flag));
//}
//
//void ClosedServerFlag(void)
//{
//	if (SERV_smdat_Flag == NULL) return;
//	UnmapViewOfFile(SERV_smdat_Flag);
//	CloseHandle(hMampF_SER_Flag);
//}
//
//int  FlagRead(ReadData_flag* data)
//{
//	printf("\nMain COM flag : %d", data->flag);
//	return data->flag;
//}