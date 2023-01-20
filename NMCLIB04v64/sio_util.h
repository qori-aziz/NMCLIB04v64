#pragma once
//---------------------------------------------------------------------------//
// Change History                                                            //
//                                                                           //
// JRK  12/16/05 - Initial Version                                           //
//                                                                           //
//---------------------------------------------------------------------------//
#ifndef sio_utilH
#define sio_utilH
#endif

//Serial IO function prototypes:
extern "C"  __declspec(dllexport) void ErrorPrinting(int f);
extern "C"  __declspec(dllexport) int ErrorMsgBox(const char* msgstr);
extern "C"  __declspec(dllexport) int SimpleMsgBox(char* msgstr);
extern "C"  __declspec(dllexport) HANDLE SioOpen(char* name, unsigned int baudrate);
extern "C"  __declspec(dllexport) BOOL SioPutChars(HANDLE ComPort, char* stuff, int n);
extern "C"  __declspec(dllexport) DWORD SioGetChars(HANDLE ComPort, char* stuff, int n);
extern "C"  __declspec(dllexport) DWORD SioTest(HANDLE ComPort);
extern "C"  __declspec(dllexport) BOOL SioClrInbuf(HANDLE ComPort);
extern "C"  __declspec(dllexport) BOOL SioChangeBaud(HANDLE ComPort, unsigned int baudrate);
extern "C"  __declspec(dllexport) BOOL SioClose(HANDLE ComPort);
