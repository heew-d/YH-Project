#include "SerialTypedf.hpp"
#include "SerialCOM.hpp"
#include "UsbSerial.hpp"


int MW_SerialOpen(char *Port, int BaudRate, int _T, int _M);
int MW_SerialRecv(long *ID, int *length, char data[8]);
int MW_SerialSend(long ID, char data[8]);
int Mw_SerialClose(void);

