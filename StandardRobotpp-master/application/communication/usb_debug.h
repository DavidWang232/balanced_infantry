#ifndef USB_DEBUG_H
#define USB_DEBUG_H
#include "struct_typedef.h"

extern void ModifyDebugDataPackage(uint8_t index, float data, const char * name);
extern void ModifyPidDebugDataPackage(float fdb, float ref, float pid_out);

#endif  // USB_DEBUG_H
