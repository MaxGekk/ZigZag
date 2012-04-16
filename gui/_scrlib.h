/**
 * This header file contains declarations necessary for interaction of the platform with Screen
 * library and should not be normally used by the application developer
 */
#ifndef __SCRLIB_INT_H
#define __SCRLIB_INT_H

#include "scrlib.h"

/**
 * Initialization of the screen library
 */
void InitializeScrLib(void);

/**
 * Passes the focus to the next control
 */
void RollFocus();

void FireKeyMessage(enum zzMessages msg);

void ChangeCarraigePositon(char shift);

void CarriageState(void);

#endif
