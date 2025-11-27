#ifndef _AWG_H
#define _AWG_H

#include "main.h"

void moveF(void);
void LoadAWGvars(void);
void SaveAWGvars(void);
void BuildWave(void);

// Global AWG variable
extern uint8_t  AWGBuffer[BUFFER_AWG];
extern uint8_t  cycles;     // Cycles in AWG buffer

#endif
