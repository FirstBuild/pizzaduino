/*
 * ac_input.h
 * 
 * Read the AC inputs
 */

#ifndef AC_INPUT_H
#define AC_INPUT_H

#include <stdint.h>

// Call setupAcInputs in the setup function to initialize.
void setupAcInputs(void);

// Call runAcInputs from the loop function.
void runAcInputs(void);

// Call the get functions to get the status of the AC inputs
bool getAcInputOne(void);
bool getAcInputTwo(void);

#endif
