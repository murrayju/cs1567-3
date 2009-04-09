//Justin Murray
//PIDlib header

#include "types.h"

int bumped(api_HANDLES_t *);

double Move(api_HANDLES_t *, FilterHandles_t *, pidHandles_t *, double, double);
double Turn(api_HANDLES_t *, FilterHandles_t *, pidHandles_t *, double);
void * mapRobot(void *);

//Initializes a pidData_t struct see PIDlib.c for implementation details
pidData_t * initializePID(int);
