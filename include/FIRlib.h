//Dean Pantages
//Justin Murray
//FIR Library Header File

#include "types.h"

//Next Sample Function see FIRlib.c for implementation details
double nextSample(FilterData_t *, double);

//Initializes a FilterData_t struct see FIRlib.c for implementation details
FilterData_t * initializeFilter(int);

//Returns the filterd sonar values
void filterSonar(api_HANDLES_t *, FilterHandles_t *, double *, double *);

//Returns the filtered ir values
void filterIR(api_HANDLES_t *, FilterHandles_t *, double *, double *);
