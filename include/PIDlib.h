//Justin Murray
//PIDlib header

#include "types.h"

int bumped(api_HANDLES_t *);

double Move(api_HANDLES_t *, FilterHandles_t *, double, double);

double Turn(api_HANDLES_t *, FilterHandles_t *, double);
double Turn2(api_HANDLES_t * dev, FilterHandles_t * filter, double A, int R);
