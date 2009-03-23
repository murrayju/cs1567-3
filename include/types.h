//Headers for all structs and defines
#ifndef __types_h
#define __types_h

#include "TurretAPI.h"
#include "create_comms.h"

#define DEBUG
#define ABSOLUTE_COORD

#define PI 3.141592654

#define HALL_WIDTH 240  //width of hallway in centimeters
#define HALL_VAR 220	//hallway sonar max value acceptable
#define IR_VAR 45.0	//ir sensor variance

#define TIMEOUT 0.1

// Wall status defines
#define WALLS_BOTH		3
#define WALLS_LEFT_ONLY		2
#define WALLS_RIGHT_ONLY	1
#define WALLS_NONE		0

//number of filter samples
#define MAX_SAMPLES 16

//defines filter type
#define IR	0
#define SONAR	1

//defines number of samples
#define IR_SAMPLES	16
#define SONAR_SAMPLES	16

//define max sample value
#define IR_MAX		50
#define SONAR_MAX	260

#define T_ANGLE 93.0
#define COMPORT "/dev/ttyS2"

//Filter Coefficients
#define SONAR_COEFFS 	{0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}
#define IR_COEFFS 	{0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}//{0.211939524148286456695089441382151562721,0.471832709834875574372858864080626517534,0.471832709834875574372858864080626517534,0.211939524148286456695089441382151562721}

typedef struct _api_HANDLES {
	create_comm_t *c;
	turret_comm_t *t;
} api_HANDLES_t;

//Filter data struct
typedef struct _FilterData {
	int max;
	int num_samples;
	double coefficients[MAX_SAMPLES];
	int  next_index;
	double samples[MAX_SAMPLES];
} FilterData_t;

typedef struct _FilterHandles {
	FilterData_t * sonarL;
	FilterData_t * sonarR;
	FilterData_t * ir;
} FilterHandles_t;

typedef struct _coordData {
	double X;
	double Y;
} coordData_t;

#endif
