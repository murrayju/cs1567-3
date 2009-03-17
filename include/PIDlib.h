//Justin Murray
//PIDlib header

#include "create_comms.h"

#define PI 3.141592654

#define ABSOLUTE_COORD

#define HALL_WIDTH 182 //width of hallway in centimeters
#define HALL_VAR 200	//hallway sonar max value acceptable
#define IR_VAR 35.0		//ir sensor variance

// Wall status defines
#define WALLS_BOTH		0
#define WALLS_LEFT_ONLY		1
#define WALLS_RIGHT_ONLY	2
#define WALLS_NONE		3

typedef struct _api_HANDLES {
	create_comm_t *c;
	turret_comm_t *t;
} api_HANDLES_t;

int bumped(api_HANDLES_t *);

double Move(api_HANDLES_t *, double, double);

double Turn(api_HANDLES_t *, double);
