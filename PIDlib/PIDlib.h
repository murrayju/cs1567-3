//Justin Murray
//PIDlib header

#include <libplayerc/playerc.h>

#define PI 3.141592654

#define ABSOLUTE_COORD

#define HALL_WIDTH 182 //width of hallway in centimeters
#define HALL_VAR 200	//hallway sonar max value acceptable
#define IR_VAR 35.0

// Wall status defines
#define WALLS_BOTH		0
#define WALLS_LEFT_ONLY		1
#define WALLS_RIGHT_ONLY	2
#define WALLS_NONE		3

typedef struct _playerc_HANDLES {
	playerc_client_t * client;
	playerc_position2d_t * pos2d;	
	
	//defines range devices based on the environment
#ifdef stage_environment
  	playerc_ranger_t * sonar;
  	playerc_ranger_t * ir;
#else
	playerc_position1d_t * pos1d;
  	playerc_bumper_t * bumper;
  	playerc_sonar_t * sonar;
  	playerc_ir_t	* ir;
	playerc_power_t * power;
#endif
} playerc_HANDLES_t;

int bumped(playerc_HANDLES_t *);

double Move(playerc_HANDLES_t *, double, double);

double Turn(playerc_HANDLES_t *, double);
