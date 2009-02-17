//Justin Murray
//PIDlib header

#include <libplayerc/playerc.h>

#define PI 3.141592654

#define ABSOLUTE_COORD

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
} playerc_HANDLES;

int bumped(playerc_HANDLES *);

double Move(playerc_HANDLES *, double, double);

double Turn(playerc_HANDLES *, double);
