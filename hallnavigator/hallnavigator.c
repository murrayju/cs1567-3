//Justin Murray
//Dean Pantages
//Main Hallway Navigator Executable

#include <stdio.h>
#include <libplayerc/playerc.h>

//#define stage_environment

#ifdef stage_environment
#define SERVER "localhost"
#define PORT	12121
#else
#define SERVER "gort"
#define PORT	9876
#define T_ANGLE 90.0
#endif

typedef struct _playerc_HANDLES {
	playerc_client_t * client;
	playerc_position2d_t * pos2d;
	playerc_position1d_t * pos1d;
	
	//defines range devices based on the environment
#ifdef stage_environment
  	playerc_ranger_t * sonar;
  	playerc_ranger_t * ir;
#else
  	playerc_bumper_t * bumper;
  	playerc_sonar_t * sonar;
  	playerc_ir_t	* ir;
	playerc_power_t * power;
#endif
} playerc_HANDLES;

int main(int argc, const char **argv) {
	int i;
	playerc_HANDLES hnd;

	
	// Create a client object and connect to the server
	hnd.client = playerc_client_create(NULL, SERVER, PORT);
  	if (playerc_client_connect(hnd.client) != 0) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
	printf("Connected to Robot\n");
	
	// Create a position2d proxy (device id "position2d:0") and susbscribe
	// in read/write mode
	hnd.pos2d = playerc_position2d_create(hnd.client, 0);
	if (playerc_position2d_subscribe(hnd.pos2d, PLAYERC_OPEN_MODE) != 0) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
	
	hnd.pos1d = playerc_position1d_create(hnd.client, 0);
	if (playerc_position1d_subscribe(hnd.pos1d, PLAYERC_OPEN_MODE) != 0) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
	
	// Create and subscribe proxies based on stage_environment
#ifdef stage_environment
	hnd.sonar = playerc_ranger_create(hnd.client, 0);
	if(playerc_ranger_subscribe(hnd.sonar, PLAYERC_OPEN_MODE)) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
	
	hnd.ir = playerc_ranger_create(hnd.client, 1);
	if(playerc_ranger_subscribe(hnd.ir, PLAYERC_OPEN_MODE)) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
#else
	hnd.bumper = playerc_bumper_create(hnd.client, 0);
	if(playerc_bumper_subscribe(hnd.bumper, PLAYERC_OPEN_MODE)) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
	
	hnd.sonar = playerc_sonar_create(hnd.client, 0);
	if(playerc_sonar_subscribe(hnd.sonar, PLAYERC_OPEN_MODE)) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
	
	hnd.ir = playerc_ir_create(hnd.client, 1);
	if(playerc_ir_subscribe(hnd.ir, PLAYERC_OPEN_MODE)) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
	
	hnd.power = playerc_power_create(hnd.client, 0);
	if(playerc_power_subscribe(hnd.power, PLAYERC_OPEN_MODE)) {
		fprintf(stderr, "error: %s\n", playerc_error_str());
		return -1;
	}
#endif
	// Enable the robots motors and rotates turret servo POSSIBLE ISSUE HERE 
	//I THINK THIS NEEDS TO GO IN THE IFDEF BECAUSE THE POS1D doesnt exist in stage
	playerc_position2d_enable(hnd.pos2d, 1);
	playerc_position1d_enable(hnd.pos1d, 1);
	playerc_position1d_set_cmd_pos(hnd.pos1d, T_ANGLE,0);
	
	for (i = 0; i < 200; i++) {
		// Read data from the server and display current robot position
		playerc_client_read(hnd.client);
		printf("pos2d: %f %f %f  pos1d: %f\n",	hnd.pos2d->px, hnd.pos2d->py, hnd.pos2d->pa, hnd.pos1d->pos);
	} 
	
	// Shutdown and tidy up.  Close all of the proxy handles
	playerc_position2d_unsubscribe(hnd.pos2d);
	playerc_position2d_destroy(hnd.pos2d);
	playerc_position1d_unsubscribe(hnd.pos1d);
	playerc_position1d_destroy(hnd.pos1d);
#ifdef stage_environment
	playerc_ranger_unsubscribe(hnd.sonar);
	playerc_ranger_destroy(hnd.sonar);
	playerc_ranger_unsubscribe(hnd.ir);
	playerc_ranger_destroy(hnd.ir);
#else
	playerc_bumper_unsubscribe(hnd.bumper);
	playerc_bumper_destroy(hnd.bumper);
	playerc_sonar_unsubscribe(hnd.sonar);
	playerc_sonar_destroy(hnd.sonar);
	playerc_ir_unsubscribe(hnd.ir);
	playerc_ir_destroy(hnd.ir);
	playerc_power_unsubscribe(hnd.power);
	playerc_power_destroy(hnd.power);
#endif
	playerc_client_disconnect(hnd.client);
	playerc_client_destroy(hnd.client);
	
	return 0;
}
