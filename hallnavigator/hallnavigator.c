//Justin Murray
//Dean Pantages
//Main Hallway Navigator Executable

#include <stdio.h>
#include <string.h>
#include <libplayerc/playerc.h>
//#define stage_environment

#include "../PIDlib/PIDlib.h"

#ifdef stage_environment
#define SERVER "localhost"
#define PORT	12121
#else
#define SERVER "gort"
#define PORT	9876
#define T_ANGLE 90.0
#endif

int main(int argc, const char **argv) {
	int i;
	playerc_HANDLES hnd;
	char server[20];
	int port;
	
	//Defaults
	strcpy(server,SERVER);
	port = PORT;

	// process command line args
	i = 1;
	while( i < argc ) {
		if(!strcmp(argv[i], "-s")) {
			if(argc < (i + 2)) {
				printf("Improper usage of -s.\nUsage: hallnavigator -s SERVER\n");
				return -1;
			}
			strcpy(server,argv[i+1]);
			i+=2;
		} else if(!strcmp(argv[i], "-p")) {
			if(argc < (i + 2)) {
				printf("Improper usage of -p.\nUsage: hallnavigator -p PORT\n");
				return -1;
			}
			port = atof(argv[i+1]);
			i+=2;
		} else {
			printf("Unrecognized option [%s]\n",argv[i]);
			return -1;
		}
	}
	
	// Create a client object and connect to the server
	hnd.client = playerc_client_create(NULL, server, port);
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
	// Enable the robots motors and rotates turret servo
	playerc_position2d_enable(hnd.pos2d, 1);
#ifndef stage_environment
	playerc_position1d_enable(hnd.pos1d, 1);
	playerc_position1d_set_cmd_pos(hnd.pos1d, T_ANGLE,0);
#endif
	
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
