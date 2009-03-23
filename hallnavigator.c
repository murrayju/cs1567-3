//Justin Murray
//Dean Pantages
//Main Hallway Navigator Executable

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "types.h"

#define FULLCONTROL 1

#include "PIDlib.h"
#include "FIRlib.h"

//Define the default set of waypoints
#define NUM_WAYPOINTS 9
#define WAYPOINT_ARRAY {7.3152,0.0, 7.3152,7.62, 26.21,7.62, 26.21,-3.048, 30.1752,-3.048, 30.1752,-12.192, 7.3152,-12.192, 7.3152,0, 0,0}

//Global handles so they can be used by sighandle
api_HANDLES_t * hands;
FilterHandles_t * filt;

void cleanup() {
	create_set_speeds(hands->c, 0, 0);
	create_close(hands->c);
	turret_close(hands->t);
	free(filt->sonarR);
	free(filt->sonarL);
	free(filt->ir);
	free(hands);
	free(filt);
}

void sighandle(int sig) {
	//stop the robot when program dies
	printf("Signal caught (%d), shutting down\n",sig);
	cleanup();
	exit(-2);
}

int main(int argc, const char **argv) {
	int i, a;
	int i2c_fd;
	coordData_t * waypoints;
	int numWaypts = NUM_WAYPOINTS;
	double w[] = WAYPOINT_ARRAY;
	
	//Set signals to handle
	signal(SIGINT, &sighandle);
	signal(SIGTERM, &sighandle);
	signal(SIGABRT, &sighandle);
	
	//allocate structs
	if((hands = malloc(sizeof(api_HANDLES_t))) == NULL) {
		fprintf(stderr,"Error calling malloc\n");
		return -1;
	}
	if((filt = malloc(sizeof(FilterHandles_t))) == NULL) {
		fprintf(stderr,"Error calling malloc\n");
		return -1;
	}
	if((waypoints = malloc(sizeof(coordData_t) * numWaypts)) == NULL) {
		fprintf(stderr,"Error calling malloc\n");
		return -1;
	}
	
	for(i=0; i<numWaypts; i++) {
		waypoints[i].X = w[i*2];
		waypoints[i].Y = w[i*2+1];
	}
	
	// allocate devices
	if((hands->c = create_create(COMPORT)) == NULL) {
		fprintf(stderr,"Error connecting create\n");
		return -1;
	}
	if((hands->t = turret_create()) == NULL) {
		fprintf(stderr,"Error connecting turrett\n");
		return -1;
	}
	
	//open serial com port
	if(create_open(hands->c, FULLCONTROL) < 0) {
		fprintf(stderr,"Failed to open create\n");
		return -1;
	}
	
	//open i2c device
	if(turret_open(hands->t) < 0) {
		fprintf(stderr,"Failed to connect to robostix\n");
		return -1;
	}
	
	//initialize the robostix board
	turret_init(hands->t);
	
	//Initialize Filter structs
	filt->sonarR = initializeFilter(SONAR);
	filt->sonarL = initializeFilter(SONAR);
	filt->ir = initializeFilter(IR);

	// process command line args
	i = 1;
	while( i < argc ) {
		if(0/*!strcmp(argv[i], "-s")*/) {
			//no options currently
		} else {
			printf("Unrecognized option [%s]\n",argv[i]);
			return -1;
		}
	}
	
	// robot is ready
#ifdef DEBUG
	printf("All connections established, ready to go\n");
#endif
	//Rotate the Servo
	turret_SetServo(hands->t, T_ANGLE);
	for(i=0; i<numWaypts; i++) {
		Move(hands,filt,waypoints[i].X,waypoints[i].Y);
#ifdef DEBUG
		printf("\nArrived at waypoint %d\n\n", i+1);
#endif
	}

	// Shutdown and tidy up.  Close all of the proxy handles
	cleanup();
	return 0;
}
