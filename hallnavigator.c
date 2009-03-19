//Justin Murray
//Dean Pantages
//Main Hallway Navigator Executable

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "TurretAPI.h"
#include "create_comms.h"

#define FULLCONTROL 1
#define DEBUG

#include "PIDlib.h"
#include "FIRlib.h"

#define T_ANGLE 90.0
#define COMPORT "/dev/ttyS2"

extern int bumped(api_HANDLES_t *);

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
}

int main(int argc, const char **argv) {
	int i, a;
	int i2c_fd;
	
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
	while(!bumped(hands)) {
		create_get_sensors(hands->c, TIMEOUT);
		turret_get_sonar(hands->t);
		turret_get_ir(hands->t);
		create_print(hands->c);
		printf("IR: %d   %d\tSonar: %d   %d\n",hands->t->ir[0],hands->t->ir[1],hands->t->sonar[0],hands->t->sonar[1]);
		if(hands->t->ir[0] < 35) {
			create_set_speeds(hands->c, 0, 0);
#ifdef DEBUG
			printf("Obstruction detected by IR\n");
#endif
		} else {
			create_set_speeds(hands->c, 5, 0);
		}
	}
	

	// Shutdown and tidy up.  Close all of the proxy handles
	cleanup();
	return 0;
}
