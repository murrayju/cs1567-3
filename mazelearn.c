//Justin Murray
//Dean Pantages
//Main Maze Solver Executable

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include "types.h"

#define FULLCONTROL 1

#include "PIDlib.h"
#include "FIRlib.h"
#include "MAZElib.h"

//Global handles so they can be used by sighandle
api_HANDLES_t * hands;
FilterHandles_t * filt;
pidHandles_t * pids;
extern roboPos_t * posData;

void cleanup() {
    create_set_speeds(hands->c, 0, 0); //Stop moving
    create_close(hands->c);
    turret_close(hands->t);
    free(filt->sonar0);
    free(filt->sonar1);
    free(filt->ir0);
    free(filt->ir1);
    free(pids->trans);
    free(pids->sonar);
    free(pids->angle);
    free(pids->angleT);
    free(posData);
    posData = NULL;
    free(hands);
    hands = NULL;
    free(filt);
    file = NULL;
    free(pids);
    pids = NULL;
}

void sighandle(int sig) {
    //stop the robot when program dies
    printf("Signal caught (%d), shutting down...\n",sig);
    cleanup();
    exit(-2);
}

int main(int argc, const char **argv) {
    int i, a, num, rnum;
    double t;
    int i2c_fd;
    pthread_t thread;
    double N,S,E,W;
    int wdis;
    mazeNode nodes[MAX_NODES];

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
    if((pids = malloc(sizeof(pidHandles_t))) == NULL) {
        fprintf(stderr,"Error calling malloc\n");
        return -1;
    }
    if((posData = malloc(sizeof(roboPos_t))) == NULL) {
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
    filt->sonar0 = initializeFilter(FILT_SONAR);
    filt->sonar1 = initializeFilter(FILT_SONAR);
    filt->ir0 = initializeFilter(FILT_IR);
    filt->ir1 = initializeFilter(FILT_IR);

    //Initialize PID structs
    pids->trans = initializePID(TRANS_PID);
    pids->angle = initializePID(ANGLE_PID);
    pids->sonar = initializePID(SONAR_PID);
    pids->angleT = initializePID(ANGLET_PID);

    //Initialize node list
    memset(nodes,0,MAX_NODES*sizeof(mazeNode));
    for(i=0; i<MAX_NODES; i++) {
        nodes[i].probN = PROB_UNINIT;
        nodes[i].probS = PROB_UNINIT;
        nodes[i].probE = PROB_UNINIT;
        nodes[i].probW = PROB_UNINIT;
    }

    // process command line args
    i = 1;
    while( i < argc ) {

        if(!strcmp(argv[i], "-w")) {
            /*
            //Accept command line waypoints
            if(argc >= i+1) {
                numWaypts = atoi(argv[++i]);
                if(argc >= i+(2*numWaypts)) {
                    free(waypoints);
                    if((waypoints = malloc(sizeof(coordData_t) * numWaypts)) == NULL) {
                        fprintf(stderr,"Error calling malloc\n");
                        return -1;
                    }

                    for(a=0; a<numWaypts; a++) {
                        waypoints[a].X = atoi(argv[++i]);
                        waypoints[a].Y = atoi(argv[++i]);
                    }
                    i++;
                } else {
                    printf("Error using [-w].\n\tUsage: -w numPoints X1 Y1 X2 Y2...\n");
                }
            } else {
                printf("Error using [-w].\n\tUsage: -w numPoints X1 Y1 X2 Y2...\n");
            }
            */
        } else {
            printf("Unrecognized option [%s]\n",argv[i]);
            return -1;
        }
    }

    //Create thread to monitor robot movement
    pthread_create(&thread, NULL, mapRobot, (void *)hands);

    // robot is ready
#ifdef DEBUG
    printf("All connections established, ready to go...\n");
#endif
    //Rotate the Servo
    turret_SetServo(hands->t, T_ANGLE);

    //Fill the sensors with data before we start
    for(i=0; i<FILT_IR_SAMPLES; i++) {
        filterIR(hands, filt, &t, &t);
#ifndef USE_IR_MODE
        filterSonar(hands, filt, &t, &t);
#endif
    }

    i=0;
    //Solve the maze
    while((wdis = What_Do_I_See(hands, filt, &N, &S, &E, &W)) != GOAL) {
        nodes[i].walls = wdis;
        num = COUNT_WALLS(nodes[i].walls);

        //Allocate fair probs if this is the first time
        if(!nodes[i].probset) {
            if(!HAS_WEST_WALL(nodes[i].walls)) {
                nodes[i].probW = 100 / num;
            }
            if(!HAS_EAST_WALL(nodes[i].walls)) {
                nodes[i].probE = 100 / num;
            }
            if(!HAS_NORTH_WALL(nodes[i].walls)) {
                nodes[i].probN = 100 / num;
            }
            if(!HAS_SOUTH_WALL(nodes[i].walls)) {
                nodes[i].probS = 100 / num;
            }
        }


    }

    // Shutdown and tidy up.  Close all of the proxy handles
    cleanup();
    return 0;
}

