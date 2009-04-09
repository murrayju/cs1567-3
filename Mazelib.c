#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "types.h"
#include "MAZElib.h"
#include "PIDlib.h"

int Move_To_Next(api_HANDLES_t * dev, FilterHandles_t * filt, pidHandles_t * pid, int direction) {
    /*
     *  This function handles moving the robot from one cell to any of the four surrounding it.
     */
    
    double angle;
    double newX, newY;
    
    //determines the angle for the robot
    
    if(direction == SOUTH_BIT){         //goal is SOUTH
        angle = SOUTH;
        newX = (dev->ox - CELL_DIST);
        newY = dev->oy;
#ifdef DEBUG
        printf("Rotating robot to face SOUTH\n");
#endif
    } else if(direction == EAST_BIT){     //goal is EAST
        angle = EAST;
        newX = dev->ox;
        newY = (dev->oy - CELL_DIST);
#ifdef DEBUG
        printf("Rotating robot to face EAST\n");
#endif
    } else if(direction == NORTH_BIT){    //goal is NORTH
        angle = NORTH;
        newX = (dev->ox + CELL_DIST);
        newY = dev->oy;
#ifdef DEBUG
        printf("Rotating robot to face NORTH\n");
#endif
    } else if(direction == WEST_BIT){     //goal is WEST
        angle = WEST;
        newX = dev->ox;
        newY = (dev->oy + CELL_DIST);
#ifdef DEBUG
        printf("Rotating robot to face WEST\n");
#endif
    } else {
        //Error
        return -1;
    }
    
    //rotates robot into correct heading
    Turn(dev,filt,pid,angle);
    
    //Fix robot orientation
    //fixOrientation(dev, filt, pid);
    
    //moves robot to next cell
    Move(dev,filt,pid,newX,newY);
    
    //Fix robot orientation
    //fixOrientation(dev, filt, pid);
    
    return 0;
}

int set_Wall_Flags(double N, double S, double E, double W) {
    //sets the wall flags
    int walls = 0;
    
    if(N <= SIDE_DIST) {
        SET_NORTH_WALL(walls);
    }
    
    if(S <= SIDE_DIST) {
        SET_SOUTH_WALL(walls);
    }

    if(E <= SIDE_DIST) {
        SET_EAST_WALL(walls);
    }
    
    if(W <= SIDE_DIST) {
        SET_WEST_WALL(walls);
    }
    printf("Set Wall Flags: %d, count: %d\n",walls,COUNT_WALLS(walls));
    return walls;
    
}

int What_Do_I_See(api_HANDLES_t * dev, FilterHandles_t * filt, double * N, double * S, double * E, double * W) {
    /*
     *  This function checks for walls on all four sides of the robot and
     *  returns a value indicating which of the 15 positions it sees
     */
     
     //determines which heading the robot has and sets the values accoringly
     if(fabs(angleDiff(NORTH, dev->oa)) <= PI/4.0) {    //facing north
        filterSonar(dev, filt, S, N);  //gets sonar readings
        filterIR(dev, filt, E, W);     //gets ir readings
     } else if(fabs(angleDiff(EAST, dev->oa)) <= PI/4.0) {    //facing east
        filterSonar(dev, filt, W, E);  //gets sonar readings
        filterIR(dev, filt, S, N);     //gets ir readings
     } else if(fabs(angleDiff(SOUTH, dev->oa)) <= PI/4.0) {   //facing south
        filterSonar(dev, filt, N, S);  //gets sonar readings
        filterIR(dev, filt, W, E);     //gets ir readings
     } else { // if(fabs(angleDiff(WEST, dev->oa)) <= PI/4.0) {    //facing west
        filterSonar(dev, filt, E, W);  //gets sonar readings
        filterIR(dev, filt, N, S);     //gets ir readings
     }
     
     printf("WhatDoISee: %f %f %f %f\n",*N,*S,*E,*W);
     
     return set_Wall_Flags(*N, *S, *E, *W);
}
