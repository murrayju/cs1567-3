#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "types.h"
#include "MAZElib.h"
#include "PIDlib.h"


extern double ox, oy, oa;

int Move_To_Next(api_HANDLES_t * dev, FilterHandles_t * filt, pidHandles_t * pid, int direction) {
    /*
     *  This function handles moving the robot from one cell to any of the four surrounding it.
     */
    
    double angle;
    double newX, newY;
    
    //determines the angle for the robot
    
    if(direction == SOUTH_BIT){         //goal is SOUTH
        angle = SOUTH;
        newX = (ox - CELL_DIST);
        newY = oy;
#ifdef DEBUG
        printf("Rotating robot to face SOUTH\n");
#endif
    } else if(direction == EAST_BIT){     //goal is EAST
        angle = EAST;
        newX = ox;
        newY = (oy - CELL_DIST);
#ifdef DEBUG
        printf("Rotating robot to face EAST\n");
#endif
    } else if(direction == NORTH_BIT){    //goal is NORTH
        angle = NORTH;
        newX = (ox + CELL_DIST);
        newY = oy;
#ifdef DEBUG
        printf("Rotating robot to face NORTH\n");
#endif
    } else if(direction == WEST_BIT){     //goal is WEST
        angle = WEST;
        newX = ox;
        newY = (oy + CELL_DIST);
#ifdef DEBUG
        printf("Rotating robot to face WEST\n");
#endif
    } else {
        //Error
        return -1;
    }
    
    printf("Dean turn: %f\n", angle);
    //rotates robot into correct heading
    Turn(dev,filt,pid,angle);
    
    printf("Dean move: %f %f\n", newX, newY);
    //moves robot to next cell
    Move(dev,filt,pid,newX,newY);
    
    return 0;
}
