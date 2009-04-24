//Justin Murray
//Dean Pantages
//PID control functions

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "PIDlib.h"
#include "FIRlib.h"
#include "MAZElib.h"

roboPos_t * posData;

double prevError(pidData_t * data) {
    if(data->iErr == 0) {
        return data->errorHist[NUMERR - 1];
    } else {
        return data->errorHist[data->iErr - 1];
    }
}

double errorSum(pidData_t * data) {
    double sum = 0;
    int i;
    for(i=0; i<NUMERR; i++) {
        sum += data->errorHist[i];
    }

    if(sum > data->maxI) {
        return data->maxI;
    } else {
        return sum;
    }
}

double dist(double x, double y) {
    return sqrt(x*x + y*y);
}

double PID(pidData_t * data) {
    double pTerm, dTerm, iTerm;

    double error = data->errorHist[data->iErr];

    pTerm = data->Kp * error;

    //This ensures that the first PID value does not show an enormous dTerm
    if(data->doDiff) {
        dTerm = data->Kd * (error - prevError(data));
    } else {
        dTerm = 0;
        data->doDiff = 1;
    }

    iTerm = data->Ki * errorSum(data);

    return (pTerm + dTerm + iTerm);
}

//Calculates the error as the pythagorean distance from current pos to target pos
//Updates the PID stuct with the current error value
double tranError(double cX, double cY, pidData_t * data, double tX, double tY) {
    double Xrem, Yrem;
    data->iErr = (data->iErr + 1) % NUMERR;

    //Find the remaining distance from target
    Xrem = tX - cX;
    Yrem = tY - cY;

    //Add value to the PID struct and return
    return data->errorHist[data->iErr] = dist(Xrem,Yrem);
}

//return the difference between A1 and A2, relative to A1
//A1 is the goal, A2 is current
double angleDiff(double A1, double A2) {
    A1 = NORMALIZE(A1);
    A2 = NORMALIZE(A2);

    double temp;
    if(A1 >= 0 && A1 <= (PI / 2.0)) {
        if(A2 >= 0) {
            return A1 - A2;
        } else {
            temp = A1 - A2;
            if(temp > PI) {
                temp -= PI;
                return -PI + temp;
            } else {
                return temp;
            }
        }
    } else if(A1 > (PI / 2.0)) {
        if(A2 >= 0) {
            return A1 - A2;
        } else {
            temp = -((PI - A1) + (PI + A2));
            if(temp < -PI) {
                temp += PI;
                return PI + temp;
            } else {
                return temp;
            }
        }
    } else if(A1 < 0 && A1 >= (-PI / 2.0)) {
        if(A2 <= 0) {
            return A1 - A2;
        } else {
            temp = A1 - A2;
            if(temp < -PI) {
                temp += PI;
                return PI + temp;
            } else {
                return temp;
            }
        }
    } else {
        if(A2 <= 0) {
            return A1 - A2;
        } else {
            temp = (PI + A1) + (PI - A2);
            if(temp > PI) {
                temp -= PI;
                return -PI + temp;
            } else {
                return temp;
            }
        }
    }
}

double rotError(double cA, pidData_t * data, double tA) {
    data->iErr = (data->iErr + 1) % NUMERR;
    return data->errorHist[data->iErr] = angleDiff(tA,cA);
}

//Calculate the correct straight line heading from current pos to target
//The rotational error is the difference between this and the current heading
//Updates the PID struct with the current error value
double targetRotError(double cX, double cY, double cA, pidData_t * data, double tX, double tY) {
    double Xrem, Yrem;
    double phi;
    data->iErr = (data->iErr + 1) % NUMERR;

    //find the remaining distance to the target
    Xrem = tX - cX;
    Yrem = tY - cY;

    //find the correct angle from pos to target
    if(Xrem == 0.0) {
        if(Yrem >= 0.0) {
            phi = PI/2.0;
        } else {
            phi = -PI/2.0;
        }
    } else if(Xrem > 0.0) {
        phi = atan(Yrem / Xrem);
    } else {
        if(Yrem >= 0.0) {
            phi = PI + atan(Yrem / Xrem);
        } else {
            phi = atan(Yrem / Xrem) - PI;
        }
    }

    return data->errorHist[data->iErr] = angleDiff(phi, cA);
}

int bumped(api_HANDLES_t * dev) {
    static int storeBump = 0;
    if(storeBump) {
#ifdef DEBUG
        printf("Bumped!\n");
#endif
        return 1;
    } else {
        return storeBump = (dev->c->bumper_left || dev->c->bumper_right);
    }
}

double hall_center_err(double left, double right, int * walls, pidData_t * data) {
    /*Negative error means too far to the right and positive means to far left*/
    double error;
    data->iErr = (data->iErr + 1) % NUMERR;

    if(right <= HALL_VAR && left <= HALL_VAR) { //both sonar can see a wall
        error = left - right;
        *walls = WALLS_BOTH;
    } else if(right <= HALL_VAR && left > HALL_VAR) {   //right sonar can see a wall and left cannot
        error = (HALL_WIDTH/2.0) - right;
        *walls = WALLS_RIGHT_ONLY;
    } else if(left <= HALL_VAR && right > HALL_VAR) {   //left sonar can see a wall and right cannot
        error = left - (HALL_WIDTH/2.0);
        *walls = WALLS_LEFT_ONLY;
    } else {
        error = 0.0;    // Stay in center
        *walls = WALLS_NONE;
    }
    return (data->errorHist[data->iErr] = error);
}

//Updates the readings from whichever sensor is facing forward
//Looks for an obstruction in the front and returns 1 if detected
int checkInFront(api_HANDLES_t * dev, FilterHandles_t * filter) {
    double front,back;
#ifdef USE_IR_MODE
    return 0; //Sonar does not work on pris
    //filterSonar(dev, filter, &back, &front);
#else
    filterIR(dev, filter, &back, &front);
#endif
    if(front <= FRONT_DIST) {
        return 1;
    }
    return 0;
}

//Used as a multiplier for velocity based on battery charge
//Determines the charge percentage and multiplies by a given factor
double scaleByCharge(api_HANDLES_t * dev, double factor) {
    return 1;//(1 - dev->c->charge/dev->c->capacity)*factor;
}

pidData_t * initializePID(int type) {
    // This function initializes a pidData struct

    int i;
    static double trans_c[] = TRANS_PID_C;
    static double angle_c[] = ANGLE_PID_C;
    static double sonar_c[] = SONAR_PID_C;
    static double angleT_c[] = ANGLET_PID_C;
    pidData_t * p = malloc(sizeof(pidData_t));

    if(p == NULL) { return NULL; }

    // Initialize everything to zero
    memset(p,0,sizeof(pidData_t));

    // Copy the #defined coeffs into the struct
    if(type == TRANS_PID) {
        memcpy(p, trans_c, NUM_PID_C * sizeof(double));
    } else if(type == ANGLE_PID) {
        memcpy(p, angle_c, NUM_PID_C * sizeof(double));
    } else if(type == SONAR_PID) {
        memcpy(p, sonar_c, NUM_PID_C * sizeof(double));
    } else if(type == ANGLET_PID) {
        memcpy(p, angleT_c, NUM_PID_C * sizeof(double));
    }
    return p;
}

void correctOdomErr(api_HANDLES_t * dev, double sonarErr, int walls, double tX, double tY, double * adjX, double * adjY) {
    //Determine current heading
    double odoErr;
    static double devErrX = 0.0, devErrY = 0.0;
    if(fabs(angleDiff(NORTH, dev->oa)) < STRAIGHT_TOL) {
        //Heading NORTH
        odoErr = dev->oy - tY;
#ifdef USE_IR_MODE
        if(walls != WALLS_NONE) {
#else
        if(walls == WALLS_BOTH) {
#endif
            devErrY = (odoErr + sonarErr/100.0);
        }
        *adjY = dev->oy - devErrY;
        *adjX = dev->ox - devErrX;
    } else if(fabs(angleDiff(WEST, dev->oa)) < STRAIGHT_TOL) {
        //Heading WEST
        odoErr = dev->ox - tX;
#ifdef USE_IR_MODE
        if(walls != WALLS_NONE) {
#else
        if(walls == WALLS_BOTH) {
#endif
            devErrX = (odoErr - sonarErr/100.0);
        }
        *adjY = dev->oy - devErrY;
        *adjX = dev->ox - devErrX;

    } else if(fabs(angleDiff(EAST, dev->oa)) < STRAIGHT_TOL) {
        //Heading EAST
        odoErr = dev->ox - tX;
#ifdef USE_IR_MODE
        if(walls != WALLS_NONE) {
#else
        if(walls == WALLS_BOTH) {
#endif
            devErrX = (odoErr + sonarErr/100.0);
        }
        *adjY = dev->oy - devErrY;
        *adjX = dev->ox - devErrX;

    } else if(fabs(angleDiff(SOUTH, dev->oa)) < STRAIGHT_TOL) {
        //Heading SOUTH
        odoErr = dev->oy - tY;
#ifdef USE_IR_MODE
        if(walls != WALLS_NONE) {
#else
        if(walls == WALLS_BOTH) {
#endif
            devErrY = (odoErr - sonarErr/100.0);
        }
        *adjY = dev->oy - devErrY;
        *adjX = dev->ox - devErrX;
    } else {
        //Turning, no adjustment
        *adjY = dev->oy - devErrY;
        *adjX = dev->ox - devErrX;
    }
}

//Returns the distance to one wall (the only wall seen)
double oneWall(api_HANDLES_t * dev, FilterHandles_t * filter, double walls) {
    double wallL, wallR;

#ifdef USE_IR_MODE
    filterIR(dev, filter, &wallR, &wallL);
#else
    filterSonar(dev, filter, &wallL, &wallR);
#endif

    if(walls == WALLS_LEFT_ONLY) {
        return wallL;
    } else {
        return wallR;
    }
}

int sign(double d) {
    if(d >= 0) {
        return 1;
    }
    return -1;
}

void correctAngleErr(api_HANDLES_t * dev, FilterHandles_t * filter, double walls, double rError) {

    double h, err, val, prev, vA, min;
    double tol = 1.1;

    create_set_speeds(dev->c, 0.0, 0.0); //Stop moving
    //Determine the desired cardinal direction
    if(fabs(angleDiff(NORTH, (dev->oa+rError))) <= PI/4.0) {
        h = NORTH;
    } else if(fabs(angleDiff(WEST, (dev->oa+rError))) <= PI/4.0) {
        h = WEST;
    } else if(fabs(angleDiff(EAST, (dev->oa+rError))) <= PI/4.0) {
        h = EAST;
    } else { // if(fabs(angleDiff(SOUTH, dev->oa)) < PI/4.0) {
        h = SOUTH;
    }
    vA = 0.35 * sign(angleDiff(h,dev->oa));
    usleep(500000);
    val = min = oneWall(dev,filter,walls);
    create_set_speeds(dev->c, 0.0, vA);
    do {
        usleep(500000);
        prev = val;
        val = oneWall(dev,filter,walls);
        if(val < min) {
            min = val;
        }
        printf("prev: %f val: %f min: %f vA: %f\n",prev,val,min,vA); 
    } while(val <= prev);
    vA *= -0.75; //Slow down and change direction
    create_set_speeds(dev->c, 0.0, vA);
    printf("Change direction!\n");
    usleep(500000);
    do {
        usleep(500000);
        prev = val;
        val = oneWall(dev,filter,walls);
        if(val < min) {
            min = val;
        }
        printf("prev: %f val: %f min: %f vA: %f\n",prev,val,min,vA); 
    } while(val <= prev);
    //create_set_speeds(dev->c, 0.0, 0.0);
    printf("Est min: %f\n", min);
    
    vA *= -0.95; //Slow down and change direction
    create_set_speeds(dev->c, 0.0, vA);
    while((err = (val-min)) > tol) {
        usleep(500000);
        prev = val;
        val = oneWall(dev,filter,walls);
        if(val < min) {
            min = val;
        }
        if(val > prev) {
            vA *= -0.95; //Slow down and change direction
            create_set_speeds(dev->c, 0.0, vA);
            printf("Change direction!\n");
        }
        printf("prev: %f val: %f min: %f vA: %f\n",prev,val,min,vA); 
    }
    printf("I think I am oriented at dist: %f\n", val);
    
    //Set the corrected angle
    dev->oa = h;
}

//This is the thread that runs to sample and filter robot odometry data
//Uses api struct vars ox, oy, oa
void * mapRobot(void * api) {
    api_HANDLES_t * dev = (api_HANDLES_t *)api;
    FilterData_t * distFilt = initializeFilter(FILT_ODO);
    FilterData_t * angleFilt = initializeFilter(FILT_ODO);
    double dist, angle;
    
    dev->ox = 0;
    dev->oy = 0;
    dev->oa = 0;
    while(posData != NULL) {
        filterOdometry(dev, distFilt, angleFilt, &dist, &angle);
        #ifdef FILTER_ODO
        dev->oa = NORMALIZE(dev->oa + angle);
        dev->ox += dist * cos(dev->oa);
        dev->oy += dist * sin(dev->oa);
        #else
        dev->oa = NORMALIZE(dev->oa + dev->c->angle);
        dev->ox += dev->c->dist * cos(dev->oa);
        dev->oy += dev->c->dist * sin(dev->oa);
        #endif
        usleep(ODO_SLEEP);
    }
}

double Move(api_HANDLES_t * dev, FilterHandles_t * filter, pidHandles_t * pids, double X, double Y) {
    double tError, rError, hError;
    double vX, vA;
    double adjX, adjY;
    double wallL, wallR;
    int walls;
    int arrived = 0;
    int closeToWall = 0;

#ifdef USE_IR_MODE
    filterIR(dev, filter, &wallR, &wallL);
#else
    filterSonar(dev, filter, &wallL, &wallR);
#endif

    while(!bumped(dev) && !arrived) {
        //Make sure the path is clear ahead
        if(checkInFront(dev, filter)) {
            create_set_speeds(dev->c, 0, 0);
            #ifdef DEBUG
            printf("Obstruction detected by IR");
            #endif
            while(!bumped(dev) && checkInFront(dev, filter)) {
                #ifdef DEBUG
                printf(".");
                fflush(stdout);
                #endif
                usleep(LOOP_SLEEP);  //sleep long enough for the sensors to refresh
            }
            #ifdef DEBUG
            printf("\n");
            #endif
            if(bumped(dev)) { break; }
        }
        hError = hall_center_err(wallL,wallR,&walls,pids->sonar);
        correctOdomErr(dev, hError, WALLS_NONE, X, Y, &adjX, &adjY);
        tError = tranError(adjX, adjY, pids->trans, X, Y);
        rError = targetRotError(adjX, adjY, dev->oa,pids->angle, X, Y);
        if(rError > 0.75*PI || rError < -0.75*PI) {
            //We passed it up, error should be negative
            tError = -tError;
            pids->trans->errorHist[pids->trans->iErr] = tError;
            if(rError < 0) {
                rError += PI;
            } else {
                rError -= PI;
            }
            pids->angle->errorHist[pids->angle->iErr] = rError;
            //printf("Backwards! T: %f  R: %f\n",tError,rError);
        }
        if(walls == WALLS_NONE) {
            //We must rely on the wheel encoders
            vA = PID(pids->angle)*scaleByCharge(dev,BATT_FACT);
            vX = PID(pids->trans);
        } else {
            //Set angular velocity based on wall data
            vA = (0.3*PID(pids->angle) + 0.7*PID(pids->sonar))*scaleByCharge(dev,BATT_FACT);
            if(wallL < SIDE_DIST || wallR < SIDE_DIST) {
                //We are very close to the wall, take preventative measures
                vX = SLOW_VX * 1.5;
            } else {
                closeToWall = 0;
                vX = PID(pids->trans) * 1.2;
            }
        }

        create_set_speeds(dev->c, vX, vA);       //set new velocities
        #ifdef DEBUG
        //printf("V: %f A: %f Ch: %f Cap: %f\n", dev->c->voltage, dev->c->current, dev->c->charge, dev->c->capacity);
        printf("old: %2.3f %2.3f %2.3f\tnew: %2.3f %2.3f %2.3f\tadj: %2.3f %2.3f\tscale: %2.3f\n", dev->c->ox, dev->c->oy, dev->c->oa, dev->ox, dev->oy, dev->oa, adjX, adjY, scaleByCharge(dev,BATT_FACT));
        printf("VX: %2.3f  VA: %2.3f  Te: %2.3f  Re: %2.3f  wallD: %3.3f %3.3f He: %2.3f  walls: %d\n", vX, vA, tError, rError, wallL, wallR, hError, walls);
        #endif

        usleep(LOOP_SLEEP);  //sleep long enough for the sensors to refresh
        //create_get_sensors(dev->c, TIMEOUT);     //update odometer sensor data
        //get new filtered data from sonar
#ifdef USE_IR_MODE
        filterIR(dev, filter, &wallR, &wallL);
#else
        filterSonar(dev, filter, &wallL, &wallR);
#endif
        if(fabs(tError) <= pids->trans->tol) { arrived = 1; } //Check if we made it yet
    }

    return tranError(adjX, adjY, pids->trans, X, Y);
}

double Turn(api_HANDLES_t * dev, FilterHandles_t * filter, pidHandles_t * pids, double A) {
    double rError;
    double vX=0, vA;
    int arrived = 0;

    while(!bumped(dev) && !arrived) {
        rError = rotError(dev->oa,pids->angleT, A);

        vA = PID(pids->angleT)*scaleByCharge(dev,BATT_FACT);
        vX = 0;

        create_set_speeds(dev->c, vX, vA); //set new velocities
        #ifdef DEBUG
            printf("turn old: %2.3f\tnew: %2.3f\tscale: %2.3f\tVX: %2.3f\tVA: %2.3f\tRe: %2.3f\n", dev->c->oa, dev->oa, scaleByCharge(dev,BATT_FACT),vX, vA, rError);
        #endif

        usleep(LOOP_SLEEP);  //sleep long enough for the sensors to refresh
        if(fabs(rError) <= pids->angleT->tol) { arrived = 1; } //Check if we made it yet
    }

    return rotError(dev->oa,pids->angleT, A);
}

struct minInfo {
    double val;
    double prev;
    double min;
    int inc;
    int atMin;
};

//Turn the robot to be square with the walls, correct the odometer heading
void fixOrientation(api_HANDLES_t * dev, FilterHandles_t * filter, pidHandles_t * pids) {
    double h, vA, vote;
    double tol = 1.1;
    struct minInfo dir[4];
    int walls, i, num;

    create_set_speeds(dev->c, 0.0, 0.0); //Stop moving
    
    if(bumped(dev)) { return; }
    
    //Determine the desired cardinal direction
    if(fabs(angleDiff(NORTH, (dev->oa))) <= PI/4.0) {
        h = NORTH;
    } else if(fabs(angleDiff(WEST, (dev->oa))) <= PI/4.0) {
        h = WEST;
    } else if(fabs(angleDiff(EAST, (dev->oa))) <= PI/4.0) {
        h = EAST;
    } else { // if(fabs(angleDiff(SOUTH, dev->oa)) < PI/4.0) {
        h = SOUTH;
    }
    
    //initialize values
    walls = What_Do_I_See(dev, filter, &(dir[0].val), &(dir[1].val), &(dir[2].val), &(dir[3].val));
    if(walls == 0) { return; } //No data to use...
    for(i=0; i<4; i++) {
        dir[i].min = dir[i].val;
        dir[i].atMin = 0;
        dir[i].inc = 0;
    }
    
    vA = 0.20 * sign(angleDiff(h,dev->oa));
    usleep(500000);
    
    do {
        create_set_speeds(dev->c, 0.0, vA);
        do {
            usleep(500000);
            //Update vals in struct
            for(i=0; i<4; i++) {
                dir[i].prev = dir[i].val;
            }
            
            //Look for new mins
            walls = What_Do_I_See(dev, filter, &(dir[0].val), &(dir[1].val), &(dir[2].val), &(dir[3].val));
            if(walls == 0) { return; } //No data to use...
            for(i=0; i<4; i++) {
                if(dir[i].val <= dir[i].min) {
                    dir[i].min = dir[i].val;
                    dir[i].atMin = 1;
                } else {
                    if((dir[i].val - dir[i].min) <= tol) {
                        dir[i].atMin = 1;
                    } else {
                        dir[i].atMin = 0;
                    }
                }
            }
            
            //Determine if we should break loop
            for(i=0; i<4; i++) {
                if(dir[i].val > dir[i].prev) {
                    dir[i].inc = 1;
                }
            }
            if(!HAS_NORTH_WALL(walls)) {
                dir[0].inc = 0;
                dir[0].atMin = 0;
            }
            if(!HAS_SOUTH_WALL(walls)) {
                dir[1].inc = 0;
                dir[1].atMin = 0;
            }
            if(!HAS_EAST_WALL(walls)) {
                dir[2].inc = 0;
                dir[2].atMin = 0;
            }
            if(!HAS_WEST_WALL(walls)) {
                dir[3].inc = 0;
                dir[3].atMin = 0;
            }
            
            //Count the sensors that increased
            num=0;
            for(i=0; i<4; i++) {
                num += dir[i].inc;
            }
            vote = (double)num/(double)COUNT_WALLS(walls);
            
            #ifdef DEBUG
            for(i=0; i<4; i++) {
                printf("s: %d  val: %f  prev: %f  min: %f  inc: %d  atMin: %d\n",i,dir[i].val,dir[i].prev,dir[i].min,dir[i].inc,dir[i].atMin);
            }
            printf("Vote: %f  num: %d  walls: %d\n\n",vote,num,COUNT_WALLS(walls));
            #endif
        } while(!bumped(dev) && vote < 0.6);
        vA *= -0.95; //Slow down and change direction
        printf("Change direction!\n");
        
        //See if we are within tolerance of the min
        num=0;
        for(i=0; i<4; i++) {
            num += dir[i].atMin;
        }
        vote = (double)num/(double)COUNT_WALLS(walls);
        #ifdef DEBUG
        printf("AtMin Vote: %f  num: %d  walls: %d\n\n",vote,num,COUNT_WALLS(walls));
        #endif
    } while(!bumped(dev) && vote < 0.6);
    create_set_speeds(dev->c, 0.0, 0.0); //Stop
    
    //Set the corrected angle
    dev->oa = h;
}


void centerFrontBack(api_HANDLES_t * dev, FilterHandles_t * filter, pidHandles_t * pids){
    /*  This function will be used to center the robot in the cell.  
    *   This function is simple and will only center front to back because
    *   we can then turn the robot 90 degrees and call the same function again
    */
    
    double front, back, difference, heading;
    char wall_front = wall_back = wall_both = 0;
    
    //sonar0 is in back sonar1 is the front
    //CELL_VAR = 80.0
    //HALFCELL_VAR = 40.0
    
    //polls sonar
    filterSonar(dev, filter, &back, &front);
#ifdef DEBUG
    printf("Distance in front is: %d\n", front);
    printf("Distance in back is: %d\n", back);
#endif

    if(front < CELL_VAR && front > 0.0) //sees a wall in front
        wall_front = 1;
    if(back < CELL_VAR && back > 0.0)   //sees a wall behind
        wall_back = 1;
    if(wall_front && wall_back)
        wall_both = 1;
        
    if(wall_both){  //calculates difference based on two walls

        difference = front - back;
#ifdef DEBUG
        printf("The robot sees a front and a back wall\n");
        printf("The difference between the front and back is: %d\n",difference);
        if(difference > 0.0)
            printf("The robot needs to move forward by: %d\n", difference/2.0);
        if(difference < 0.0)
            printf("The robot needs to move backward by: %d\n", difference/2.0);
#endif

        //checks the robots heading and sets coordinates based on heading
        if(fabs(angleDiff(NORTH, dev->oa)) <= PI/4.0) {    //facing NORTH
            Move(dev,filter,pids, (dev->ox+(difference/2.0)), dev->oy);
        }
        else if(fabs(angleDiff(EAST, dev->oa)) <= PI/4.0) { //facing EAST
            Move(dev,filter,pids, dev->ox, (dev->oy - (difference/2.0)));
        }
        else if(fabs(angleDiff(SOUTH, dev->oa)) <= PI/4.0) {    //facing SOUTH
            Move(dev,filter,pids, (dev->ox - (difference/2.0)), dev->oy);
        }
        else {                                              //facing WEST
            Move(dev,filter,pids, dev->ox, (dev->oy + (difference/2.0)));
        }
    }
    else if(wall_front){    //calculates difference based on front wall only
        
        difference = front - HALFCELL_VAR;
#ifdef DEBUG
        printf("The robot sees the front wall only\n");
        printf("The difference between the front and center is: %d\n", difference);
        if(difference > 0.0)
            printf("The robot needs to move forward by: %d\n",difference);
        if(difference < 0.0)
            printf("The robot needs to move backward by: %d\n",difference);
#endif
        
        //checks the robots heading and sets coordinated based on heading
        if(fabs(angleDiff(NORTH, dev->oa)) <= PI/4.0) {    //facing NORTH
            Move(dev,filter,pids, (dev->ox+difference), dev->oy);
        }
        else if(fabs(angleDiff(EAST, dev->oa)) <= PI/4.0) { //facing EAST
            Move(dev,filter,pids, dev->ox, (dev->oy - difference));
        }
        else if(fabs(angleDiff(SOUTH, dev->oa)) <= PI/4.0) {    //facing SOUTH
            Move(dev,filter,pids, (dev->ox - difference), dev->oy);
        }
        else{                                               //facing WEST
            Move(dev,filter,pids, dev->ox, (dev->oy + difference));
        }
    }
    else{               //calculates difference based on back wall only
        difference = HALFCELL_VAR - back;
#ifdef  DEBUG
        printf("The robot sees the back wall only\n");
        printf("The difference between the back and center is: %d\n", difference);
        if(difference > 0.0)
            printf("The robot needs to move forward by: %d\n",difference);
        if(difference < 0.0)
            printf("The robot needs to move backward by: %d\n",difference);
#endif
        //checks the robots heading and sets coordinated based on heading
        if(fabs(angleDiff(NORTH, dev->oa)) <= PI/4.0) {    //facing NORTH
            Move(dev,filter,pids, (dev->ox+difference), dev->oy);
        }
        else if(fabs(angleDiff(EAST, dev->oa)) <= PI/4.0) { //facing EAST
            Move(dev,filter,pids, dev->ox, (dev->oy - difference));
        }
        else if(fabs(angleDiff(SOUTH, dev->oa)) <= PI/4.0) {    //facing SOUTH
            Move(dev,filter,pids, (dev->ox - difference), dev->oy);
        }
        else{                                               //facing WEST
            Move(dev,filter,pids, dev->ox, (dev->oy + difference));
        }
    }
    
#ifdef DEBUG
    printf("Adjustment Complete, Exiting function\n");
#endif
        
}
