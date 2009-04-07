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

roboPos_t * posData;
double ox=0,oy=0,oa=0;

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
    return (1 - dev->c->charge/dev->c->capacity)*factor;
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

void correctOdomErr(double sonarErr, int walls, double tX, double tY, double * adjX, double * adjY) {
    //Determine current heading
    double odoErr;
    static double devErrX = 0.0, devErrY = 0.0;
    if(fabs(angleDiff(NORTH, oa)) < STRAIGHT_TOL) {
        //Heading NORTH
        odoErr = oy - tY;
#ifdef USE_IR_MODE
        if(walls != WALLS_NONE) {
#else
        if(walls == WALLS_BOTH) {
#endif
            devErrY = (odoErr + sonarErr/100.0);
        }
        *adjY = oy - devErrY;
        *adjX = ox - devErrX;
    } else if(fabs(angleDiff(WEST, oa)) < STRAIGHT_TOL) {
        //Heading WEST
        odoErr = ox - tX;
#ifdef USE_IR_MODE
        if(walls != WALLS_NONE) {
#else
        if(walls == WALLS_BOTH) {
#endif
            devErrX = (odoErr - sonarErr/100.0);
        }
        *adjY = oy - devErrY;
        *adjX = ox - devErrX;

    } else if(fabs(angleDiff(EAST, oa)) < STRAIGHT_TOL) {
        //Heading EAST
        odoErr = ox - tX;
#ifdef USE_IR_MODE
        if(walls != WALLS_NONE) {
#else
        if(walls == WALLS_BOTH) {
#endif
            devErrX = (odoErr + sonarErr/100.0);
        }
        *adjY = oy - devErrY;
        *adjX = ox - devErrX;

    } else if(fabs(angleDiff(SOUTH, oa)) < STRAIGHT_TOL) {
        //Heading SOUTH
        odoErr = oy - tY;
#ifdef USE_IR_MODE
        if(walls != WALLS_NONE) {
#else
        if(walls == WALLS_BOTH) {
#endif
            devErrY = (odoErr - sonarErr/100.0);
        }
        *adjY = oy - devErrY;
        *adjX = ox - devErrX;
    } else {
        //Turning, no adjustment
        *adjY = oy - devErrY;
        *adjX = ox - devErrX;
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
    if(fabs(angleDiff(NORTH, (oa+rError))) <= PI/4.0) {
        h = NORTH;
    } else if(fabs(angleDiff(WEST, (oa+rError))) <= PI/4.0) {
        h = WEST;
    } else if(fabs(angleDiff(EAST, (oa+rError))) <= PI/4.0) {
        h = EAST;
    } else { // if(fabs(angleDiff(SOUTH, oa)) < PI/4.0) {
        h = SOUTH;
    }
    vA = 0.35 * sign(angleDiff(h,oa));
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
    oa = h;
}

//This is the thread that runs to sample and filter robot odometry data
//Uses global vars ox, oy, oa
void * mapRobot(void * api) {
    api_HANDLES_t * dev = (api_HANDLES_t *)api;
    FilterData_t * distFilt = initializeFilter(FILT_ODO);
    FilterData_t * angleFilt = initializeFilter(FILT_ODO);
    double dist, angle;
    while(posData != NULL) {
        filterOdometry(dev, distFilt, angleFilt, &dist, &angle);
        oa = NORMALIZE(oa + angle);
        ox += dist * cos(oa);
        oy += dist * sin(oa);
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
        correctOdomErr(hError, walls, X, Y, &adjX, &adjY);
        tError = tranError(adjX, adjY, pids->trans, X, Y);
        rError = targetRotError(adjX, adjY, oa,pids->angle, X, Y);
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
            vA = (0.2*PID(pids->angle) + 0.8*PID(pids->sonar))*scaleByCharge(dev,BATT_FACT);
            if(wallL < SIDE_DIST || wallR < SIDE_DIST) {
                //We are very close to the wall, take preventative measures
                vX = SLOW_VX;
                //Take this opportunity to correct the oa value
                //But only do it once
                if(!closeToWall) {
                    closeToWall = 1;
                    correctAngleErr(dev, filter, walls, rError);
                    correctOdomErr(hError, walls, X, Y, &adjX, &adjY);
                }
            } else {
                closeToWall = 0;
                vX = PID(pids->trans);
            }
        }

        create_set_speeds(dev->c, vX, vA);       //set new velocities
        #ifdef DEBUG
        //printf("V: %f A: %f Ch: %f Cap: %f\n", dev->c->voltage, dev->c->current, dev->c->charge, dev->c->capacity);
        printf("old: %2.3f %2.3f %2.3f\tnew: %2.3f %2.3f %2.3f\tadj: %2.3f %2.3f\tscale: %2.3f\n", dev->c->ox, dev->c->oy, dev->c->oa, ox, oy, oa, adjX, adjY, scaleByCharge(dev,BATT_FACT));
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
    double tError, rError, hError;
    double vX=0, vA;
    int arrived = 0;

    while(!bumped(dev) && !arrived) {
        rError = rotError(oa,pids->angle, A);

        vA = PID(pids->angle)*scaleByCharge(dev,BATT_FACT);
        vX = 0;

        create_set_speeds(dev->c, vX, vA);       //set new velocities
        #ifdef DEBUG
            printf("turn old: %2.3f\tnew: %2.3f\tscale: %2.3f\tVX: %2.3f\tVA: %2.3f\tRe: %2.3f\n", dev->c->oa, oa, scaleByCharge(dev,BATT_FACT),vX, vA, rError);
        #endif

        usleep(LOOP_SLEEP);  //sleep long enough for the sensors to refresh
        if(fabs(rError) <= pids->angle->tol) { arrived = 1; } //Check if we made it yet
    }

    return rotError(oa,pids->angle, A);
}
