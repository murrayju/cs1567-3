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
    return sqrt( x*x + y*y);
}

double PID(pidData_t * data) {
    double pTerm, dTerm, iTerm;

    double error = data->errorHist[data->iErr];

    pTerm = data->Kp * error;

    if(data->doDiff) {
        dTerm = data->Kd * (error - prevError(data));
    } else {
        dTerm = 0;
        data->doDiff = 1;
    }

    iTerm = data->Ki * errorSum(data);

    return (pTerm + dTerm + iTerm);
}

double tranError(double cX, double cY, pidData_t * data, double tX, double tY) {
    double Xrem, Yrem;
    data->iErr = (data->iErr + 1) % NUMERR;

    //Find the remaining distance from target
    Xrem = tX - cX;
    Yrem = tY - cY;

    return data->errorHist[data->iErr] = dist(Xrem,Yrem);
}

//return the difference between A1 and A2, relative to A1
//A1 is the goal, A2 is current
double angleDiff(double A1, double A2) {
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
        return 1;
    } else {
        return storeBump = (dev->c->bumper_left || dev->c->bumper_right);
    }
}

double angleMultiplier(double v) {
    if(fabs(v) > 1.25) {
        return 1.25;
    } else if(fabs(v) < .5) {
        return .5;
    } else {
        return fabs(v);
    }
}

double hall_center_err(double left, double right, int * walls, pidData_t * data) {
    /*Negative error means too far to the right and positive means to far left*/
    double error;
    data->iErr = (data->iErr + 1) % NUMERR;

    if(right <= HALL_VAR && left <= HALL_VAR) { //both sonar can see a wall
        error = right - left;
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
    return (data->errorHist[data->iErr] = -error);
}

int checkInFront(api_HANDLES_t * dev, FilterHandles_t * filter) {
    /*
     *  This function takes the ir handle and filter, updates the ir filter data
     *  using the handle and then checkes to see if the updated value is less
     *  than the set variance.  If it is it returns a 1 indicating something is
     *  in front of it, and it needs to stop and wait
     */
    turret_get_ir(dev->t);

    if(nextSample(filter->ir, dev->t->ir[1]) <= IR_VAR) {
        return 1;
    }
    return 0;
}

pidData_t * initializePID(int type) {
    // This function initializes a pidData struct

    int i;
    static double trans_c[] = TRANS_PID_C;
    static double angle_c[] = ANGLE_PID_C;
    static double sonar_c[] = SONAR_PID_C;
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
    }
    return p;
}

void correctOdomErr(api_HANDLES_t * dev, double sonarErr, double tX, double tY, int walls, double * adjX, double * adjY) {
    //Determine current heading
    double a = dev->c->oa;
    double odoErr;
    static double devErrX = 0.0, devErrY = 0.0;
    if(a > -PI/4.0 && a <= PI/4.0) {
        //Heading NORTH
        odoErr = dev->c->oy - tY;
        if(walls == WALLS_BOTH && abs(odoErr) < 6.0) {
            devErrY = (odoErr + sonarErr/100.0);
        }
        *adjY = dev->c->oy - devErrY;
        *adjX = dev->c->ox - devErrX;
        //printf("Heading north: odoErr: %f devErr: %f sonarErr: %f tX: %f tY: %f Y: %f adjY: %f\n",odoErr,devErrY,sonarErr/100.0,tX,tY,dev->c->oy,adjY);
    } else if(a > PI/4.0 && a <= PI*0.75) {
        //Heading WEST
        odoErr = dev->c->ox - tX;
        if(walls == WALLS_BOTH && abs(odoErr) < 6.0) {
            devErrX = (odoErr - sonarErr/100.0);
        }
        *adjY = dev->c->oy - devErrY;
        *adjX = dev->c->ox - devErrX;

    } else if(a > -PI*0.75 && PI <= -PI/4.0) {
        //Heading EAST
        odoErr = dev->c->ox - tX;
        if(walls == WALLS_BOTH && abs(odoErr) < 6.0) {
            devErrX = (odoErr + sonarErr/100.0);
        }
        *adjY = dev->c->oy - devErrY;
        *adjX = dev->c->ox - devErrX;

    } else {
        //Heading SOUTH
        odoErr = dev->c->oy - tY;
        if(walls == WALLS_BOTH && abs(odoErr) < 6.0) {
            devErrY = (odoErr - sonarErr/100.0);
        }
        *adjY = dev->c->oy - devErrY;
        *adjX = dev->c->ox - devErrX;
    }
}

double Move(api_HANDLES_t * dev, FilterHandles_t * filter, pidHandles_t * pids, double X, double Y) {
    double tError, rError, hError;
    double vX, vA;
    double adjX, adjY;
    double sonarL, sonarR;
    int walls;
    int arrived = 0;

    create_get_sensors(dev->c, TIMEOUT);
    filterSonar(dev,filter,&sonarL,&sonarR);

    while(!bumped(dev) && !arrived) {
        //Make sure the path is clear ahead
        if(0 && checkInFront(dev, filter)) {
            create_set_speeds(dev->c, 0, 0);
            #ifdef DEBUG
            printf("Obstruction detected by IR");
            #endif
            while(!bumped(dev) && checkInFront(dev, filter)) {
                #ifdef DEBUG
                printf(".");
                fflush(stdout);
                #endif
                usleep(100000);  //sleep long enough for the sensors to refresh
            }
            #ifdef DEBUG
            printf("\n");
            #endif
        }
        hError = hall_center_err(sonarL,sonarR,&walls,pids->sonar);
        correctOdomErr(dev, hError, X, Y, walls, &adjX, &adjY);
        tError = tranError(adjX, adjY, pids->trans, X, Y);
        rError = targetRotError(adjX, adjY, dev->c->oa,pids->angle, X, Y);
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
            vA = PID(pids->angle);
        } else {
            //Set angular velocity based on wall data
            vA = /*PID(pids->angle) + */PID(pids->sonar);
        }
        vX = PID(pids->trans);
        vA *= angleMultiplier(vX);

        create_set_speeds(dev->c, vX, vA);       //set new velocities
        #ifdef DEBUG
        printf("VX: %2.3f  VA: %2.3f  Te: %2.3f  Re: %2.3f  pos: %2.3f %2.3f %2.3f  adj: %2.3f %2.3f  sonar: %3.3f %3.3f He: %2.3f  walls: %d\n", vX, vA, tError, rError, dev->c->ox, dev->c->oy, dev->c->oa, adjX, adjY, sonarL, sonarR, hError, walls);
        #endif

        usleep(100000);  //sleep long enough for the sensors to refresh
        create_get_sensors(dev->c, TIMEOUT);     //update odometer sensor data
        filterSonar(dev,filter,&sonarL,&sonarR); //get new filtered data from sonar
        if(abs(tError) <= pids->trans->tol) { arrived = 1; } //Check if we made it yet
    }

    create_get_sensors(dev->c, TIMEOUT);//update position from sensors

    return tranError(adjX, adjY, pids->trans, X, Y);
}
