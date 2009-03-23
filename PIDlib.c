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

#define NUMERR 10

typedef struct pid_data_struct {
    double Kp, Kd, Ki;
    double errorHist[NUMERR];
    int iErr;
    double tol;
    double maxI;
    double Xi, Yi, Ai;
    int doDiff;
} pid_data;

int checkInFront(api_HANDLES_t *, FilterHandles_t *);
double hall_center_err(double, double, int *, pid_data *);

double prevError(pid_data * data) {
    if(data->iErr == 0) {
        return data->errorHist[NUMERR - 1];
    } else {
        return data->errorHist[data->iErr - 1];
    }
}

double errorSum(pid_data * data) {
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

double PID(pid_data * data) {
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

    //printf("PID P=%f D=%f I=%f total=%f\n",pTerm, dTerm, iTerm, (pTerm+dTerm+iTerm));

    return (pTerm + dTerm + iTerm);
}

double tranError(api_HANDLES_t * dev, pid_data * data, double tX, double tY) {
    double relX = dev->c->ox - data->Xi; //X distance travelled
    double relY = dev->c->oy - data->Yi; //Y distance travelled
    double Xreal, Yreal, Xrem, Yrem;
    double mew, theta, phi, error;
    data->iErr = (data->iErr + 1) % NUMERR;

    //Find the remaining distance from target
    Xrem = tX - dev->c->ox;
    Yrem = tY - dev->c->oy;
    //printf("Remaining real distance - x=%f y=%f\n",Xrem,Yrem);

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

double targetRotError(api_HANDLES_t * dev, pid_data * data, double tX, double tY) {
    double relX = dev->c->ox - data->Xi; //X distance travelled
    double relY = dev->c->oy - data->Yi; //Y distance travelled
    double Xreal, Yreal, Xrem, Yrem;
    double mew, theta, phi, error;
    data->iErr = (data->iErr + 1) % NUMERR;

    //find the remaining distance to the target
    Xrem = tX - dev->c->ox;
    Yrem = tY - dev->c->oy;

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

    error = angleDiff(phi, dev->c->oa);
    //printf("TRE: dist=(%f,%f) phi=%f err=%f\n",Xrem,Yrem,phi,error);
    return data->errorHist[data->iErr] = error;
}

double rotError(api_HANDLES_t * dev, pid_data * data, double tA) {
    double relA = dev->c->oa;
    double error;

    data->iErr = (data->iErr + 1) % NUMERR;
    error = angleDiff(tA, dev->c->oa);

    return data->errorHist[data->iErr] = error;
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

double Move(api_HANDLES_t * dev, FilterHandles_t * filter, double X, double Y) {
    double tError, rError, vX, vA;
    double sonarL, sonarR;
    double hallErr;
    int walls;
    pid_data tranData, rotData, wallData;

    //clear structs
    memset(&tranData, 0, sizeof(pid_data));
    memset(&rotData, 0, sizeof(pid_data));
    memset(&wallData, 0, sizeof(pid_data));

    //Set parameters
    tranData.Kp = 0.9;
    tranData.Kd = 1.0;
    tranData.Ki = 0.01;
    tranData.tol = 0.5; //This is the tolerance for hitting the target
    tranData.maxI = 10;

    rotData.Kp = 0.5;
    rotData.Kd = 1.0;
    rotData.Ki = .005;
    rotData.tol = 0.01;
    rotData.maxI = 10;

    wallData.Kp = 0.006;
    wallData.Kd = 0.006;
    wallData.Ki = .00001;
    wallData.tol = 0.00005;
    wallData.maxI = 0.001;

    create_get_sensors(dev->c, TIMEOUT);
    filterSonar(dev,filter,&sonarL,&sonarR);

    //Store initial position info
    tranData.Xi = dev->c->ox;
    tranData.Yi = dev->c->oy;
    tranData.Ai = dev->c->oa;

    rotData.Xi = dev->c->ox;
    rotData.Yi = dev->c->oy;
    rotData.Ai = dev->c->oa;

    while(!bumped(dev) && (tError = tranError(dev, &tranData, X, Y)) > tranData.tol) {
        //Make sure the path is clear ahead
        if(checkInFront(dev, filter)) {
            create_set_speeds(dev->c, 0, 0);
            #ifdef DEBUG
            printf("Obstruction detected by IR");
            #endif
            while(checkInFront(dev, filter) && !bumped(dev)) {
                #ifdef DEBUG
                printf(".");
                fflush(stdout);
                #endif
            }
            #ifdef DEBUG
            printf("\n");
            #endif
        } else {
            hallErr = hall_center_err(sonarL,sonarR,&walls,&wallData);
            rError = targetRotError(dev,&rotData, X, Y);
            if(rError > 0.75*PI || rError < -0.75*PI) {
                //We passed it up, error should be negative
                tError = -tError;
                tranData.errorHist[tranData.iErr] = tError;
                if(rError < 0) {
                    rError += PI;
                } else {
                    rError -= PI;
                }
                rotData.errorHist[rotData.iErr] = rError;
                //printf("Backwards! T: %f  R: %f\n",tError,rError);
            }
            if(walls == WALLS_NONE) {
                //We must rely on the wheel encoders
                vA = PID(&rotData);
            } else {
                //Set angular velocity based on wall data
                vA = PID(&rotData) + PID(&wallData);
            }
            vX = PID(&tranData);
            vA *= angleMultiplier(vX);

            create_set_speeds(dev->c, vX, vA);       //set new velocities
            #ifdef DEBUG
            printf("VX: %2.3f  VA: %2.3f  Te: %2.3f  Re: %2.3f  pos: %2.3f %2.3f %2.3f  sonar: %3.3f %3.3f He: %2.3f  walls: %d\n", vX, vA, tError, rError, dev->c->ox, dev->c->oy, dev->c->oa, sonarL, sonarR, hallErr, walls);
            #endif
        }
        usleep(100000);  //sleep long enough for the sensors to refresh
        create_get_sensors(dev->c, TIMEOUT);     //update odometer sensor data
        filterSonar(dev,filter,&sonarL,&sonarR); //get new filtered data from sonar
    }

    create_get_sensors(dev->c, TIMEOUT);//update position from sensors

    return tranError(dev, &tranData, X, Y);
}

double hall_center_err(double left, double right, int * walls, pid_data * data) {
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
