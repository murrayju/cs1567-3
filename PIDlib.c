//Justin Murray
//Dean Pantages
//PID control functions

#include <stdio.h>
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

int checkInFront(api_HANDLES_t *, FilterData_t *);
double hall_center_err(api_HANDLES_t *, FilterHandles_t *, int *);

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
	
	printf("PID P=%f D=%f I=%f total=%f\n",pTerm, dTerm, iTerm, (pTerm+dTerm+iTerm));
	
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
	printf("Remaining real distance - x=%f y=%f\n",Xrem,Yrem);
	
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
	printf("TRE: dist=(%f,%f) phi=%f err=%f\n",Xrem,Yrem,phi,error);
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

double Move(api_HANDLES_t * dev, double X, double Y) {
	double tError, rError, vX, vA;
	pid_data tranData, rotData;
	
	//clear structs
	memset(&tranData, 0, sizeof(pid_data));
	memset(&rotData, 0, sizeof(pid_data));
	
	//Set parameters
	tranData.Kp = 0.9;
	tranData.Kd = 1.0;
	tranData.Ki = 0.01;
	tranData.tol = 0.01;
	tranData.maxI = 10;
	
	rotData.Kp = 2.0;
	rotData.Kd = 4.0;
	rotData.Ki = .01;
	rotData.tol = 0.005;
	rotData.maxI = 10;
	
	create_get_sensors (dev, TIMEOUT);
	
	//finish refractoring code here!!!
	
	//Store initial position info
	tranData.Xi = dev->c->ox;
	tranData.Yi = dev->c->oy;
	tranData.Ai = dev->c->oa;
	
	rotData.Xi = dev->c->ox;
	rotData.Yi = dev->c->oy;
	rotData.Ai = dev->c->oa;
	
	while(!bumped(dev) && (tError = tranError(dev, &tranData, X, Y)) > tranData.tol) {
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
		vX = PID(&tranData);
		vA = PID(&rotData) * angleMultiplier(vX);
				
		create_set_speeds (dev, vX, vA);     //set new speeds
		create_get_sensors (dev, TIMEOUT); //update position from sensors
		
		/*printf("VX: %f  VA: %f  Terror: %f  Rerror: %f  position : %f %f %f  bumpers: %d %d\n", vX, vA, tError, rError, hands->pos2d->px, hands->pos2d->py, hands->pos2d->pa, hands->bumper->bumpers[0], hands->bumper->bumpers[1]);*/
	} 
	
	create_set_speeds (dev, 0, 0);  //STOP!
	create_get_sensors (dev, TIMEOUT);//update position from sensors
	
	return tranError(dev, &tranData, X, Y);
}

double Turn(api_HANDLES_t * dev, double A) {
	double rError, vA;
	pid_data rotData;
	
	//clear struct
	memset(&rotData, 0, sizeof(pid_data));
	
	//Set parameters
	rotData.Kp = 1.1;
	rotData.Kd = 0.05;
	rotData.Ki = 0.06;
	rotData.tol = 0.005;
	rotData.maxI = 0.5;
	
	create_get_sensors (dev, TIMEOUT);
	
	//Store initial position info
	rotData.Xi = dev->c->ox;
	rotData.Yi = dev->c->oy;
	rotData.Ai = dev->c->oa;
	
	while(!bumped(dev) && fabs(rError = rotError(dev,&rotData, A)) > rotData.tol) {
		vA = PID(&rotData);
		create_set_speeds (dev, 0, vA);  //set new speeds
		create_get_sensors (dev, TIMEOUT); //update position from sensors
		/*
		printf("VA: %f  Rerror: %f  position : %f %f %f  bumpers: %d %d\n", vA, rError, hands->pos2d->px, hands->pos2d->py, hands->pos2d->pa, hands->bumper->bumpers[0], hands->bumper->bumpers[1]);*/
	} 
	
	create_set_speeds (dev, 0, 0); //STOP!
	create_get_sensors (dev, TIMEOUT); //update position from sensors
	return rotError(dev, &rotData, A);
}

double hall_center_err(api_HANDLES_t * dev, FilterHandles_t * filt, int * walls) {
	/*This assumes that sonar:0 is on the right of the robot when looking at it
	from above and sonar:1 is on the left
	Negative error means too far to the right and positive means to far left*/
	double right;
	double left;
	double error;
	
	right = nextSample(filt->sonarR, dev->t->sonar[0]);
	left = nextSample(filt->sonarL,dev->t->sonar[1]);
	
	if(right <= HALL_VAR && left <= HALL_VAR) {	//both sonar can see a wall
		error = right - left;
		*walls = WALLS_BOTH;
	} else if(right <= HALL_VAR && left > HALL_VAR) {	//right sonar can see a wall and left cannot
		error = (HALL_WIDTH/2.0) - right;
		*walls = WALLS_RIGHT_ONLY;
	} else if(left <= HALL_VAR && right > HALL_VAR) {	//left sonar can see a wall and right cannot
		error = left - (HALL_WIDTH/2.0);
		*walls = WALLS_LEFT_ONLY;
	} else {
		error = 0.0;	// Stay in center
		*walls = WALLS_NONE;
	}
	return error;
}

int checkInFront(api_HANDLES_t * dev,FilterData_t *filter) {
	/*
	 *	This function takes the ir handle and filter, updates the ir filter data 
	 *	using the handle and then checkes to see if the updated value is less
	 *	than the set variance.  If it is it returns a 1 indicating something is
	 *	in front of it, and it needs to stop and wait
	 */
	int value;
	
	value = nextSample(filter, dev->t->ir[0]);
	
	if(value <= IR_VAR) {
		return 1;
	}
	return 0;
}
