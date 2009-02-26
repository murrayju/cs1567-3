//Justin Murray
//Dean Pantages
//PID control functions

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <libplayerc/playerc.h>
#include "PIDlib.h"
#include "../FIRlib/FIRlib.h"

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

int checkInFront(playerc_HANDLES_t *, FilterData_t *);
double hall_center_err(playerc_HANDLES_t *, FilterHandles_t *, int *);

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

double tranError(playerc_position2d_t * pos2D, pid_data * data, double tX, double tY) {
	double relX = pos2D->px - data->Xi; //X distance travelled
	double relY = pos2D->py - data->Yi; //Y distance travelled
	double Xreal, Yreal, Xrem, Yrem;
	double mew, theta, phi, error;
	data->iErr = (data->iErr + 1) % NUMERR;

	//Find the remaining distance from target
	Xrem = tX - pos2D->px;
	Yrem = tY - pos2D->py;
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

double targetRotError(playerc_position2d_t * pos2D, pid_data * data, double tX, double tY) {
	double relX = pos2D->px - data->Xi; //X distance travelled
	double relY = pos2D->py - data->Yi; //Y distance travelled
	double Xreal, Yreal, Xrem, Yrem;
	double mew, theta, phi, error;
	data->iErr = (data->iErr + 1) % NUMERR;

	//find the remaining distance to the target
	Xrem = tX - pos2D->px;
	Yrem = tY - pos2D->py;
	
	//find the correct angle from pos to target
	//phi = atan2(Yrem, Xrem);
	if(Xrem == 0.0) {
		if(Yrem >= 0.0) {
			phi = PI/2.0;
		} else {
			phi = -PI/2.0;
		}
	} else if(Xrem > 0.0) {
		phi = atan(Yrem / Xrem);
	} else { //if(Xrem < 0.0) {
		if(Yrem >= 0.0) {
			phi = PI + atan(Yrem / Xrem);
		} else {
			phi = atan(Yrem / Xrem) - PI;
		}
	}
	
	//error = phi - pos2D->pa;
	error = angleDiff(phi, pos2D->pa);
	printf("TRE: dist=(%f,%f) phi=%f err=%f\n",Xrem,Yrem,phi,error);
	return data->errorHist[data->iErr] = error;
}

double rotError(playerc_position2d_t * pos2D, pid_data * data, double tA) {
	double relA = pos2D->pa;
	double error;
	
	data->iErr = (data->iErr + 1) % NUMERR;
	error = angleDiff(tA, pos2D->pa);
	
	return data->errorHist[data->iErr] = error;
}


int bumped(playerc_HANDLES_t * hands) {
#ifdef stage_environment
	return 0;
#else
	static int storeBump = 0;
	if(storeBump) {
		return 1;
	} else {
		return storeBump = (hands->bumper->bumpers[0] || hands->bumper->bumpers[1]);
	}
#endif
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

double Move(playerc_HANDLES_t * hands, double X, double Y) {
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
	
	playerc_client_read(hands->client);
	
	//Store initial position info
	tranData.Xi = hands->pos2d->px;
	tranData.Yi = hands->pos2d->py;
	tranData.Ai = hands->pos2d->pa;
	
	rotData.Xi = hands->pos2d->px;
	rotData.Yi = hands->pos2d->py;
	rotData.Ai = hands->pos2d->pa;
	
	while(!bumped(hands) && (tError = tranError(hands->pos2d, &tranData, X, Y)) > tranData.tol) {
		rError = targetRotError(hands->pos2d,&rotData, X, Y);
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
		playerc_position2d_set_cmd_vel(hands->pos2d, vX, 0, vA, 1); //set new speeds
		playerc_client_read(hands->client); //update position from sensors
		
		printf("VX: %f  VA: %f  Terror: %f  Rerror: %f  position : %f %f %f  bumpers: %d %d\n", vX, vA, tError, rError, hands->pos2d->px, hands->pos2d->py, hands->pos2d->pa, hands->bumper->bumpers[0], hands->bumper->bumpers[1]);
	} 
	
	playerc_position2d_set_cmd_vel(hands->pos2d, 0, 0, 0, 1); //STOP!
	playerc_client_read(hands->client); //update position from sensors
	
	return tranError(hands->pos2d, &tranData, X, Y);
}

double Turn(playerc_HANDLES_t * hands, double A) {
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
	
	playerc_client_read(hands->client);
	
	//Store initial position info
	rotData.Xi = hands->pos2d->px;
	rotData.Yi = hands->pos2d->py;
	rotData.Ai = hands->pos2d->pa;
	
	while(!bumped(hands) && fabs(rError = rotError(hands->pos2d,&rotData, A)) > rotData.tol) {
		vA = PID(&rotData);
		playerc_position2d_set_cmd_vel(hands->pos2d, 0, 0, vA, 1); //set new speeds
		playerc_client_read(hands->client); //update position from sensors
		
		printf("VA: %f  Rerror: %f  position : %f %f %f  bumpers: %d %d\n", vA, rError, hands->pos2d->px, hands->pos2d->py, hands->pos2d->pa, hands->bumper->bumpers[0], hands->bumper->bumpers[1]);
	} 
	
	playerc_position2d_set_cmd_vel(hands->pos2d, 0, 0, 0, 1); //STOP!
	playerc_client_read(hands->client); //update position from sensors
	return rotError(hands->pos2d, &rotData, A);
}

double hall_center_err(playerc_HANDLES_t * hands, FilterHandles_t * filt, int * walls) {
	/*This assumes that sonar:0 is on the right of the robot when looking at it
	from above and sonar:1 is on the left
	Negative error means too far to the right and positive means to far left*/
	double right;
	double left;
	double error;
	
	right = nextSample(filt->sonarR, hands->sonar->scan[0]);
	left = nextSample(filt->sonarL,hands->sonar->scan[1]);
	
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

int checkInFront(playerc_HANDLES_t * hands,FilterData_t *filter) {
	/*
	 *	This function takes the ir handle and filter, updates the ir filter data 
	 *	using the handle and then checkes to see if the updated value is less
	 *	than the set variance.  If it is it returns a 1 indicating something is
	 *	in front of it, and it needs to stop and wait
	 */
	int value;
	
	value = nextSample(filter, hands->ir->data.ranges[0]);
	
	if(value <= IR_VAR) {
		return 1;
	}
	return 0;
}
