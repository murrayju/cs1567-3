//Headers for all structs and defines
#ifndef __types_h
#define __types_h

#include "TurretAPI.h"
#include "create_comms.h"

#define DEBUG
#define ABSOLUTE_COORD

#define PI 3.141592654

#define NUMERR 10       //Number of errors to keep in PID history array

//define PID types
#define TRANS_PID   0
#define ANGLE_PID   1
#define SONAR_PID   2

#define NUM_PID_C   5
//define PID coeffs {Kp, Kd, Ki, tolerance, maxI}
#define TRANS_PID_C {0.9, 1.0, 0.01, 0.5, 10.0}
#define ANGLE_PID_C {0.5, 1.0, 0.005, 0.01, 10.0}
#define SONAR_PID_C {0.006, 0.006, 0.00001, 0.00005, 0.0001}


#define HALL_WIDTH 240  //width of hallway in centimeters
#define HALL_VAR 220    //hallway sonar max value acceptable
#define IR_VAR 45.0     //ir sensor variance

#define TIMEOUT 0.1

// Wall status defines
#define WALLS_BOTH        3
#define WALLS_LEFT_ONLY   2
#define WALLS_RIGHT_ONLY  1
#define WALLS_NONE        0

//number of filter samples
#define MAX_SAMPLES 16

//defines filter type
#define FILT_IR     0
#define FILT_SONAR  1
#define FILT_ODO    2

//defines number of samples
#define FILT_IR_SAMPLES     16
#define FILT_SONAR_SAMPLES  16
#define FILT_ODO_SAMPLES    16

//define max sample value
#define FILT_IR_MAX     50
#define FILT_SONAR_MAX  260
#define FILT_ODO_MAX    1000

//Filter Coefficients
#define FILT_SONAR_COEFFS    {0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}
#define FILT_IR_COEFFS   {0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}//{0.211939524148286456695089441382151562721,0.471832709834875574372858864080626517534,0.471832709834875574372858864080626517534,0.211939524148286456695089441382151562721}
#define FILT_ODO_COEFFS    {0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}

//Turrett settings
#define T_ANGLE 93.0
#define COMPORT "/dev/ttyS2"

typedef struct _pidData {
    double Kp, Kd, Ki, tol, maxI;
    double errorHist[NUMERR];
    int iErr;
    int doDiff;
} pidData_t;

typedef struct _api_HANDLES {
    create_comm_t *c;
    turret_comm_t *t;
} api_HANDLES_t;

//Filter data struct
typedef struct _FilterData {
    int max;
    int num_samples;
    double coefficients[MAX_SAMPLES];
    int  next_index;
    double samples[MAX_SAMPLES];
} FilterData_t;

typedef struct _FilterHandles {
    FilterData_t * sonarL;
    FilterData_t * sonarR;
    FilterData_t * ir;
    FilterData_t * xCorrect;
    FilterData_t * yCorrect;
} FilterHandles_t;

typedef struct _pidHandles {
    pidData_t * trans;
    pidData_t * angle;
    pidData_t * sonar;
} pidHandles_t;

typedef struct _coordData {
    double X;
    double Y;
} coordData_t;

#endif
