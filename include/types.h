//Headers for all structs and defines
#ifndef __types_h
#define __types_h

#include "TurretAPI.h"
#include "create_comms.h"

#define DEBUG
#define ABSOLUTE_COORD
#define USE_IR_MODE //When this is defined, use the IR to sense the walls instead of Sonar

#define PI 3.141592654

#define STRAIGHT_TOL PI/16.0

#define NUMERR 10       //Number of errors to keep in PID history array

//define PID types
#define TRANS_PID   0
#define ANGLE_PID   1
#define SONAR_PID   2

#define NUM_PID_C   5
//define PID coeffs {Kp, Kd, Ki, tolerance, maxI}
#define TRANS_PID_C {0.8, 1.0, 0.01, 0.5, 10.0}
#ifdef USE_IR_MODE
    #define SONAR_PID_C {0.009, 0.002, 0.00002, 0.1, 0.0001}
    #define ANGLE_PID_C {0.4, 0.5, 0.0002, 0.01, 10.0}
#else
    #define SONAR_PID_C {0.0045, 0.01, 0.00001, 0.1, 0.0001}
    #define ANGLE_PID_C {0.6, 0.5, 0.0002, 0.01, 10.0}
#endif





#ifdef USE_IR_MODE
    #define HALL_WIDTH 150.0  //width of hallway in centimeters
    #define HALL_VAR 50.0   //hallway error max value acceptable
#else
    #define HALL_WIDTH 240.0  //width of hallway in centimeters
    #define HALL_VAR 220.0  //hallway error max value acceptable
#endif
#define FRONT_DIST 45.0     //Detect in front distance

#define TIMEOUT 200

// Wall status defines
#define WALLS_BOTH        3
#define WALLS_LEFT_ONLY   2
#define WALLS_RIGHT_ONLY  1
#define WALLS_NONE        0

//number of filter samples
#define MAX_SAMPLES 9

//defines filter type
#define FILT_IR     0
#define FILT_SONAR  1
#define FILT_ODO  2

//defines number of samples
#define FILT_IR_SAMPLES     9
#define FILT_SONAR_SAMPLES  9
#define FILT_ODO_SAMPLES    9

//define max sample value
#define FILT_IR_MAX     50
#define FILT_SONAR_MAX  260
#define FILT_ODO_MAX    0.1

//Filter Coefficients
#define FILT_SONAR_COEFFS   {0.055297513, 0.09282488, 0.12569353, 0.14813437, 0.15609948, 0.14813437, 0.12569353, 0.09282488, 0.055297513}//{0.0042270822, 0.018569125, 0.034649216, 0.051321678, 0.06732041, 0.08137835, 0.092350215, 0.09932517, 0.101717494, 0.09932517, 0.092350215, 0.08137835, 0.06732041, 0.051321678, 0.034649216, 0.018569125, 0.0042270822}//{0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}
#define FILT_IR_COEFFS      {0.055297513, 0.09282488, 0.12569353, 0.14813437, 0.15609948, 0.14813437, 0.12569353, 0.09282488, 0.055297513}//{0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}
#define FILT_ODO_COEFFS     {0.055297513, 0.09282488, 0.12569353, 0.14813437, 0.15609948, 0.14813437, 0.12569353, 0.09282488, 0.055297513}//{0.25,0.2,0.1,0.02,0.0,0.06,0.15,0.22}


//Turrett settings
#ifdef USE_IR_MODE
    #define T_ANGLE 167.0
#else
    #define T_ANGLE 87.0
#endif
#define COMPORT "/dev/ttyS2"

//Function to normalize an angle
#ifndef NORMALIZE
  #define NORMALIZE(z) atan2(sin(z), cos(z))
#endif


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
    double max;
    int num_samples;
    double coefficients[MAX_SAMPLES];
    int  next_index;
    double samples[MAX_SAMPLES];
} FilterData_t;

typedef struct _FilterHandles {
    FilterData_t * sonarL;
    FilterData_t * sonarR;
    FilterData_t * ir0;
    FilterData_t * ir1;
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

typedef struct _roboPos {
    double x;
    double y;
    double a;
} roboPos_t;

#endif
