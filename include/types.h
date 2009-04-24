//Headers for all structs and defines
#ifndef __types_h
#define __types_h

#include "TurretAPI.h"
#include "create_comms.h"

#define DEBUG
#define ABSOLUTE_COORD
#define USE_IR_MODE //When this is defined, use the IR to sense the walls instead of Sonar
//#define FILTER_ODO
//#define HAL
#define JOHNNY5

#define PI    (3.141592654)
#define NORTH (0.0)
#define SOUTH (PI)
#define EAST  (-PI/2.0)
#define WEST  (PI/2.0)

#define STRAIGHT_TOL PI/16.0

#define NUMERR 10       //Number of errors to keep in PID history array

//define PID types
#define TRANS_PID   0
#define ANGLE_PID   1
#define SONAR_PID   2
#define ANGLET_PID   3

#define NUM_PID_C   5
//define PID coeffs {Kp, Kd, Ki, tolerance, maxI}
#define TRANS_PID_C {0.15, 0.3, 0.005, 0.1, 10.0}		//changed these pids
#ifdef USE_IR_MODE
    #define SONAR_PID_C {0.009, 0.002, 0.00002, 0.1, 0.0001}
    #define ANGLE_PID_C {0.6, 0.5, 0.0002, 0.1, 1.0}
#else
    #define SONAR_PID_C {0.0045, 0.01, 0.00001, 0.1, 0.0001}
    #define ANGLE_PID_C {0.6, 0.5, 0.0002, 0.1, 1.0}
#endif
#define ANGLET_PID_C {0.5, 0.3, 0.006, 0.1, 0.5}

#ifdef HAL
#define BATT_FACT 6.91
#elseifdef JOHNNY5
#define BATT_FACT 10.00
#else
#define BATT_FACT 6.91
#endif

#define LOOP_SLEEP 10000
#define ODO_SLEEP  50000

#ifdef USE_IR_MODE
    #define HALL_WIDTH 80.0  //width of hallway in centimeters
    #define HALL_VAR 50.0   //hallway error max value acceptable
    #define SIDE_DIST  45.0    //Robot is very close to wall
#else
    #define HALL_WIDTH 80.0  //width of hallway in centimeters
    #define HALL_VAR 80.0  //hallway error max value acceptable
    #define SIDE_DIST   45.0    //Robot is very close to wall
#endif
#define FRONT_DIST  40.0    //Detect in front distance
#define SLOW_VX     0.25     //Slow robot speed for safe turning

#define CELL_DIST   (85.0/100.0)
#define CELL_VAR    80.0
#define HALFCELL_VAR (CELL_VAR / 2.0)

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
#define FILT_ODO    2

//defines number of samples
#define FILT_IR_SAMPLES     9
#define FILT_SONAR_SAMPLES  9
#define FILT_ODO_SAMPLES    5

//define max sample value
#define FILT_IR_MAX     50
#define FILT_SONAR_MAX  260
#define FILT_ODO_MAX    0.5

//Filter Coefficients
#define FILT_SONAR_COEFFS   {0.055297513, 0.09282488, 0.12569353, 0.14813437, 0.15609948, 0.14813437, 0.12569353, 0.09282488, 0.055297513}//{0.0042270822, 0.018569125, 0.034649216, 0.051321678, 0.06732041, 0.08137835, 0.092350215, 0.09932517, 0.101717494, 0.09932517, 0.092350215, 0.08137835, 0.06732041, 0.051321678, 0.034649216, 0.018569125, 0.0042270822}//{0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}
#define FILT_IR_COEFFS      {0.055297513, 0.09282488, 0.12569353, 0.14813437, 0.15609948, 0.14813437, 0.12569353, 0.09282488, 0.055297513}//{0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}
#define FILT_ODO_COEFFS     {0.1388507, 0.22886887, 0.26456094, 0.22886887, 0.1388507}


//Turrett settings
#ifdef USE_IR_MODE
    #ifdef PRIS
        #define T_ANGLE 167.0
    #elseifdef JOHNNY5
        #define T_ANGLE 0.0
    #else
        #define T_ANGLE 0.0
    #endif
#else
    #define T_ANGLE 85.0
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
    double ox;
    double oy;
    double oa;
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
    FilterData_t * sonar0;
    FilterData_t * sonar1;
    FilterData_t * ir0;
    FilterData_t * ir1;
} FilterHandles_t;

typedef struct _pidHandles {
    pidData_t * trans;
    pidData_t * angle;
    pidData_t * sonar;
    pidData_t * angleT;
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

typedef struct _mazeNode {
    int walls;
    struct _mazeNode * N;
    struct _mazeNode * S;
    struct _mazeNode * E;
    struct _mazeNode * W;
    struct _mazeNode * from;
    int probset;
    int probN;
    int probS;
    int probE;
    int probW;
} mazeNode;

#endif
