//Dean Pantages
//Justin Murray
//FIR Library Header File

//number of filter samples
#define MAX_SAMPLES 8

//defines filter type
#define IR = 1
#define SONAR = 2

//defines number of samples
#define IR_SAMPLES 4
#define SONAR_SAMPLES 8

//Filter Coefficients
#define S_COEFFS {-0.0484295452148003,0.0316417725690449,0.0663052386394815,0.0161092502141281,-0.0761706272785959,-0.0417806265179821,0.183762552952241,0.417189375297071,0.417189375297071,0.183762552952241,-0.0417806265179821,-0.0761706272785959,0.0161092502141281,0.0663052386394815,0.0316417725690449,-0.0484295452148003}
#define IR_COEFFS {,,,}

//Filter data struct
typedef struct _FilterData
{
  int num_samples;
  double coefficients[MAX_SAMPLES];
  int  next_index;
  double samples[MAX_SAMPLES];
} FilterData_t;


//Next Sample Function see FIRlib.c for implementation details
double nextSample(FilterData_t *filter, double nextVal);

//Initializes a FilterData_t struct see FIRlib.c for implementation details
FilterData_t * initializeFilter(int type);
