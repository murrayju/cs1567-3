//Dean Pantages
//Justin Murray
//FIR Library Header File

//number of filter samples
#define MAX_SAMPLES 16

//defines filter type
#define IR	0
#define SONAR	1

//defines number of samples
#define IR_SAMPLES	16
#define SONAR_SAMPLES	16

//define max sample value
#define IR_MAX		50
#define SONAR_MAX	260

//Filter Coefficients
#define SONAR_COEFFS 	{0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}
#define IR_COEFFS 	{0.125,0.120242471,0.106694174,0.086417715,0.0625,0.038582285,0.018305826,0.004757529,0,0.004757529,0.018305826,0.038582285,0.0625,0.086417715,0.106694174,0.120242471}//{0.211939524148286456695089441382151562721,0.471832709834875574372858864080626517534,0.471832709834875574372858864080626517534,0.211939524148286456695089441382151562721}

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
} FilterHandles_t;

//Next Sample Function see FIRlib.c for implementation details
double nextSample(FilterData_t *filter, double nextVal);

//Initializes a FilterData_t struct see FIRlib.c for implementation details
FilterData_t * initializeFilter(int type);
