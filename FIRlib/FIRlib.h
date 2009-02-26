//Dean Pantages
//Justin Murray
//FIR Library Header File

//number of filter samples
#define MAX_SAMPLES 8

//defines filter type
#define IR	0
#define SONAR	1

//defines number of samples
#define IR_SAMPLES	4
#define SONAR_SAMPLES	8

//Filter Coefficients
#define SONAR_COEFFS {-0.133866807270303117549659077667456585914,-0.083766746130837330119867090161278611049,0.202561239153125849377090617053909227252,0.406378113351603009739676508615957573056,0.406378113351603009739676508615957573056,-0.202561239153125849377090617053909227252,-0.083766746130837330119867090161278611049,-0.133866807270303117549659077667456585914}
#define IR_COEFFS {0.211939524148286456695089441382151562721,0.471832709834875574372858864080626517534,0.471832709834875574372858864080626517534,0.211939524148286456695089441382151562721}

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
