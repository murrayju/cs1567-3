//Justin Murray
//Dean Pantages
//FIRlib.c
//Contains the FIR Filter Functions

#include <stdlib.h>
#include <string.h>
#include "FIRlib.h"

double nextSample(FilterData_t * f, double nextVal) {
    /*
     *  Takes a Filter Struct and the newest sample value and
     *  returns the filtered data and updates the sample array which is
     *  implemented as a ring buffer
     */
     double sum = 0.0;
     int i,j;

     // Checks if value is within threshold
     if(nextVal > f->max) {
         nextVal = f->max;
     } else if(nextVal < -(f->max)) {
         nextVal = -(f->max);
     }

     //adds the newest value into the next slot of the array
     f->samples[f->next_index] = nextVal;

     //this calculates the sum
     j = f->next_index;
     for(i = 0;i < f->num_samples; i++) {
        sum += f->coefficients[i] * f->samples[j];
        j++;
        if(j == f->num_samples) {
            j = 0;
        }
     }

     //increments the next index location and resets it to zero if
     //it equals SAMPLES
     f->next_index++;
     f->next_index %= f->num_samples;
     return sum;
}

FilterData_t * initializeFilter(int type) {
    /*  This function initializes a FilterData struct to be used in the
    *   nextSample function.  The coefficients for the filter struct are *specified in the FIRlib.h header config file as well as the number of samples for each filter
    */

    int i;
    static double ir_coeffs[] = FILT_IR_COEFFS;
    static double sonar_coeffs[] = FILT_SONAR_COEFFS;
    static double odo_coeffs[] = FILT_ODO_COEFFS;
    FilterData_t * f = malloc(sizeof(FilterData_t));

    // Initialize everything to zero
    memset(f,0,sizeof(FilterData_t));
    if(f == NULL) { return NULL; }

    // Copy the #defined coeffs into the struct
    if(type == FILT_IR) {
        memcpy(f->coefficients, ir_coeffs, FILT_IR_SAMPLES * sizeof(double));
        f->num_samples = FILT_IR_SAMPLES;
        f->max = FILT_IR_MAX;
    } else if(type == FILT_SONAR) {
        memcpy(f->coefficients, sonar_coeffs, FILT_SONAR_SAMPLES * sizeof(double));
        f->num_samples = FILT_SONAR_SAMPLES;
        f->max = FILT_SONAR_MAX;
    } else if(type == FILT_ODO) {
        memcpy(f->coefficients, odo_coeffs, FILT_ODO_SAMPLES * sizeof(double));
        f->num_samples = FILT_ODO_SAMPLES;
        f->max = FILT_ODO_MAX;
    }
    return f;
}

void filterSonar(api_HANDLES_t * dev, FilterHandles_t * filter, double * sonar0, double * sonar1) {
    turret_get_sonar(dev->t);
    *sonar0 = nextSample(filter->sonar0, dev->t->sonar[0]);
    *sonar1 = nextSample(filter->sonar1, dev->t->sonar[1]);
}

void filterIR(api_HANDLES_t * dev, FilterHandles_t * filter, double * ir0, double * ir1) {
    turret_get_ir(dev->t);
    *ir0 = nextSample(filter->ir0, dev->t->ir[0]);
    *ir1 = nextSample(filter->ir1, dev->t->ir[1]);
}

void filterOdometry(api_HANDLES_t * dev, FilterData_t * d, FilterData_t * a, double * dist, double * angle) {
    create_get_sensors(dev->c, TIMEOUT);
    *dist = nextSample(d, dev->c->dist);
    *angle = nextSample(a, dev->c->angle);
    //printf("D: %f\tA: %f\n",*dist,*angle);
}
