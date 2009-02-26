//Justin Murray
//Dean Pantages
//FIRlib.c
//Contains the FIR Filter Functions

#include <stdlib.h>
#include <string.h>

#include "FIRlib.h"

double nextSample(FilterData_t * f, double nextVal) {
	/*
	 *	Takes a Filter Struct and the newest sample value and
	 *	returns the filtered data and updates the sample array which is 
	 *	implemented as a ring buffer
	 */
	 double sum = 0.0;
	 int i,j;
	 
	 // Checks if value is within threshold
	 if(nextVal > f->max) {
		 nextVal = f->max;
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
	/*	This function initializes a FilterData struct to be used in the 
	*	nextSample function.  The coefficients for the filter struct are *specified in the FIRlib.h header config file as well as the number of samples for each filter
	*/
	
	int i;
	double ir_coeffs[] = IR_COEFFS;
	double sonar_coeffs[] = SONAR_COEFFS;
	FilterData_t *f = malloc(sizeof(FilterData_t));
	
	// Initialize everything to zero
	memset(f,0,sizeof(FilterData_t));
	
	// Copy the #defined coeffs into the struct
	if(type == IR) {
		memcpy(f->coefficients, ir_coeffs, IR_SAMPLES * sizeof(double));
		f->num_samples = IR_SAMPLES;
		f->max = IR_MAX;
	} else if(type == SONAR) {
		memcpy(f->coefficients, sonar_coeffs, SONAR_SAMPLES * sizeof(double));
		f->num_samples = SONAR_SAMPLES;
		f->max = SONAR_MAX;
	}
	return f;
}
