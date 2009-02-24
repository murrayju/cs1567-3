//Justin Murray
//Dean Pantages
//FIRlib.c
//Contains the FIR Filter Functions

#include "FIRlib.h"

double nextSample(FilterData_t *filter, double nextVal)
{
	/*
	 *	Takes a Filter Struct and the newest sample value and
	 *	returns the filtered data and updates the sample array which is 
	 *	implemented as a ring buffer
	 */
	 double sum;
	 int i,j;
	 
	 //adds the newest value into the next slot of the array
	 f->samples[f->next_index] = nextVal;
	 
	 //this calculates the sum
	 j = next_index;
	 for(i = 0;i < filter->num_samples; i++)
	 {
	 	sum += coefficients[i]*samples[j]
	 	j++;
	 	if(j == filter->num_samples)
	 	{
	 		j = 0;
	 	}
	 }
	 
	 //increments the next index location and resets it to zero if
	 //it equals SAMPLES
	 next_index++;
	 if(next_index = filter->num_samples)
	 {
	 	next_index = 0;
	 }
	 
	 
	 
}

FilterData_t * initializeFilter(int type)
{
	/*	This function initializes a FilterData struct to be used in the 
	*	nextSample function.  The coefficients for the filter struct are *specified in the FIRlib.h header config file as well as the number of samples for each filter
	*/
	
	int i;
	FilterData_t *f = malloc(sizeof(FilterData_t));
	
	for(i = 0; i < MAX_SAMPLES; i++)
	{
		    f->samples[i] = 0;
	}
	
	if(type == IR){
		f->coefficients = IR_COEFFS;
		num_samples = IR_SAMPLES;
	}
	else if(type == SONAR){
		f->coefficients = SONAR_COEFFS;
		num_samples = SONAR_SAMPLES;
	}
	f->next_index = 0;
}
