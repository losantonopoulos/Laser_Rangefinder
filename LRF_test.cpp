// General / System includes
#include <stdio.h>
#include <iostream>

// LRF 
#include "LRF.h"

using namespace std;

int main(int argc, char *argv[]){

	const char *dev = "/dev/ttyUSB1";	
	LRF disto = LRF(dev);

/*	disto.enablePointer(0); // enable pointer
	
	cout << "Testing Sequential switching" << endl;

	cout << " " << disto.disablePointer(0) << endl;
	cout << " " << disto.enablePointer(0)  << endl;

	cout << " " << disto.enablePointer(1) << endl;
	cout << " " << disto.enablePointer(1) << endl;
	cout << " " << disto.enablePointer(0) << endl;
	cout << " " << disto.enablePointer(0) << endl;

	cout << " " << disto.disablePointer(0) << endl;
	cout << " " << disto.disablePointer(1) << endl;
	cout << " " << disto.disablePointer(1) << endl;
	cout << " " << disto.disablePointer(0) << endl;

	cout << " " << disto.enablePointer(0)  << endl;
	cout << " " << disto.disablePointer(1) << endl;
	cout << " " << disto.enablePointer(1)  << endl;
	cout << " " << disto.disablePointer(1) << endl;
	cout << " " << disto.enablePointer(0)  << endl;
	cout << " " << disto.disablePointer(0) << endl;
	cout << " " << disto.enablePointer(1)  << endl;
	cout << " " << disto.disablePointer(1) << endl;
	cout << " " << disto.enablePointer(0)  << endl;
	cout << " " << disto.disablePointer(0) << endl;

	cout << " " << disto.enablePointer(0)  << endl;
	
	double measurement;

	cout << "\nTesting Single Measurements\n";
	
	measurement = disto.getMeasurement();
	cout << "\nDistance: " << measurement << endl;
	
	measurement = disto.getMeasurement();
	cout << "\nDistance: " << measurement << endl;

	measurement = disto.getMeasurement();
	cout << "\nDistance: " << measurement << endl;
	
	measurement = disto.getMeasurement();
	cout << "\nDistance: " << measurement << endl;

	measurement = disto.getMeasurement();
	cout << "\nDistance: " << measurement << endl;
*/

	double 	avg = 0.0; 
	double	var = 0.0;	
	int 	max_rounds = 13;

	cout << "Single Measurement: ";
	cout << disto.getMeasurement() << endl;

	cout << "\nTesting " << max_rounds << " continous measurements\n";

	cout << disto.getMeasurements(&avg, &var, &max_rounds) << endl;
	cout << "Rounds: " << max_rounds << " Average: " << avg << " Variance: " << var << endl;
	
	cout << "Laser pointer State: " << disto.isON() << endl;

	return 0;
}
