#ifndef LRF_H
#define LRF_H

#include <iostream>

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <sys/ioctl.h>   /* Serial Port IO Controls */

#include <chrono>
#include <ctime>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> 

#define DEFAULT_BAUD	B115200
#define DEFAULT_DEV		"/dev/ttyUSB0"

// Messages 

#define MSG_OK			"$00023335&"
#define MSG_ON			"$0003260130&"
#define MSG_OFF			"$00022123&"
#define MSG_SINGLE_MEAS	"$00022123&"
#define MSG_CONTIN_MEAS	"$00022426&"
#define MSG_STOP_MEAS	"$0003260029&"

// Sizes

#define MAX_BYTES			255
#define REPLY_ON_LEN		22
#define REPLY_OFF_LEN		28
#define REPLY_SINGLE_LEN	28
#define REPLY_CONTIN_LEN	38

// Timing constants

#define WAIT_SER_DUR_US		5000
#define WAIT_DEV_DUR_US		10000
#define POLLING_DELAY_US	2000
#define PRE_FLUSH_DUR_US	1000
#define FLUSH_DUR_MS		500
#define REPLY_OK_DUR_MS		1000
#define REPLY_ON_DUR_MS		3000		
#define REPLY_OFF_DUR_MS	5500
#define REPLY_MEAS_DUR_MS	5500

// Distances

#define WRONG_DISTANCE		0.0
#define MIN_DISTANCE_M		0.15
#define MAX_DISTANCE_M		40.0

//#define DBG
//#define SHOW_MEASUREMENT

class LRF
{

	private:

		// Variables
		
		bool 	laser_on = false;

		int 	fd, c;

		double 	dist_min, dist_max;

		struct 	termios tty_old, tty;
		
		char 	buf[MAX_BYTES];

		// Functions
	    
	    int 	readSerial(char *buffer, int n_bytes, int timeout_ms);

	    bool 	flushSerial(int timeout_ms);

	public:

		// Variables

		// Functions

		LRF();						// Default Constructor

	    LRF(const char *dev);	// Constructor 

	    ~LRF(); // Destructor

	    double 	getMeasurement();

	    bool 	getMeasurements(double *average, double *variance, int *max_rounds);

	    bool 	enablePointer(bool override);

	    bool 	disablePointer(bool override);

	    bool 	isON(){return laser_on;};
	   	
};
 
#endif