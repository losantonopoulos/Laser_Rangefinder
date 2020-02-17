#include "LRF.h"
 
// LRF Default Constructor

LRF::LRF()
{
	fd = open(DEFAULT_DEV, O_RDWR | O_NDELAY | O_NOCTTY);

	if (fd < 0){

		fprintf(stderr,"\nLRF::Could not communicate with device at %s ", DEFAULT_DEV);
		perror(DEFAULT_DEV);
		exit(-1); 

	}else{
		printf("\nLRF::Communication established at %s", DEFAULT_DEV);
	}

	printf("\nLRF::Initializing device...");

	dist_min = MIN_DISTANCE_M;
	dist_max = MAX_DISTANCE_M;

	memset(&tty, 0, sizeof(tty));

	tty.c_cflag 	&= 	~PARENB;
	tty.c_cflag 	&=	~CSTOPB;
	tty.c_cflag 	|=	CS8;
	tty.c_cflag     &=  ~CRTSCTS;
	tty.c_cflag 	|=	CREAD | CLOCAL;

	tty.c_lflag 	&=	~ICANON;
	tty.c_lflag 	&=	~ECHO; 		// Disable echo
	tty.c_lflag 	&=	~ECHOE; 	// Disable erasure
	tty.c_lflag 	&=	~ECHONL; 	// Disable new-line echo
	tty.c_lflag 	&=	~ISIG; 		// Disable interpretation of INTR, QUIT and SUSP

	tty.c_iflag 	&=	~(IXON | IXOFF | IXANY); 							// Turn off s/w flow ctrl
	tty.c_iflag		&=	~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);	// Disable any special handling of received bytes

	tty.c_oflag		&=	~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag		&=	~ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 	0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] 	= 	1;

	cfsetispeed(&tty, DEFAULT_BAUD);
	cfsetospeed(&tty, DEFAULT_BAUD);
	
	tcflush(fd, TCIOFLUSH);

	// Save tty settings, also checking for error
	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
	    printf("\nLRF::Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}

	laser_on = false;

	LRF::disablePointer(true);

	printf("\nLRF::Initialized\n");
}

LRF::LRF(const char *dev)
{

	fd = open(dev, O_RDWR | O_NDELAY | O_NOCTTY);

	if (fd < 0){

		fprintf(stderr,"\nLRF::Could not communicate with device at %s ", dev);
		perror(dev);
		exit(-1); 

	}else{
		printf("\nLRF::Communication established at %s", dev);
	}

	printf("\nLRF::Initializing device...");

	dist_min = MIN_DISTANCE_M;
	dist_max = MAX_DISTANCE_M;

	memset(&tty, 0, sizeof(tty));

	tty.c_cflag 	&= 	~PARENB;
	tty.c_cflag 	&=	~CSTOPB;
	tty.c_cflag 	|=	CS8;
	tty.c_cflag     &=  ~CRTSCTS;
	tty.c_cflag 	|=	CREAD | CLOCAL;

	tty.c_lflag 	&=	~ICANON;
	tty.c_lflag 	&=	~ECHO; 		// Disable echo
	tty.c_lflag 	&=	~ECHOE; 	// Disable erasure
	tty.c_lflag 	&=	~ECHONL; 	// Disable new-line echo
	tty.c_lflag 	&=	~ISIG; 		// Disable interpretation of INTR, QUIT and SUSP

	tty.c_iflag 	&=	~(IXON | IXOFF | IXANY); 							// Turn off s/w flow ctrl
	tty.c_iflag		&=	~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);	// Disable any special handling of received bytes

	tty.c_oflag		&=	~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag		&=	~ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 	0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] 	= 	1;

	cfsetispeed(&tty, DEFAULT_BAUD);
	cfsetospeed(&tty, DEFAULT_BAUD);
	
	tcflush(fd, TCIOFLUSH);

	// Save tty settings, also checking for error
	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
	    printf("\nLRF::Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}

	laser_on = false;

	LRF::disablePointer(true);

	printf("\nLRF::Initialized\n");
	
}

LRF::~LRF()
{
	printf("\nLRF::Turning Laser OFF");
	LRF::disablePointer(true);
	
	printf("\nLRF::Closing connection\n");
	close(fd);
}

double LRF::getMeasurement()
{

	double res = WRONG_DISTANCE;

	#ifdef DBG
	printf("\nDBG, Measuring...");
	#endif

	if(!LRF::flushSerial(FLUSH_DUR_MS)) {
		
		#ifdef DBG
		printf("\nDBG, Could not flush");
		#endif

		if(laser_on) LRF::enablePointer(true);

		return WRONG_DISTANCE;
	}

	// Request measurement
	int n = write(fd, MSG_SINGLE_MEAS, sizeof(MSG_SINGLE_MEAS) - 1);

	usleep(WAIT_SER_DUR_US);

	n = LRF::readSerial(buf, REPLY_SINGLE_LEN, REPLY_MEAS_DUR_MS);

	#ifdef DBG
	printf("\nDBG, Read: %d", n);
	#endif

	if(n <= 0){

		#ifdef DBG
		printf("\nDBG, Could not read");
		#endif

		if(laser_on) LRF::enablePointer(true);

		return WRONG_DISTANCE;
	
	}else{

		buf[n] = '\0';
	}

	if(n == REPLY_SINGLE_LEN){

		if(strncmp(&buf[0], "$00023335&$000621000", 20) == 0){

			if(strncmp(&buf[20], "0001542&", 8) == 0){

				printf("\nLRF::Distance too short...");
			
			}else if(strncmp(&buf[20], "0001643&", 8) == 0){

				printf("\nLRF::No echo...");
			
			}else if(strncmp(&buf[20], "0001744&", 8) == 0){

				printf("\nLRF::Reflection too strong...");
			
			}else if(strncmp(&buf[20], "0001845&", 8) == 0){

				printf("\nLRF::Ambient light too strong...");
			
			}else{

				char temp[9];

				strncpy(&temp[0], &buf[20], 2);
				temp[2] = '.';
				strncpy(&temp[3], &buf[22], 5);
				temp[8] = '\0';

				res = atof(temp);

				if ((res < dist_min) || (res > dist_max)){
					res = WRONG_DISTANCE;
				}

			}

		}else{

			printf("\nLRF::Could not decode response...");

		}
	} else{

			printf("\nLRF::Could not decode response...");
	}

	if(laser_on) LRF::enablePointer(true);

	return res;
}

bool LRF::getMeasurements(double *average, double *variance, int *max_rounds){

	int 	acctual_rounds	= 0;
	
	double 	measurement 	= 0.0;	

	long double avg 		= 0.0;
	long double var 		= 0.0;
	long double sum 		= 0.0;
	long double sum_squared = 0.0;

	#ifdef DBG
	printf("\nDBG, Measuring...");
	#endif

	if(!LRF::flushSerial(FLUSH_DUR_MS)) {
		
		#ifdef DBG
		printf("\nDBG, Could not flush");
		#endif

		if(laser_on) LRF::enablePointer(true);
				
		return false;
	}

	// Request measurements

	int n = write(fd, MSG_CONTIN_MEAS, sizeof(MSG_CONTIN_MEAS) - 1);

	usleep(WAIT_SER_DUR_US);

	n = LRF::readSerial(buf, (sizeof(MSG_OK) - 1), REPLY_OK_DUR_MS);

	// Check if anything was received 
	if(n <= 0){

		#ifdef DBG
		printf("\nDBG, Could not read");
		#endif

		if(laser_on) LRF::enablePointer(true);
				
		return false;
	
	}else{

		buf[n] = '\0';
	}

	// Check if OK was received 
	if(n >= (sizeof(MSG_OK) - 1)){
		n = strncmp(buf, MSG_OK, (sizeof(MSG_OK) - 1));

		if(n != 0){

			#ifdef DBG
			printf("\nDBG, OK was not received...");
			#endif

			if(laser_on) LRF::enablePointer(true);
				
			return false;
		}
	}

	int max_r = *max_rounds;
	for(int i=0; i<max_r; i++){

		n = LRF::readSerial(buf, REPLY_CONTIN_LEN, REPLY_MEAS_DUR_MS);
		
		#ifdef DBG
		printf("\nDBG, Read: %d", n);
		#endif

		if(n <= 0){

			#ifdef DBG
			printf("\nDBG, Could not read");
			#endif

			continue;
		
		}else{

			buf[n] = '\0';
		}

		if(n == REPLY_CONTIN_LEN){

			//printf("\nDBG, Measurement(%d): \n%s\n", i, buf);
			// $001624  0001   000    04855   000   04855   000   04855  50&
			// 0 - 	 6  7-10  11 13  14 - 18 19-21 22 - 26 27-29 30 - 34

			bool 	is_ok	= 	(strncmp(&buf[0], "$001624", 7) == 0);

			is_ok = is_ok 	&& 	(strncmp(&buf[11], "000", 3) == 0);
			is_ok = is_ok 	&& 	(strncmp(&buf[19], "000", 3) == 0);
			is_ok = is_ok 	&& 	(strncmp(&buf[27], "000", 3) == 0);

			if(is_ok){

				char temp[7];

				strncpy(&temp[0], &buf[14], 2);
				temp[2] = '.';
				strncpy(&temp[3], &buf[16], 3);
				temp[6] = '\0';

				measurement = atof(temp);

				#if defined(DBG) || defined(SHOW_MEASUREMENT)
				printf("\nMeasurement(%d): %f", i, measurement);
				#endif

				if ((measurement < dist_min) || (measurement > dist_max)){
					continue;
				}
				
			}else{

				printf("\nLRF::Decode message invalid");
				continue;
			}

		} else{

				printf("\nLRF::Could not decode response...");

				if(laser_on) LRF::enablePointer(true);
				
				return false;
		}

		acctual_rounds++;

		#ifdef DBG
		printf("\nDBG, Measurement(i: %d - acct: %d): %f", i, acctual_rounds, measurement);
		#endif

		sum 		+=	(long double) measurement;
		sum_squared +=	(long double) measurement*measurement;
	}

	// Requesting to stop
	n = write(fd, MSG_STOP_MEAS, sizeof(MSG_STOP_MEAS) - 1);

	avg = sum / (long double) acctual_rounds;
	var = (sum_squared / (long double) acctual_rounds) - (avg * avg);

	*average	= (double) avg;
	*variance	= (double) var;
	*max_rounds = acctual_rounds;

	#ifdef DBG
	printf("\nDBG, Rounds: %d, Average: %f, Variance: %f", acctual_rounds, (double)avg, (double)var);
	#endif	

	if(laser_on) LRF::enablePointer(true);

	usleep(WAIT_DEV_DUR_US);

	return true;
}

bool LRF::enablePointer(bool override)
{

	if (laser_on && !override) return true;

	#ifdef DBG
	printf("\nDBG, Enabling(%d)...", override);
	#endif

	bool ret = false;

	if(!LRF::flushSerial(FLUSH_DUR_MS)) {
		
		#ifdef DBG
		printf("\nDBG, Could not flush");
		#endif

		return false;
	}

	int n = write(fd, MSG_ON, sizeof(MSG_ON) - 1);

	usleep(WAIT_SER_DUR_US);

	n = LRF::readSerial(buf, REPLY_ON_LEN, REPLY_ON_DUR_MS);

	#ifdef DBG
	printf("\nDBG, Read: %d", n);
	#endif

	if(n <= 0){

		#ifdef DBG
		printf("\nDBG, Could not read");
		#endif

		return false;
	
	}else{

		buf[n] = '\0';
	}

	if(n >= (sizeof(MSG_OK) - 1)){
		n = strncmp(buf, MSG_OK, (sizeof(MSG_OK) - 1));
	}

	ret = (n==0);

	#ifdef DBG
	printf("\nDBG, Enable: (%d)", ret);
	#endif

	if(ret) laser_on = true;
	
	usleep(WAIT_DEV_DUR_US); // wait for the laser to be ready

	return ret;
}

bool LRF::disablePointer(bool override)
{

	if (!laser_on && !override) return true;

	#ifdef DBG
	printf("\nDBG, Disabling(%d)...", override);
	#endif

	bool ret = false;

	if(!LRF::flushSerial(FLUSH_DUR_MS)) {
		
		#ifdef DBG
		printf("\nDBG, Could not flush");
		#endif

		return false;
	}

	int n = write(fd, MSG_OFF, sizeof(MSG_OFF) - 1);

	usleep(WAIT_SER_DUR_US);

	n = LRF::readSerial(buf, REPLY_OFF_LEN, REPLY_OFF_DUR_MS);

	#ifdef DBG
	printf("\nDBG, Read: %d", n);
	#endif

	if(n <= 0){

		#ifdef DBG
		printf("\nDBG, Could not read");
		#endif

		return false;
	
	}else{

		buf[n] = '\0';
	}

	if(n >= (sizeof(MSG_OK) - 1)){
		n = strncmp(buf, MSG_OK, (sizeof(MSG_OK) - 1));
	}

	ret = (n==0);

	#ifdef DBG
	printf("\nDBG, Disable: (%d)", ret);
	#endif

	if(ret) laser_on = false;
	
	usleep(WAIT_DEV_DUR_US); // wait for the laser to be ready

	return ret;
}


int LRF::readSerial(char *buffer, int n_bytes, int timeout_ms){

	auto end_time 	= std::chrono::high_resolution_clock::now();
	auto start_time = std::chrono::high_resolution_clock::now();

	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>((start_time - end_time)).count();

	int n, index = 0;

	while(1){
		
		end_time 	= std::chrono::high_resolution_clock::now();
	    ms 			= std::chrono::duration_cast<std::chrono::milliseconds>((end_time - start_time)).count();
	    
	    if(ms <= timeout_ms){
	    	
	    	n = read(fd, &buf[index], n_bytes);

	    	if(n > 0){

				index  	+= n;
				n_bytes -= n;

				if(n_bytes == 0){

					return index;
				
				}else if(n_bytes < 0){

					return -1;
				}
	    	}
	    	
	    }else{
	    	return -1;
	    }

		usleep(POLLING_DELAY_US);
	}

	return -1;
}

bool LRF::flushSerial(int timeout_ms){

	usleep(PRE_FLUSH_DUR_US);

	auto end_time 	= std::chrono::high_resolution_clock::now();
	auto start_time = std::chrono::high_resolution_clock::now();
	auto ms 		= std::chrono::duration_cast<std::chrono::milliseconds>((start_time - end_time)).count();

	int  n, index 	= 0;

	while(1){

		end_time 	= std::chrono::high_resolution_clock::now();
	    ms 			= std::chrono::duration_cast<std::chrono::milliseconds>((end_time - start_time)).count();
	    
	    if(ms <= timeout_ms){
	    	n = read(fd, &buf[0], MAX_BYTES);

	    	if(n <= 0) return true;

	    }else{
	    	return false;
	    }

		usleep(POLLING_DELAY_US);
	}

	return false;
}