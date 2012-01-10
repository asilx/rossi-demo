#ifndef DATALOGGER_H_
#define DATALOGGER_H_
#include<sstream>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <time.h>
#include <string>
#include <vector>
#include <stdexcept> 
#include <cstdio>     
#include <cstdlib>              // for getenv()
#include <list>                 // for std::list



typedef struct featureTuple
{
	double* features;
} FeatureTuple;



// This is implemented by both behavior manager and feature manager
class DataLogger
{
private:

	std::string logType;
	std::string savePath; // a common path is needed!
	long itemId;
	time_t timeReading;
	
	int featureCount;
	
public:
	DataLogger(const char* dataType, const char* path, int count); // additional featureCount
	~DataLogger();
	
	void logSingleData(double* features,int index, int label = -1);
	void logSingleData(FeatureTuple* singleTuple);
	
	void logAllData(double**items, int itemCount);
	void logAllData(FeatureTuple* tuples, int itemCount);

};
#endif 
