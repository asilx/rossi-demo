#ifndef DATALOGGER_H_
#define DATALOGGER_H_


#include<sstream>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <time.h>
#include <string>
#include <string.h>
#include <vector>
#include <stdexcept> 
#include <cstdio>     
#include <cstdlib>              // for getenv()
#include <list>                 // for std::list



typedef struct featureTuple
{
	double* features;
} FeatureTuple;

// An ugly struct
typedef struct imageFeatTuple
{	
	// raw image contents
	int rawimg_header_seq;
	time_t   rawimg_header_stamp;
	std::string rawing_header_frame_id;
	
	int rawimg_height;
	int rawimg_width;
	int rawimg_step;
	
	int rawimg_is_bigendian;
	int* rawimg_data;
	std::string rawimg_encoding;

	// filtered image contents
	
	int filimg_header_seq;
	time_t   filimg_header_stamp;
	std::string filing_header_frame_id;
	
	int filimg_height;
	int filimg_width;
	int filimg_step;
	
	int filimg_is_bigendian;
	int* filimg_data;
	std::string filimg_encoding;
	
	
	// other contents
	
	int roi_width;
	int roi_height;
	int roi_x_offset;
	int roi_y_offset;
	
	bool face_detected;
	int ooi_area;
	std::vector<int> ooi_color_r_hist;
	std::vector<int> ooi_color_g_hist;
	std::vector<int> ooi_color_b_hist;

} ImageFeatureTuple;

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
	void logSingleData(FeatureTuple* singleTuple, int label = -1);
	void logSingleData(ImageFeatureTuple *currentImgInfo, int experimentEpoch, int label);
	
	
	void logAllData(double**items, int itemCount);
	void logAllData(FeatureTuple* tuples, int itemCount);

};
#endif 
