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

// An ugly struct
typedef struct imageFeatTuple
{	
	// raw image contents
	uint32 rawimg_header_seq;
	time   rawimg_header_stamp;
	string rawing_header_frame_id;
	
	uint32 rawimg_height;
	uint32 rawimg_width;
	uint32 rawimg_step;
	
	uint8 rawimg_is_bigendian;
	uint8* rawimg_data;
	string rawimg_encoding;

	// filtered image contents
	
	uint32 filimg_header_seq;
	time   filimg_header_stamp;
	string filing_header_frame_id;
	
	uint32 filimg_height;
	uint32 filimg_width;
	uint32 filimg_step;
	
	uint8 filimg_is_bigendian;
	uint8* filimg_data;
	string filimg_encoding;		
	
	
	// other contents
	
	int roi_width;
	int roi_height;
	int roi_x_offset;
	int roi_y_offset;
	
	bool face_detected;
	uint16 ooi_area;
	vector<uint8> ooi_color_r_hist;
	vector<uint8> ooi_color_g_hist;
	vector<uint8> ooi_color_b_hist;

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
	void logSingleData(ImageFeatureTuple *currentImgInfo, int label = 0);
	
	
	void logAllData(double**items, int itemCount);
	void logAllData(FeatureTuple* tuples, int itemCount);

};
#endif 
