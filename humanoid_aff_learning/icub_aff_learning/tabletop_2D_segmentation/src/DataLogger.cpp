
//#include "BehaviorModule/DataLogger.h"

#include "tabletop_2D_segmentation/DataLogger.h"

using namespace std;

DataLogger::DataLogger(const char* dataType, const char* path, int count)
{
	itemId = 0;
	logType = string(dataType);
	savePath = string(path);	
	
	featureCount = count;
}

DataLogger::~DataLogger()
{
}

void DataLogger::logSingleData(FeatureTuple* singleTuple, int label)
{
	double *features = singleTuple->features;
	logSingleData(features,label);
}



// A very ugly implementation of image feature logging. Might be error-prone.
// label = 0 ==> init.
// label = 1 ==> final/effect
void DataLogger::logSingleData(ImageFeatureTuple *currentImgInfo, int experimentEpoch, int label)
{
	int i;
	stringstream rawS, filS, otherS;
	ofstream rawf, filf, otherf;
	
	//time(&timeReading);
	
	rawS << savePath <<  logType <<"_raw_" << experimentEpoch << "@" << label << ".dat";
	filS << savePath << logType <<"_filtered_" << experimentEpoch << "@" << label << ".dat";
	otherS << savePath << logType <<"_general_" << experimentEpoch << "@" << label << ".dat";
	 
	
	
	// Save raw image as a separate file
	rawf.open(rawS.str().c_str());	

	// Singular fields in image information
	// rawf << currentImgInfo->rawimg_header_seq << " " << currentImgInfo->rawimg_header_stamp << " " << currentImgInfo->rawimg_header_frame_id << " " << currentImgInfo->rawimg_height << " " << currentImgInfo->rawimg_width << " " << currentImgInfo->rawimg_step << " " << currentImgInfo->rawimg_is_bigendian << " " << currentImgInfo->rawimg_encoding;
	
	rawf << currentImgInfo->rawimg_height << " " << currentImgInfo->rawimg_width << " " << currentImgInfo->rawimg_step << " ";
	
	rawf << "\n";


	// the filtered data itself...
	for(i = 0; i < currentImgInfo->rawimg_height * currentImgInfo->rawimg_step; i++)
	{
		rawf << currentImgInfo->rawimg_data[i] << " ";
	}
	rawf << "\n";
	
	rawf.close();
	cout << "Logger @ " << timeReading << ": Raw image stored" << endl;
	
	// Save filtered image as a separate file
	
	filf.open(filS.str().c_str());

	// Singular fields in image information

	filf << currentImgInfo->filimg_height << " " << currentImgInfo->filimg_width << " " << currentImgInfo->filimg_step;
	
	filf << "\n";

	// the filtered data itself...
	for(i = 0; i < currentImgInfo->filimg_height * currentImgInfo->filimg_step; i++)
	{
		filf << currentImgInfo->filimg_data[i] << " ";
	}
	filf << "\n";
	
	
	filf.close();
	cout << "Logger @ " << timeReading << ": Filtered image stored" << endl;
	
	// Save the remaining information
	
	otherf.open(otherS.str().c_str());
	
	// single fields
	
	otherf << currentImgInfo->roi_width << " " << currentImgInfo->roi_height << " " << currentImgInfo->roi_x_offset << " " << currentImgInfo->roi_y_offset << " "<< currentImgInfo->ooi_area << " " << currentImgInfo->face_detected;
	otherf << " ";
	
	
	
	for(i = 0; i < 10; i++)
	{
		otherf << currentImgInfo->ooi_color_r_hist[i] << " ";
	}
	otherf << " ";
	
	for(i = 0; i < 10; i++)
	{
		otherf << currentImgInfo->ooi_color_g_hist[i] << " ";
	}
	otherf << " ";
	
	for(i = 0; i < 10; i++)
	{
		otherf << currentImgInfo->ooi_color_b_hist[i] << " ";
	}
	otherf << " ";
		
	otherf.close();
	
	cout << "Logger @ " << timeReading << ": Remaining information stored" << endl;
}


void DataLogger::logSingleData(double* features,int index,int label)
{	
	stringstream fileSS, svm_fileSS;
	
	int languageTag = (label > 0) ? label : 1;
	
	time(&timeReading);
	
	fileSS << savePath << logType.c_str() << "_" << index << "@" << timeReading << ".dat";
	svm_fileSS << savePath << logType.c_str() << "_" << index << "@" << timeReading << ".svm.data";
	
	ofstream rawfile, svmfile;
	
	rawfile.open ((fileSS.str()).c_str());
	svmfile.open((svm_fileSS.str()).c_str());
	
	rawfile << languageTag;
	svmfile << languageTag;
	
	for(int f = 0; f < featureCount; f++)
	{
	
		rawfile << " " << features[f];
		svmfile << " " << (f+1) << ":" << features[f] << " "; 
	
	}
	
	rawfile << "\n";
	svmfile << "\n";
	
	rawfile.close();
	svmfile.close();
	
	//itemId ++;
	
}

/*
void DataLogger::logSingleData(double* features,int index)
{	
	stringstream fileSS;
	
	time(&timeReading);
	
	fileSS << savePath << logType.c_str() << "_" << index << "@" << timeReading << ".dat";
	
	ofstream myfile;
	myfile.open ((fileSS.str()).c_str());
	
	for(int f = 0; f < featureCount; f++)
	{
	
		myfile << " " << features[f];
	
	}
	
	myfile << "\n";
	
	myfile.close();
	
	//itemId ++;
	
}
*/

void DataLogger::logAllData(FeatureTuple* tuples, int itemCount)
{
	double **items = new double*[itemCount];
	
	for(int i = 0; i < itemCount; i++)
	{
		items[i] = tuples[i].features;
	}
	
	logAllData(items,itemCount);
	
	for(int i = 0; i < itemCount; i++)
	{
		items[i] = NULL;
		delete items[i];
	}
	
	delete[] items;
	
}

void DataLogger::logAllData(double**items, int itemCount)
{
	stringstream fileSS;	
	time(&timeReading);
	
	fileSS << savePath << logType.c_str() << "s_@" << timeReading << ".dat";
	
	ofstream myfile;
	myfile.open ((fileSS.str()).c_str());
	
	for(int i = 0; i < itemCount; i++)
	{
		for(int f = 0; f < featureCount; f++)
		{
			myfile << " " << items[i][f];
		}
		
		myfile << "\n";
		
		itemId ++;
	}	
	
	myfile.close();	
	
}


void loadSampleData(FeatureTuple* tuples,int featureCount)
{
	ifstream myfile;
	myfile.open("sample_data.txt");
	
	double x;
	
	int tupleIndex = 0;
	
	while(tupleIndex < 2)
	{
		for(int i = 0; i < featureCount; i++)
		{
			myfile >> x;	
			cout << x << endl;
			
			tuples[tupleIndex].features[i] = x;
		}
		
		tupleIndex ++;
	}
	myfile.close();
	
	cout << "Out of the way" << endl;
	
}



/*
	Sample Usage:
	./DataLogger <save_path> <data_type> <featureCount>
	./DataLogger /home/onur/experimentData object 5
*/
/*
int main(int argc, char**argv)
{

	if(argc < 4)
	{
		cout << "Usage: ./DataLogger <save_path> <data_type> <featureCount>" << endl;
	
	}
	
	int featureCount = atoi(argv[3]); 
	
//	DataLogger *logger = new DataLogger(argv[1],argv[2],featureCount);
	
	DataLogger *logger = new DataLogger("object","./",5);
	
	FeatureTuple* sampleTuples = new FeatureTuple[2];
	
	sampleTuples[0].features = new double[5];
	sampleTuples[1].features = new double[5];
		
	loadSampleData(sampleTuples, 5);
	
	logger->logSingleData(&sampleTuples[0]);
	logger->logSingleData(&sampleTuples[1]);
	
	logger->logAllData(sampleTuples, 2);
	
	delete []sampleTuples[0].features;
	delete []sampleTuples[1].features;
	
	delete [] sampleTuples;
	
	delete logger;
	
	
	return 0;
}
*/




