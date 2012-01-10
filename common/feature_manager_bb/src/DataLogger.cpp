
#include "DataLogger.h"

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

void DataLogger::logSingleData(FeatureTuple* singleTuple)
{
	double *features = singleTuple->features;
	logSingleData(features,0);
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




