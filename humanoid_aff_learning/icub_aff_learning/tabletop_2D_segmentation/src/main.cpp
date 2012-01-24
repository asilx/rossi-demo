//============================================================================
// Name        : Table Calibration Mono.cpp
// Author      : Yigit
// Version     :
// Copyright   : You dont have right to copy
// Description : Ansi-stylaaa yeah
//============================================================================


#include <stdio.h>
#include <signal.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <math.h>
#include <yarp/os/Network.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "cv_bridge/CvBridge.h"
#include "tabletop_2D_segmentation/Perception2D.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"

#include "/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/common/aff_msgs/msg_gen/cpp/include/aff_msgs/ExperimentState.h"
#include "/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/common/aff_msgs/msg_gen/cpp/include/aff_msgs/Features.h"

// ++Onur: inclusion for data logging

#include "tabletop_2D_segmentation/DataLogger.h"

// --Onur

const int N_BINS = 10;
const std::string WORKSPACE_2D_SEGMENTATION_SRV_NAME ="/tabletop_2D_segmentation";

YARP_DECLARE_DEVICES(icubmod);

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
//using namespace cv;
#define PI 3.1415926535898

struct Packet {
	IplImage* raw_image;
	IplImage* filtered_image;
	CvRect boundingBox;
	int segmented_area;

	bool faceDetected;
	CvHistogram* redHist;
	CvHistogram* blueHist;
	CvHistogram* greenHist;
};

bool first_srv_callback = false;

/*
 * ------------PROTOTYPES--------------
 */
 
void sighandler(int sig);
//Table Processing
int findEdgeColorForIdentification(CvPoint2D32f p1, CvPoint2D32f p2,IplImage* image);
int findEdgeColorForVerification(CvPoint2D32f p1, CvPoint2D32f p2,IplImage* image);
int* determineWays(CvPoint2D32f* singleCorners, int indexAndDegrees[][9],int index);
int findExtensionColor(CvPoint2D32f source, CvPoint2D32f destination,IplImage* image);
IplImage * ContrastBrightness(IplImage *src, int Contrast, int Brightness);
void tagOtherCorners(int* ways, int x);
void tagMainCorner(bool selections[], int verhor[], int x, int *ways);
void findAreasAndCorners(int indexAndDegrees[][9]);
void segmentAreaObjects();
void findSizeAndHistograms(int x,int y,IplImage *image);
void initializeForTagging3KnownCorners(int *ways,bool selections[],int verhor[],int x);
//Data Processing
void computeEffect();
void imageLoggingOp(ImageFeatureTuple &imgTuple, int region_id,sensor_msgs::Image::Ptr raw, sensor_msgs::Image::Ptr filtered);

//Communication-Services-Callbacks
bool perception2DCallback(tabletop_2D_segmentation::Perception2D::Request& request,tabletop_2D_segmentation::Perception2D::Response& response);
void expStateCallback(aff_msgs::ExperimentState::ConstPtr exp_state);
void percept(int region_id, tabletop_2D_segmentation::Perception2D::Request& request, tabletop_2D_segmentation::Perception2D::Response& response);

//Detection-Object Segmentation-Visualization of Images
void segmentTabletopObjects(int* confidenceCounter);
bool detect(IplImage* img, CvPoint* pt1, CvPoint *pt2);
void segmentObjectsNew(IplImage* bgr_frameLeft, bool forIdentification);

//Memory Issues-Initialization
void initializePacket(Packet *willBeSent);
void releaseMemory();
void initializeTheGlobalVars();
void initializeWays(int *ways,int x,int indexAndDegrees[][9]);

//Necessary Operations
int cvOverlayImage(IplImage* src, IplImage* overlay, CvPoint location,bool flag);
void drawWireFrame(IplImage *image, bool willBeFilled[][4], float aboveLimit,IplImage *colourImage);
float getEuclidianDistance(CvPoint2D32f p1, CvPoint2D32f p2);
int floodFill(int a, int b, int *minX, int *minY, int *maxX, int *maxY);
int getImageSize(IplImage *overlay);
bool floodFillCenterBased(int x, int y, int xPixelOffset, int yPixelOffset);
/*
 * -----------END OF PROTOTYPES-------------
 */

IplImage *img = NULL, *imgResult, *copyBgr, *Image2, *imgRed, *imgGreen,*imgBlue;
char fileName[100];
char positions[4][4][100];
char areas[3][3][100];
float homograpySource[32] = {
		100.0, 100.0,
		200.0, 100.0,
		300.0, 100.0,
		400.0, 100.0,
		100.0, 175.0,
		200.0, 175.0,
		300.0, 175.0,
		400.0, 175.0,
		100.0, 250.0,
		200.0, 250.0,
		300.0, 250.0,
		400.0, 250.0,
		100.0, 325.0,
		200.0, 325.0,
		300.0, 325.0,
		400.0, 325.0
};

float homograpyVisualization[32] = {
		60.0, 50.0,
		180.0, 50.0,
		300.0, 50.0,
		420.0, 50.0,
		45.0, 130.0,
		200.0, 130.0,
		305.0, 130.0,
		435.0, 130.0,
		30.0, 220.0,
		170.0, 220.0,
		310.0, 220.0,
		450.0, 220.0,
		15.0, 320.0,
		165.0, 320.0,
		315.0, 320.0,
		465.0, 320.0
};

CvPoint2D32f places[4][4];
bool detectedFace = false;
CvPoint* pt1, *pt2;
unsigned long total[3];
bool newFace = false;

PolyDriver *clientGaze;
PolyDriver *clientTorso;
IGazeControl *igaze;
IEncoders *ienc;
IPositionControl *ipos;
IPositionControl *iposGaze;

int diff, mean[3], temp[3];
CvScalar color, color1;
CvScalar value11;
float threshold = 100.0;//200

IplImage * hue,*sat,*sat1,*sat2, *sat3; // hue channel
IplImage * hue1; // Hue upper bound
IplImage * hue2; // Hue lower bound
IplImage * hue3; // hue color filtering
IplImage * hsvImage; // hsv image

int c = 0;
int d = 0;

BufferedPort<Bottle> in;
Bottle *inBot;

int* ways;
bool* identified;
int** identificationIndex;
bool willBeFilled[4][4];
int willBeFilledCountsTotal[4][4];
int willBeFilledCountsPerTurn[5][4][4];
CvPoint2D32f singleCornersPerTurn[5][4][4];
bool identifiedPerTurn[10][4][4];
CvPoint2D32f singleCornersMean[4][4];
int setOrReset[3][3];
int countForIdentified = 0;
CvPoint2D32f* singleCorners;
CvPoint2D32f* featuresPerTurn;
CvPoint2D32f* features;
CvPoint2D32f* newCornerFeatures;

int countForPixels;

yarp::sig::Vector angles(3);
yarp::sig::Vector headPos(3);
yarp::sig::Vector headAngles(4);
yarp::sig::Vector eyeAngles(3);
yarp::sig::Vector previousHeadPos(3);
yarp::sig::Vector previousHeadAngles(4);
yarp::sig::Vector previousEyeAngles(3);

BufferedPort<ImageOf<PixelBgr> > *portImgOut, *portDenemeOut, *portDenemeOut1;

// Create memory for calculations
static CvMemStorage* storage = 0;

// Create a new Haar classifier
static CvHaarClassifierCascade* cascade = 0;

// Create a string that contains the cascade name
//OUTconst char* cascade_name = "/home/kadir/interconnection/workspace/metu-ros-pkg/stacks/aff_learning/humanoid_aff_learning/icub_aff_learning/tabletop_2D_segmentation/bin/haarcascade_frontalface_alt.xml";
//const char* cascade_name = "/home/yigit/rosWorkspace/tabletop_2D_segmentation/bin/haarcascade_frontalface_alt.xml";
const char* cascade_name = "/home/asil/rossi_workspace/metu-ros-pkg/stacks/aff_learning/humanoid_aff_learning/icub_aff_learning/tabletop_2D_segmentation/bin/haarcascade_frontalface_alt.xml";


IplImage* images[9];
bool seenOrNot[9];
CvPoint* wireFramePoints = new CvPoint[9];

int locationAndSize[9][4];
Packet willBeSent;
Packet willBeSentOld;

int counter = 0;
IplImage* tempImg,*bgr_frameLeftCopy;
bool motionDone;
int perceptTrying;

int experimentEpoch = 0;


//++Kadir
sensor_msgs::CvBridge* bridge_;
ros::ServiceServer srv_perception_2D;
ros::Subscriber sub_exp_state_;
ros::Publisher pub_tabletop_2d_features_;
aff_msgs::ExperimentState exp_state_;
IplImage* raw_image;
IplImage* filtered_image;
IplImage *rawForDisappear;
IplImage *filteredForDisappear;
//OUTcv_bridge::CvImagePtr cv_raw_image_ptr;
ros::NodeHandle* nh;
int oldRegionId = -2;

//// ++Onur: Logging the retrieved information
DataLogger 	*imageLogger = new DataLogger("image","./",-1),*imageEffectLogger = new DataLogger("effect_image","./",-1); // TODO: be sure count is of no use or go for something other than -1
ImageFeatureTuple imgTuple, imgEffectTuple; // TODO delete/add if necessary
//--Onur

int countt = 0;
int countOfCorners = 0;
int corner_count = 600;
int corner_countPerTurn = 600;
int motion;
bool flag = true;
ImageOf<PixelBgr> *frameLeft;
CvFont font;
ImageOf<PixelBgr> imgOut, denemeOut, denemeOut1;
BufferedPort<ImageOf<PixelBgr> > imagePortLeft, imagePortRight;
BufferedPort<Bottle> motionSense;
IplImage* tempImgG,*bgr_frameLeftBrighted;
IplImage* tempImg1;
IplImage* tempImgForROI;
IplImage* wireFrame;
IplImage* tmp_image;
IplImage* eig_image;
IplImage *ROIGrayAndBinary;
IplImage *ROI;
IplImage *destination;
IplImage *maskImage;
IplImage *copyOfGlobBgr;

bool justForFeatureExtraction = true;

int main(int argc, char** argv) {

	experimentEpoch = atoi(argv[1]);	
	
	ros::init(argc, argv, "tabletop_2D_segmentation");
	nh = new ros::NodeHandle();

	srv_perception_2D = nh->advertiseService(WORKSPACE_2D_SEGMENTATION_SRV_NAME, perception2DCallback);

	sub_exp_state_ = nh->subscribe<aff_msgs::ExperimentState> ("/experiment_state", 10, expStateCallback);

	pub_tabletop_2d_features_ = nh->advertise<aff_msgs::Features>("/tabletop_2D_features", 10);

	YARP_REGISTER_DEVICES(icubmod);

	pt1 = new CvPoint;
	pt2 = new CvPoint;
	bridge_ = new sensor_msgs::CvBridge();

	portImgOut = new BufferedPort<ImageOf<PixelBgr> > ;

	Network::init();

	Property optGaze("(device gazecontrollerclient)");
	optGaze.put("remote", "/iKinGazeCtrl");
	optGaze.put("local", "/gaze_client");

	clientGaze = new PolyDriver;
	if (!clientGaze->open(optGaze)) {
		delete clientGaze;
		return false;
	}

	// open the view
	clientGaze->view(igaze);
	//clientGaze->view(iposGaze);

	// put the gaze in tracking mode, so that
	// when the torso moves, the gaze controller
	// will compensate for it
	igaze->setTrackingMode(false);
	igaze->setEyesTrajTime(1);
	igaze->setNeckTrajTime(1.2);

	motionSense.open("/ext_motion_detection:i");
	Network::connect("/ext_motion_detection:o", "/ext_motion_detection:i");

	portImgOut = new BufferedPort<ImageOf<PixelBgr> > ;
	portImgOut->open("/video/imageLeft/out");

	portDenemeOut = new BufferedPort<ImageOf<PixelBgr> > ;
	portDenemeOut->open("/video/imageDeneme/out");

	portDenemeOut1 = new BufferedPort<ImageOf<PixelBgr> > ;
	portDenemeOut1->open("/video/imageDeneme1/out");

	Network::connect("/video/imageLeft/out", "/ed");
	Network::connect("/video/imageDeneme/out", "/deneme");
	Network::connect("/video/imageDeneme1/out", "/deneme1");

	imagePortLeft.open("/video/imageLeft/in");
	Network::connect("/icub/camCalib/left/out", "/video/imageLeft/in");
	


	wireFramePoints[0].x = 280;
	wireFramePoints[0].y = 174;
	wireFramePoints[1].x = 527;
	wireFramePoints[1].y = 174;
	wireFramePoints[2].x = 780;
	wireFramePoints[2].y = 174;
	wireFramePoints[3].x = 254;
	wireFramePoints[3].y = 358;
	wireFramePoints[4].x = 525;
	wireFramePoints[4].y = 358;
	wireFramePoints[5].x = 788;
	wireFramePoints[5].y = 358;
	wireFramePoints[6].x = 217;
	wireFramePoints[6].y = 598;
	wireFramePoints[7].x = 515;
	wireFramePoints[7].y = 598;
	wireFramePoints[8].x = 825;
	wireFramePoints[8].y = 598;

	sprintf(positions[0][0], "Top Left Corner");
	sprintf(positions[0][1], "Top Left Side");
	sprintf(positions[0][2], "Top Right Side");
	sprintf(positions[0][3], "Top Right Corner");
	sprintf(positions[1][0], "Left Up Side");
	sprintf(positions[1][1], "Middle Up Left");
	sprintf(positions[1][2], "Middle Up Right");
	sprintf(positions[1][3], "Right Up Side");
	sprintf(positions[2][0], "Left Down Side");
	sprintf(positions[2][1], "Middle Down Left");
	sprintf(positions[2][2], "Middle Down Right");
	sprintf(positions[2][3], "Right Down Side");
	sprintf(positions[3][0], "Down Left Corner");
	sprintf(positions[3][1], "Down Left Side");
	sprintf(positions[3][2], "Down Right Side");
	sprintf(positions[3][3], "Down Right Corner");

	sprintf(areas[0][0], "Left Up Corner");
	sprintf(areas[0][1], "Top Middle");
	sprintf(areas[0][2], "Right Up Corner");
	sprintf(areas[1][0], "Left Side");
	sprintf(areas[1][1], "Middle");
	sprintf(areas[1][2], "Right Side");
	sprintf(areas[2][0], "Left Down Corner");
	sprintf(areas[2][1], "Down Middle");
	sprintf(areas[2][2], "Right Down Corner");

	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 0.5, CV_AA);
	imgResult = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	bgr_frameLeftBrighted = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
	bgr_frameLeftCopy = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	tempImgG = cvCreateImage(cvGetSize(imgResult), IPL_DEPTH_8U, 1);
	tempImg = cvCreateImage(cvGetSize(imgResult), IPL_DEPTH_8U, 3);
	tempImg1 = cvCreateImage(cvGetSize(imgResult), IPL_DEPTH_8U, 3);
	tempImgForROI = cvCreateImage(cvGetSize(imgResult), IPL_DEPTH_8U, 3);
	wireFrame = cvCreateImage(cvSize(1024, 768), IPL_DEPTH_8U, 3);

	initializeTheGlobalVars();
	initializePacket(&willBeSent);
	initializePacket(&willBeSentOld);

	igaze->getHeadPose(previousHeadPos,previousHeadAngles);
	igaze->getAngles(previousEyeAngles);
	int confidenceCounter = 0;
	float difference = 0.0;
	
	for(int a = 0; a < 4;a++)
		for(int b = 0; b < 4;b++)
		{
			willBeFilledCountsTotal[a][b] = 0;
			singleCornersMean[a][b].x = 0.0;
			singleCornersMean[a][b].y = 0.0;
		}
	
	for(int c = 0; c < 5;c++)	
		for(int a = 0; a < 4;a++)
			for(int b = 0; b < 4;b++)
			{
				willBeFilledCountsPerTurn[c][a][b] = 0;
				singleCornersPerTurn[c][a][b].x = 0.0;
				singleCornersPerTurn[c][a][b].y = 0.0;
				identifiedPerTurn[c][a][b] = false;
			}
	
	while (flag) {
		if (!nh->ok())
			break;

		//igaze->getHeadPose(headPos,headAngles);
		igaze->getAngles(eyeAngles);
		
		//printf("----------------------------------------------------\n");
		//printf("Head Position: X:%f, Y:%f, Z:%f\n",headPos[0], headPos[1],headPos[2]);
		//printf("Head Degree: XA:%f, YA:%f, ZA:%f, THETA:%f\n",headAngles[0], headAngles[1], headAngles[2], headAngles[3]);
		//printf("Eye Degree: AZ:%f, EL:%f, VE:%f\n",eyeAngles[0],eyeAngles[1],eyeAngles[2]);
		//printf("===================================================\n");
		//printf("Head Position Difference: X:%f, Y:%f, Z:%f\n",abs(headPos[0] - previousHeadPos[0]), abs(headPos[1] - previousHeadPos[1]),abs(headPos[2] - previousHeadPos[2]));
		//printf("Head Degree Difference: XA:%f, YA:%f, ZA:%f, THETA:%f\n",abs(headAngles[0] - previousHeadAngles[0]), abs(headAngles[1] - previousHeadAngles[1]), abs(headAngles[2] - previousHeadAngles[2]), abs(headAngles[3]- previousHeadAngles[3]));
		//printf("Eye Degree Difference: AZ:%f, EL:%f, VE:%f\n",abs(eyeAngles[0]- previousEyeAngles[0]),abs(eyeAngles[1]- previousEyeAngles[1]),abs(eyeAngles[2] - previousEyeAngles[2]));
		//printf("----------------------------------------------------\n");
		
		difference = abs(eyeAngles[0]- previousEyeAngles[0]) + abs(eyeAngles[1]- previousEyeAngles[1]);
		
		//for(int a = 0; a < 3;a++)
			//previousHeadPos[a] = headPos[a];
			
		//for(int a = 0; a < 4;a++)
		//	previousHeadAngles[a] = headAngles[a];
			
		for(int a = 0; a < 3;a++)
			previousEyeAngles[a] = eyeAngles[a];
			
		
		
		//printf("Difference:%f\n",difference);
		if(difference < 0.1)	
		{
			confidenceCounter++;
		}
		else
		{
			confidenceCounter = 0;
			for(int a = 0; a < 4;a++)
				for(int b = 0; b < 4;b++)
				{
					willBeFilledCountsTotal[a][b] = 0;
					singleCornersMean[a][b].x = 0.0;
					singleCornersMean[a][b].y = 0.0;
				}
				
			for(int c = 0; c < 5;c++)	
				for(int a = 0; a < 4;a++)
					for(int b = 0; b < 4;b++)
					{
						willBeFilledCountsPerTurn[c][a][b] = 0;
						singleCornersPerTurn[c][a][b].x = 0.0;
						singleCornersPerTurn[c][a][b].y = 0.0;
					}
			
		}
		segmentTabletopObjects(&confidenceCounter);		
		printf("confidenceCounter:%d\n",confidenceCounter);
		
		ros::spinOnce();
	}
}

float getEuclidianDistance(CvPoint2D32f p1, CvPoint2D32f p2) {
	return sqrt(pow((p1.y - p2.y), 2) + pow((p1.x - p2.x), 2));
}
int findExtensionColor(CvPoint2D32f source, CvPoint2D32f destination,IplImage* image) {

	CvPoint2D32f newPoint;
	double a = source.y - destination.y;
	double b = destination.x - source.x;
	double c = destination.y * source.x - source.y * source.x + source.x
			* source.y - source.y * destination.x;
	double xNew;
	if (destination.x > source.x) {
		xNew = destination.x +  (destination.x - source.x) / 3;//2x/5
		if (xNew < 640) {
			newPoint.x = xNew;
		} else {
			return -2;
		}

	} else {
		xNew = destination.x - (source.x - destination.x) / 3; //2x/5
		if (xNew > 0) {
			newPoint.x = xNew;
		} else {
			return -2;
		}

	}

	newPoint.y = (-1 * c - a * xNew) / b;
	if (newPoint.y < 480 && newPoint.y >= 0)
	{
		return findEdgeColorForIdentification(destination, newPoint, image);
	}
	else {
		return -2;
	}
}
int findEdgeColorForVerification(CvPoint2D32f p1, CvPoint2D32f p2,IplImage* image) {
	int interval = 100;
	int counter = 0;
	double deltaX = (p2.x - p1.x) / interval;
	double deltaY = (p2.y - p1.y) / interval;
	CvScalar ptr;
	for (int i = 0; i < interval; i++) {
		if (p1.y + i * deltaY < image->height && p1.x + i * deltaX < image->width) {
			if (p1.y + i * deltaY < image->height && p1.y + i * deltaY > 0 && p1.x + i * deltaX < image->width && p1.x + i * deltaX > 0) {
				ptr = cvGet2D(image, p1.y + i * deltaY, p1.x + i * deltaX);
				if (ptr.val[0] == 255.0) {
					counter++;
				}
			}

		}

	}
	if (interval - counter < 15)
		return 1;
	else
		return 0;

}
int findEdgeColorForIdentification(CvPoint2D32f p1, CvPoint2D32f p2, IplImage* image) {

	int interval = 100;
	int counter = 0;
	double deltaX = (p2.x - p1.x) / interval;
	double deltaY = (p2.y - p1.y) / interval;
	CvScalar ptr;
	for (int i = 0; i < interval; i++) {
		if (p1.y + i * deltaY < image->height && p1.x + i * deltaX < image->width) {
			if (p1.y + i * deltaY < image->height && p1.y + i * deltaY > 0 && p1.x + i * deltaX < image->width && p1.x + i * deltaX	> 0) {
				ptr = cvGet2D(image, p1.y + i * deltaY, p1.x + i * deltaX);
				if (ptr.val[0] == 255.0) {
					counter++;
				}
			}

		}

	}

	int noOfBlack = interval - counter;
	int noOfWhite = counter;
	if (noOfWhite > 50)
		return 1;
	else if (noOfBlack > 80)
		return 0;
	else
		return -1;
}
// 0->left, 1->down, 2->right, 3->up
int* determineWays(CvPoint2D32f* singleCorners, int indexAndDegrees[][9],int index) {
	int* ways = new int[4];
	double distance = 0;
	double minHor = 10000;
	int minHorIndex = -1;
	double minVer = 10000;
	int maxHorIndex = -1;
	double maxHor = 0;
	int maxVerIndex = -1;
	double maxVer = 0;
	int minVerIndex = -1;

	for (int x = 0; x < indexAndDegrees[index][0]; x++) {
		distance = singleCorners[indexAndDegrees[index][2 * x + 1]].y - singleCorners[index].y;
		if (distance > maxVer) {
			maxVer = distance;
			maxVerIndex = indexAndDegrees[index][2 * x + 1];
		}
		if (distance < minVer) {
			minVer = distance;
			minVerIndex = indexAndDegrees[index][2 * x + 1];
		}

	}
	ways[1] = maxVerIndex;
	ways[3] = minVerIndex;
	for (int x = 0; x < indexAndDegrees[index][0]; x++) {
		distance = singleCorners[indexAndDegrees[index][2 * x + 1]].x - singleCorners[index].x;
		if (distance > maxHor) {
			maxHor = distance;
			maxHorIndex = indexAndDegrees[index][2 * x + 1];
		}
		if (distance < minHor) {
			minHor = distance;
			minHorIndex = indexAndDegrees[index][2 * x + 1];
		}

	}
	ways[0] = minHorIndex;
	ways[2] = maxHorIndex;
	return ways;
}
void tagMainCorner(bool selections[], int verhor[], int x, int* ways) {
	if (selections[0] && selections[1]) {
		identificationIndex[x][0] = 2 - verhor[0];
		identificationIndex[x][1] = 1 + verhor[1];
	} else if (selections[1] && selections[2]) {
		identificationIndex[x][0] = 2 - verhor[0];
		identificationIndex[x][1] = 2 - verhor[1];
	} else if (selections[2] && selections[3]) {
		identificationIndex[x][0] = 1 + verhor[0];
		identificationIndex[x][1] = 2 - verhor[1];
	} else if (selections[3] && selections[0]) {
		identificationIndex[x][0] = 1 + verhor[0];
		identificationIndex[x][1] = 1 + verhor[1];
	}
	identified[x] = true;
	willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]] = true;
	places[identificationIndex[x][0]][identificationIndex[x][1]] = singleCorners[x];
	countForIdentified++;
	tagOtherCorners(ways, x);
}
void tagOtherCorners(int* ways, int x) {

	if (ways[0] != -1) {
		identificationIndex[ways[0]][0] = identificationIndex[x][0];
		identificationIndex[ways[0]][1] = identificationIndex[x][1] - 1;
	}
	if (ways[1] != -1) {
		identificationIndex[ways[1]][0] = identificationIndex[x][0] + 1;
		identificationIndex[ways[1]][1] = identificationIndex[x][1];
	}
	if (ways[2] != -1) {
		identificationIndex[ways[2]][0] = identificationIndex[x][0];
		identificationIndex[ways[2]][1] = identificationIndex[x][1] + 1;
	}

	if (ways[3] != -1) {
		identificationIndex[ways[3]][0] = identificationIndex[x][0] - 1;
		identificationIndex[ways[3]][1] = identificationIndex[x][1];
	}

	for (int a = 0; a < 4; a++) {

		if (ways[a] != -1 && !identified[ways[a]]
				&& identificationIndex[ways[a]][0] >= 0
				&& identificationIndex[ways[a]][0] < 4
				&& identificationIndex[ways[a]][1] >= 0
				&& identificationIndex[ways[a]][1] < 4) {
			identified[ways[a]] = true;
			willBeFilled[identificationIndex[ways[a]][0]][identificationIndex[ways[a]][1]]
					= true;
			places[identificationIndex[ways[a]][0]][identificationIndex[ways[a]][1]]
					= singleCorners[ways[a]];
			countForIdentified++;
		}

	}

}
void drawWireFrame(IplImage *image, bool willBeFilled[][4], float aboveLimit, IplImage *colourImage)//,int motion)
{

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 0.5, CV_AA);
	cvZero(image);
	cvDrawLine(image, cvPoint(175, 101), cvPoint(875, 101), cvScalar(255, 255,255), 2);
	cvDrawLine(image, cvPoint(875, 101), cvPoint(990, 733), cvScalar(255, 255,255), 2);
	cvDrawLine(image, cvPoint(990, 733), cvPoint(37, 733), cvScalar(255, 255,255), 2);
	cvDrawLine(image, cvPoint(37, 733), cvPoint(175, 101), cvScalar(255, 255,255), 2);
	cvDrawLine(image, cvPoint(142, 254), cvPoint(902, 254), cvScalar(255, 255,255), 2);
	cvDrawLine(image, cvPoint(95, 470), cvPoint(941, 470), cvScalar(255, 255,255), 2);
	cvDrawLine(image, cvPoint(399, 101), cvPoint(344, 733), cvScalar(255, 255,255), 2);
	cvDrawLine(image, cvPoint(649, 101), cvPoint(681, 733), cvScalar(255, 255,255), 2);

	if (aboveLimit < -26) {
		if (detectedFace == true) {

			cvOverlayImage(image, destination, cvPoint(image->width / 2 - destination->width / 2, 0), true);
		} else
			cvPutText(image, "Face Not Detected", cvPoint(10, 30), &font, cvScalar(0, 255, 0, 0));

		//for (int x = 0; x < 3; x++) {
			//for (int y = 0; y < 3; y++) {

				//if ((willBeFilled[x][y] && willBeFilled[x + 1][y + 1]) || seenOrNot[y + x * 3]) {
					//if (images[y + x * 3] != NULL && (setOrReset[x][y] == 1 || setOrReset[x][y] == 0)) {
						//if ((x != 2 && setOrReset[x + 1][y] != 1) || x == 2)
							//findSizeAndHistograms(x,y,image);
					//}
				//} else if ((willBeFilled[x][y + 1] && willBeFilled[x + 1][y]) || seenOrNot[y + x * 3]) {
					//if (images[y + x * 3] != NULL && (setOrReset[x][y] == 1 || setOrReset[x][y] == 0)) {
						//if ((x != 2 && setOrReset[x + 1][y] != 1) || x == 2)
							//findSizeAndHistograms(x,y,image);
					//}

				//}

			//}
		//}
	} else if (aboveLimit > -26 && detect(colourImage, pt1, pt2)) {
		cout << "detected" << endl;
		if (pt2->x > 0 && pt2->y > 0) {
			if (ROI != NULL) {
				cvReleaseImage(&ROI);
				ROI = NULL;
			}
			ROI = cvCreateImage(cvSize(pt2->x, pt2->y), IPL_DEPTH_8U, 3);
			for (int a = 0; a < ROI->width; a++) {
				for (int b = 0; b < ROI->height; b++) {
					if (pt1->y + b < colourImage->height && pt1->y + b > 0 && pt1->x + a < colourImage->width && pt1->x + a> 0) {
						CvScalar over = cvGet2D(colourImage, pt1->y + b, pt1->x+ a);
						if (b < ROI->height && b > 0 && a < ROI->width && a > 0) {
							cvSet2D(ROI, b, a, over);
						}
					}
				}
			}

			if ((int) ((ROI->width * 50) / 100) > 0 && (int) ((ROI->height * 50) / 100) > 0) {
				if (destination != NULL) {
					cvReleaseImage(&destination);
					destination = NULL;
				}
				destination = cvCreateImage(cvSize(75, 75), ROI->depth, ROI->nChannels);
				cvResize(ROI, destination);
				cvOverlayImage(image, destination, cvPoint(image->width / 2 - destination->width / 2, 0), true);
				detectedFace = true;

			}
		}

		//if (willBeFilled != NULL) {
			//for (int x = 0; x < 3; x++) {
				//for (int y = 0; y < 3; y++) {
					//if ((willBeFilled[x][y] && willBeFilled[x + 1][y + 1]) || seenOrNot[y + x * 3]) {
						//if (images[y + x * 3] != NULL && (setOrReset[x][y] == 1 || setOrReset[x][y] == 0)) {
							//if ((x != 2 && setOrReset[x + 1][y] != 1) || x == 2)
								//findSizeAndHistograms(x,y,image);
						//}
					//} else if ((willBeFilled[x][y + 1] && willBeFilled[x + 1][y]) || seenOrNot[y + x * 3]) {
						//if (images[y + x * 3] != NULL && (setOrReset[x][y] == 1 || setOrReset[x][y] == 0)) {
							//if ((x != 2 && setOrReset[x + 1][y] != 1) || x == 2)
								//findSizeAndHistograms(x,y,image);
						//}
					//}

				//}
			//}
		//}

	} else if (aboveLimit > -26 && !detect(colourImage, pt1, pt2)) {
		cout << "not detected" << endl;
		pt1->x = -1;
		pt1->y = -1;
		pt2->x = -1;
		pt2->y = -1;
		cvPutText(image, "Face Not Detected", cvPoint(10, 30), &font, cvScalar(
				0, 255, 0, 0));
		detectedFace = false;
		//if (willBeFilled != NULL) {
			//for (int x = 0; x < 3; x++) {
				//for (int y = 0; y < 3; y++) {
					//if ((willBeFilled[x][y] && willBeFilled[x + 1][y + 1]) || seenOrNot[y + x * 3]) {
						//if (images[y + x * 3] != NULL && (setOrReset[x][y] == 1	|| setOrReset[x][y] == 0)) {
							//if ((x != 2 && setOrReset[x + 1][y] != 1) || x == 2)
								//findSizeAndHistograms(x,y,image);
						//}
					//} else if ((willBeFilled[x][y + 1] && willBeFilled[x + 1][y]) || seenOrNot[y + x * 3]) {
						//if (images[y + x * 3] != NULL && (setOrReset[x][y] == 1 || setOrReset[x][y] == 0)) {
							//if ((x != 2 && setOrReset[x + 1][y] != 1) || x == 2)
								//findSizeAndHistograms(x,y,image);
						//}

					//}

				//}
			//}
		//}

	}

	willBeSent.faceDetected = detectedFace;
}
//Function to detect and draw any faces that is present in an image
bool detect(IplImage* img, CvPoint* pt1, CvPoint *pt2) {
	bool detected = false;

	// Allocate the memory storage
	storage = cvCreateMemStorage(0);

	// Load the HaarClassifierCascade
	cascade = (CvHaarClassifierCascade*) cvLoad(cascade_name, 0, 0, 0);
	// Create two points to represent the face locations
	int i;

	// Clear the memory storage which was used before
	cvClearMemStorage(storage);
	// Find whether the cascade is loaded, to find the faces. If yes, then:
	if (cascade) {

		// There can be more than one face in an image. So create a growable sequence of faces.
		// Detect the objects and store them in the sequence
		CvSeq* faces = cvHaarDetectObjects(img, cascade, storage, 1.1, 3,
				CV_HAAR_DO_CANNY_PRUNING, cvSize(10, 10));

		// Loop the number of faces found.
		for (i = 0; i < faces->total; i++) {
			// Create a new rectangle for drawing the face
			CvRect* r = (CvRect*) cvGetSeqElem(faces, i);
			detected = true;
			// Find the dimensions of the face,and scale it if necessary
			pt1->x = r->x;
			pt1->y = r->y;
			pt2->x = r->width;
			pt2->y = r->height;

			//cvRectangle( img, pt2, pt1, CV_RGB(255,255,255), 3, 8, 0 );
			r = NULL;

		}
		faces = NULL;
	}
	return detected;

}
int cvOverlayImage(IplImage* src, IplImage* overlay, CvPoint location, bool flag) {
	int sizeOfObject = 0;
	for (int x = 0; x < overlay->width; x++) {

		for (int y = 0; y < overlay->height; y++) {

			if (y < overlay->height && y > 0 && x < overlay->width && x > 0) {
				CvScalar over = cvGet2D(overlay, y, x);
				if (y + location.y < src->height && y + location.y > 0 && x + location.x < src->width && x + location.x > 0 && flag) {
					cvSet2D(src, y + location.y, x + location.x, over);
				} else if (y + location.y < src->height && y + location.y > 0 && x + location.x < src->width && x + location.x > 0 && !flag) {
					if (over.val[0] > 5.0 && over.val[0] < 250.0) {
						cvSet2D(src, y + location.y - overlay->height / 2, x + location.x - overlay->width / 2, over);
						sizeOfObject++;
					}

				}
			}
		}
	}
	return sizeOfObject;
}
int floodFill(int a, int b, int *minX, int *minY, int *maxX, int *maxY) {

	if ((a < copyBgr->width - 2) && (a > 1) && (b < copyBgr->height - 2) && (b > 1)) {
		CvScalar color1 = cvGet2D(copyBgr, b, a);
		if ((color1.val[0] != 255 || color1.val[1] != 255 || color1.val[2] != 255)) {
			if (maskImage != NULL) {
				cvReleaseImage(&maskImage);
				maskImage = NULL;
			}
			maskImage = cvCreateImage(cvSize(copyBgr->width + 2, copyBgr->height + 2), copyBgr->depth, 1);
			cvZero(maskImage);
			cvFloodFill(copyBgr, cvPoint(a, b), cvScalarAll(255), cvScalarAll(20), cvScalarAll(20), NULL, (255 << 8) +CV_FLOODFILL_MASK_ONLY + 4, maskImage);
			cvSmooth(maskImage, maskImage, CV_MEDIAN, 9, 9, 0, 0);

			for (int x = 4; x < maskImage->width - 4; x++) {
				for (int y = 4; y < maskImage->height - 4; y++) {
					color1 = cvGet2D(maskImage, y, x);
					if (color1.val[0] == 255) {
						*minX = x - 1;
						x = maskImage->width;
						y = maskImage->height;
					}
				}
			}

			for (int x = maskImage->width - 4; x >= 4; x--) {
				for (int y = maskImage->height - 4; y >= 4; y--) {

					color1 = cvGet2D(maskImage, y, x);
					if (color1.val[0] == 255) {
						*maxX = x - 1;
						x = 0;
						y = 0;
					}
				}
			}
			//From up
			for (int y = 4; y < maskImage->height - 4; y++) {
				for (int x = 4; x < maskImage->width - 4; x++) {
					color1 = cvGet2D(maskImage, y, x);
					if (color1.val[0] == 255) {
						*minY = y - 1;
						x = maskImage->width;
						y = maskImage->height;
					}
				}
			}
			//From down
			for (int y = maskImage->height - 4; y >= 4; y--) {
				for (int x = maskImage->width - 4; x >= 4; x--) {
					color1 = cvGet2D(maskImage, y, x);
					if (color1.val[0] == 255) {
						*maxY = y - 1;
						x = 0;
						y = 0;
					}
				}
			}
			cvReleaseImage(&maskImage);
			maskImage = NULL;
			return 3;
		} else {
			return -1;
		}
	} else
		return -2;

}
void initializePacket(Packet *willBeSent) {

	int numBins = N_BINS;
	float range[] = { 0, 255 };
	float *ranges[] = { range };

	
	willBeSent->blueHist = cvCreateHist(1, &numBins, CV_HIST_ARRAY,ranges, 1);
	willBeSent->redHist = cvCreateHist(1, &numBins, CV_HIST_ARRAY,ranges, 1);
	willBeSent->greenHist = cvCreateHist(1, &numBins, CV_HIST_ARRAY,ranges, 1);
	cvClearHist(willBeSent->redHist);
	cvClearHist(willBeSent->greenHist);
	cvClearHist(willBeSent->blueHist);
	
	willBeSent->boundingBox = cvRect(0, 0, 0, 0);
	
	willBeSent->segmented_area = -1;

	willBeSent->raw_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
	willBeSent->filtered_image = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U,3);
	willBeSent->faceDetected = false;

}
void initializeTheGlobalVars() {

	for (int x = 0; x < 3; x++) {
		for (int y = 0; y < 3; y++) {
			setOrReset[x][y] = -1;
		}
	}
	for (int x = 0; x < 9; x++) {
		images[x] = NULL;
	}
	for (int x = 0; x < 9; x++) {
		seenOrNot[x] = false;
	}

	rawForDisappear = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
	filteredForDisappear = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
	copyOfGlobBgr = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

}
void releaseMemory() {

	if (features != NULL) {
		delete[] features;
		features = NULL;
	}

	if (newCornerFeatures != NULL) {
		delete[] newCornerFeatures;
		newCornerFeatures = NULL;
	}

	if (singleCorners != NULL) {
		delete[] singleCorners;
		singleCorners = NULL;
	}

	for (int i = 0; i < counter; i++) {
		if (identificationIndex[i] != NULL) {
			delete[] identificationIndex[i];
			identificationIndex[i] = NULL;
		}

	}

	if (identificationIndex != NULL) {
		delete[] identificationIndex;
		identificationIndex = NULL;
	}

	if (identified != NULL) {
		delete[] identified;
		identified = NULL;
	}

	//	cout<<"hiiiii"<<endl;
	if (newCornerFeatures != NULL) {
		delete[] newCornerFeatures;
		newCornerFeatures = NULL;
	}

	if (singleCorners != NULL) {
		delete[] singleCorners;
		singleCorners = NULL;
	}

}
bool floodFillCenterBased(int x, int y, int xPixelOffset, int yPixelOffset) {
	
	if (Image2 != NULL) {
		cvReleaseImage(&Image2);
		Image2 = NULL;
	}
	Image2 = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 3);

	cvZero(Image2);
	int result = 0;
	int minY, maxY, minX, maxX;
	minY = 480, maxY = 0, minX = 640, maxX = 0;
	total[0] = 0;
	total[1] = 0;
	total[2] = 0;
	diff = 0;
	countForPixels = 0;
	cvZero(Image2);
	result = floodFill(copyBgr->width / 2 + xPixelOffset, copyBgr->height / 2 + yPixelOffset, &minX, &minY, &maxX, &maxY);

	if (result == 3) {
		if (minX < copyBgr->width && minX >= 0 && maxX < copyBgr->width && maxX >= 0 && minY < copyBgr->height && minY >= 0 && maxY < copyBgr->height && maxY >= 0) {
			
			IplImage *image = cvCreateImage(cvSize(maxX - minX, maxY - minY),copyBgr->depth, copyBgr->nChannels);

			willBeSent.boundingBox = cvRect(minX, minY, maxX- minX, maxY - minY);
			
			cvSetImageROI(copyBgr, cvRect(minX, minY, maxX - minX, maxY - minY));
			if (cvGetImageROI(copyBgr).width == maxX - minX && cvGetImageROI(copyBgr).height == maxY - minY) 
				cvCopy(copyBgr, image);
			cvResetImageROI(copyBgr);
			willBeSent.segmented_area= getImageSize(image);
			cvClearHist(willBeSent.blueHist);
			cvClearHist(willBeSent.redHist);
			cvClearHist(willBeSent.greenHist);
			imgRed = cvCreateImage(cvGetSize(image), 8, 1);
			imgGreen = cvCreateImage(cvGetSize(image), 8, 1);
			imgBlue = cvCreateImage(cvGetSize(image), 8, 1);

			cvSplit(image, imgBlue, imgGreen, imgRed, NULL);

			cvCalcHist(&imgRed, willBeSent.redHist, 0, 0);
			cvCalcHist(&imgGreen, willBeSent.greenHist, 0, 0);
			cvCalcHist(&imgBlue, willBeSent.blueHist, 0, 0);

			cvReleaseImage(&imgRed);
			cvReleaseImage(&imgGreen);
			cvReleaseImage(&imgBlue);
			cvReleaseImage(&image);
			imgRed = NULL;
			imgGreen = NULL;
			imgBlue = NULL;
			return true;
		} else
			return false;

	} else if (result == -1 || result == -2) {
		return false;
	}
	else
		return false;
}
//void initializeForTagging2KnownCorners(ways,selections,verhor)
//{
//	int control[2];
//	int indexesEmpty[2];
//	int indexesFull[2];
//	int counterE = 0,counterF = 0;
//	for(int a = 0; a < 4; a++)
//	{
//		if(ways[a] != -1)
//		{
//			indexesFull[counterF] = a;
//			counterF++;
//		}
//		else
//		{
//			indexesEmpty[counterE] = a;
//			counterE++;
//		}
//	}
//
//	control[0] = findExtensionColor(singleCorners[ways[indexesFull[0]]], singleCorners[x], imgResult);
//	control[1] = findExtensionColor(singleCorners[ways[indexesFull[1]]],singleCorners[x], imgResult);
//
//}
int getImageSize(IplImage *overlay) {
	int sizeOfObject = 0;
	for (int x = 0; x < overlay->width; x++) {
		for (int y = 0; y < overlay->height; y++) {

			if (y < overlay->height && y > 0 && x < overlay->width && x > 0) {
				CvScalar over = cvGet2D(overlay, y, x);
				if (over.val[0] > 5.0 && over.val[0] < 250.0) {
					sizeOfObject++;
				}
			}
			//cvSet2D(src, y+location.y - overlay->height/2, x+location.x- overlay->width/2, over);
		}
	}

	return sizeOfObject;
}
void findAreasAndCorners(int indexAndDegrees[][9]) {

	////Look for 4 neighboured corners
	for (int x = 0; x < counter; x++) {
		if (indexAndDegrees[x][0] == 4 && !identified[x]) {
			ways = determineWays(singleCorners, indexAndDegrees, x);
			bool leftSelected = false;
			bool rightSelected = false;
			bool upSelected = false;
			bool downSelected = false;
			int horizontal;
			int vertical;
			if (abs(singleCorners[ways[0]].x - 320) < abs( singleCorners[ways[2]].x - 320)) {
				leftSelected = true;
				horizontal = findExtensionColor(singleCorners[x],singleCorners[ways[0]], imgResult);
				cout<<"leftSelected"<<endl;
			} else {
				rightSelected = true;
				horizontal = findExtensionColor(singleCorners[x],singleCorners[ways[2]], imgResult);
				cout<<"rightSelected"<<endl;
			}
			if (abs(singleCorners[ways[1]].y - 240) < abs( singleCorners[ways[3]].y - 240)) {
				downSelected = true;
				vertical = findExtensionColor(singleCorners[x], singleCorners[ways[1]], imgResult);
				cout<<"downSelected"<<endl;
			} else {
				upSelected = true;
				vertical = findExtensionColor(singleCorners[x],singleCorners[ways[3]], imgResult);
				cout<<"upSelected"<<endl;
			}

			bool selections[4];
			selections[0] = leftSelected;
			selections[1] = downSelected;
			selections[2] = rightSelected;
			selections[3] = upSelected;
			int verhor[2];
			verhor[0] = vertical;
			verhor[1] = horizontal;
			if (verhor[0] >= 0 && verhor[1] >= 0) {
				tagMainCorner(selections, verhor, x, ways);
			}

			if (ways != NULL) {
				delete[] ways;
				ways = NULL;
			}
		}

	}
	
	for (int g = 0; g < counter; g++) {
		if (!identified[g]) {

			for (int y = 0; y < indexAndDegrees[g][0]; y++) {
				if (identified[indexAndDegrees[g][2 * y + 1]]) {
					if ((indexAndDegrees[g][2 * y + 2] <= 225) && (indexAndDegrees[g][2 * y + 2] > 135)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y+ 1]][0];
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1] - 1;

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}

					} else if ((indexAndDegrees[g][2 * y + 2] > 315) || (indexAndDegrees[g][2 * y + 2] <= 45)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0];
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1] + 1;

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					} else if ((indexAndDegrees[g][2 * y + 2] <= 315) && (indexAndDegrees[g][2 * y + 2] > 225)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0] - 1;
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1];

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					} else if ((indexAndDegrees[g][2 * y + 2] <= 135) && (indexAndDegrees[g][2 * y + 2] > 45)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0] + 1;
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1];

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					}
				}
			}
		}
	}

	//////Look for 3 neighboured Corners
	for (int x = 0; x < counter; x++) {
		if (indexAndDegrees[x][0] == 3 && !identified[x]) {

			int *ways = new int[4];
			initializeWays(ways,x,indexAndDegrees);
			bool selections[4];
			selections[0] = false;
			selections[1] = false;
			selections[2] = false;
			selections[3] = false;
			int verhor[2];
			verhor[0] = false;
			verhor[1] = false;

			initializeForTagging3KnownCorners(ways,selections,verhor,x);

			if (ways != NULL) {
				delete[] ways;
				ways = NULL;
			}
		}
	}
	
	for (int g = 0; g < counter; g++) {
		if (!identified[g]) {

			for (int y = 0; y < indexAndDegrees[g][0]; y++) {
				if (identified[indexAndDegrees[g][2 * y + 1]]) {
					if ((indexAndDegrees[g][2 * y + 2] <= 225) && (indexAndDegrees[g][2 * y + 2] > 135)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y+ 1]][0];
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1] - 1;

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}

					} else if ((indexAndDegrees[g][2 * y + 2] > 315) || (indexAndDegrees[g][2 * y + 2] <= 45)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0];
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1] + 1;

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					} else if ((indexAndDegrees[g][2 * y + 2] <= 315) && (indexAndDegrees[g][2 * y + 2] > 225)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0] - 1;
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1];

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					} else if ((indexAndDegrees[g][2 * y + 2] <= 135) && (indexAndDegrees[g][2 * y + 2] > 45)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0] + 1;
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1];

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					}
				}
			}
		}
	}

	for (int x = 0; x < counter; x++) {
		if (indexAndDegrees[x][0] == 2 && !identified[x]) {
			//cvCopyImage(bgr_frameLeftCopy,copyOfGlobBgr);
			
			//cvCircle(copyOfGlobBgr, cvPoint(singleCorners[x].x, singleCorners[x].y), 4, cvScalar(255, 255, 0), -1, 8);
			
			
			//cvShowImage("SingleCorners",copyOfGlobBgr);
			//cvWaitKey();
			int *ways = new int[4];
			initializeWays(ways,x,indexAndDegrees);

			bool selections[4];
			selections[0] = false;
			selections[1] = false;
			selections[2] = false;
			selections[3] = false;
			int verhor[2];
			verhor[0] = -1;
			verhor[1] = -1;
			int control[2];

			if (ways[3] == -1 && ways[2] == -1) {
				cout<<"Yukari ve sag bos"<<endl;
				control[0] = findExtensionColor(singleCorners[ways[1]],
						singleCorners[x], imgResult);
				control[1] = findExtensionColor(singleCorners[ways[0]],
						singleCorners[x], imgResult);

				if (control[0] == 0 && control[1] == 0) {
					identificationIndex[x][0] = 0;
					identificationIndex[x][1] = 3;
					identified[x] = true;
					willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
							= true;
					places[identificationIndex[x][0]][identificationIndex[x][1]]
							= singleCorners[x];
				} else if (control[0] == 0 && control[1] == 1) {

					int horizontal = findExtensionColor(singleCorners[x],
							singleCorners[ways[0]], imgResult);
					if (horizontal == 1) {
						identificationIndex[x][0] = 0;
						identificationIndex[x][1] = 2;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					} else if (horizontal == 0) {
						identificationIndex[x][0] = 0;
						identificationIndex[x][1] = 1;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					}

				} else if (control[0] == 1 && control[1] == 0) {
					int vertical = findExtensionColor(singleCorners[x],
							singleCorners[ways[1]], imgResult);
					if (vertical == 1) {
						identificationIndex[x][0] = 1;
						identificationIndex[x][1] = 3;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					} else if (vertical == 0) {
						identificationIndex[x][0] = 2;
						identificationIndex[x][1] = 3;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					}
				} else if (control[0] == 1 && control[1] == 1) {
					selections[1] = true;
					selections[0] = true;
					verhor[0] = findExtensionColor(singleCorners[x],
							singleCorners[ways[1]], imgResult);
					verhor[1] = findExtensionColor(singleCorners[x],
							singleCorners[ways[0]], imgResult);

					if (verhor[0] != -2 && verhor[1] != -2 && verhor[0] != -1
							&& verhor[1] != -1) {
						tagMainCorner(selections, verhor, x, ways);
					}
				}

			} else if (ways[0] == -1 && ways[3] == -1) {
				cout<<"Sol ve ust bos"<<endl;
				//cout<<"0 ve 3 yok"<<endl;
				control[0] = findExtensionColor(singleCorners[ways[2]],
						singleCorners[x], imgResult);
				control[1] = findExtensionColor(singleCorners[ways[1]],
						singleCorners[x], imgResult);
				//				cout<<"control[0]"<<control[0]<<endl;
				//				cout<<"control[1]"<<control[1]<<endl;
				if (control[0] == 0 && control[1] == 0) {
					identificationIndex[x][0] = 0;
					identificationIndex[x][1] = 0;
					identified[x] = true;
					willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
							= true;
					places[identificationIndex[x][0]][identificationIndex[x][1]]
							= singleCorners[x];
				} else if (control[0] == 0 && control[1] == 1) {

					int vertical = findExtensionColor(singleCorners[x],
							singleCorners[ways[1]], imgResult);
					if (vertical == 1) {
						identificationIndex[x][0] = 1;
						identificationIndex[x][1] = 0;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					} else if (vertical == 0) {
						identificationIndex[x][0] = 2;
						identificationIndex[x][1] = 0;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					}

				} else if (control[0] == 1 && control[1] == 0) {
					int horizontal = findExtensionColor(singleCorners[x],
							singleCorners[ways[2]], imgResult);
					if (horizontal == 1) {
						identificationIndex[x][0] = 0;
						identificationIndex[x][1] = 1;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					} else if (horizontal == 0) {
						identificationIndex[x][0] = 0;
						identificationIndex[x][1] = 2;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					}
				} else if (control[0] == 1 && control[1] == 1) {
					selections[1] = true;
					selections[2] = true;
					verhor[0] = findExtensionColor(singleCorners[x],
							singleCorners[ways[1]], imgResult);
					verhor[1] = findExtensionColor(singleCorners[x],
							singleCorners[ways[2]], imgResult);

					if (verhor[0] != -2 && verhor[1] != -2 && verhor[0] != -1
							&& verhor[1] != -1) {
						//cout<<"TAGGING"<<endl;
						tagMainCorner(selections, verhor, x, ways);
					}
				}
			} else if (ways[1] == -1 && ways[2] == -1) {
				cout<<"Asagi ve sag bos"<<endl;
				control[0] = findExtensionColor(singleCorners[ways[3]],singleCorners[x], imgResult);
				control[1] = findExtensionColor(singleCorners[ways[0]],singleCorners[x], imgResult);
				//cvShowImage("Nassi ya",imgResult);
				//cvWaitKey();
								cout<<"control[0]"<<control[0]<<endl;
								cout<<"control[1]"<<control[1]<<endl;
				if (control[0] == 0 && control[1] == 0) {
					identificationIndex[x][0] = 3;
					identificationIndex[x][1] = 3;
					identified[x] = true;
					willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
							= true;
					places[identificationIndex[x][0]][identificationIndex[x][1]]
							= singleCorners[x];
				} else if (control[0] == 0 && control[1] == 1) {

					int horizontal = findExtensionColor(singleCorners[x],
							singleCorners[ways[0]], imgResult);
					if (horizontal == 1) {
						identificationIndex[x][0] = 3;
						identificationIndex[x][1] = 2;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					} else if (horizontal == 0) {
						identificationIndex[x][0] = 3;
						identificationIndex[x][1] = 1;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					}

				} else if (control[0] == 1 && control[1] == 0) {
					int vertical = findExtensionColor(singleCorners[x],
							singleCorners[ways[3]], imgResult);
					if (vertical == 1) {
						identificationIndex[x][0] = 2;
						identificationIndex[x][1] = 3;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					} else if (vertical == 0) {
						identificationIndex[x][0] = 1;
						identificationIndex[x][1] = 3;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					}
				} else if (control[0] == 1 && control[1] == 1) {
					selections[3] = true;
					selections[0] = true;
					verhor[0] = findExtensionColor(singleCorners[x],
							singleCorners[ways[3]], imgResult);
					verhor[1] = findExtensionColor(singleCorners[x],
							singleCorners[ways[0]], imgResult);

					if (verhor[0] != -2 && verhor[1] != -2 && verhor[0] != -1
							&& verhor[1] != -1) {
						tagMainCorner(selections, verhor, x, ways);
					}
				}
			} else if (ways[0] == -1 && ways[1] == -1) {
				cout<<"Sol ve asagi bos"<<endl;
				control[0] = findExtensionColor(singleCorners[ways[2]],
						singleCorners[x], imgResult);
				control[1] = findExtensionColor(singleCorners[ways[3]],
						singleCorners[x], imgResult);
				//				cout<<"control[0]"<<control[0]<<endl;
				//				cout<<"control[1]"<<control[1]<<endl;
				if (control[0] == 0 && control[1] == 0) {
					identificationIndex[x][0] = 3;
					identificationIndex[x][1] = 0;
					identified[x] = true;
					willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
							= true;
					places[identificationIndex[x][0]][identificationIndex[x][1]]
							= singleCorners[x];
				} else if (control[0] == 0 && control[1] == 1) {

					int vertical = findExtensionColor(singleCorners[x],
							singleCorners[ways[3]], imgResult);
					if (vertical == 1) {
						identificationIndex[x][0] = 2;
						identificationIndex[x][1] = 0;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					} else if (vertical == 0) {
						identificationIndex[x][0] = 1;
						identificationIndex[x][1] = 0;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					}

				} else if (control[0] == 1 && control[1] == 0) {
					int horizontal = findExtensionColor(singleCorners[x],
							singleCorners[ways[2]], imgResult);
					if (horizontal == 1) {
						identificationIndex[x][0] = 3;
						identificationIndex[x][1] = 1;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					} else if (horizontal == 0) {
						identificationIndex[x][0] = 3;
						identificationIndex[x][1] = 2;
						identified[x] = true;
						willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]]
								= true;
						places[identificationIndex[x][0]][identificationIndex[x][1]]
								= singleCorners[x];
					}
				} else if (control[0] == 1 && control[1] == 1) {
					selections[3] = true;
					selections[2] = true;
					verhor[0] = findExtensionColor(singleCorners[x],
							singleCorners[ways[3]], imgResult);
					verhor[1] = findExtensionColor(singleCorners[x],
							singleCorners[ways[2]], imgResult);

					if (verhor[0] != -2 && verhor[1] != -2 && verhor[0] != -1
							&& verhor[1] != -1) {
						tagMainCorner(selections, verhor, x, ways);
					}
				}
			}

			if (ways != NULL) {
				delete[] ways;
				ways = NULL;
			}
		}
	}

	for (int g = 0; g < counter; g++) {
		if (!identified[g]) {

			for (int y = 0; y < indexAndDegrees[g][0]; y++) {
				if (identified[indexAndDegrees[g][2 * y + 1]]) {
					if ((indexAndDegrees[g][2 * y + 2] <= 225) && (indexAndDegrees[g][2 * y + 2] > 135)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y+ 1]][0];
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1] - 1;

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}

					} else if ((indexAndDegrees[g][2 * y + 2] > 315) || (indexAndDegrees[g][2 * y + 2] <= 45)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0];
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1] + 1;

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					} else if ((indexAndDegrees[g][2 * y + 2] <= 315) && (indexAndDegrees[g][2 * y + 2] > 225)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0] - 1;
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1];

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					} else if ((indexAndDegrees[g][2 * y + 2] <= 135) && (indexAndDegrees[g][2 * y + 2] > 45)) {
						identificationIndex[g][0] = identificationIndex[indexAndDegrees[g][2 * y + 1]][0] + 1;
						identificationIndex[g][1] = identificationIndex[indexAndDegrees[g][2 * y + 1]][1];

						if (identificationIndex[g][0] >= 0 && identificationIndex[g][0] < 4 && identificationIndex[g][1] >= 0 && identificationIndex[g][1] < 4) {
							identified[g] = true;
							willBeFilled[identificationIndex[g][0]][identificationIndex[g][1]] = true;
							places[identificationIndex[g][0]][identificationIndex[g][1]] = singleCorners[g];
							break;
						} else {
							identificationIndex[g][0] = -1;
							identificationIndex[g][1] = -1;
						}
					}
				}
			}
		}
	}
}
void initializeForTagging3KnownCorners(int *ways,bool selections[],int verhor[],int x)
{
	int distanceToExtract = -1;
	int crossEmptySideIndex = -1;
	int wayIndex = -1;
	int xIndex = -1, yIndex = -1;
	for(wayIndex = 0;wayIndex < 4;wayIndex++)
	{
		if(ways[wayIndex] == -1)
		{
			crossEmptySideIndex = (wayIndex + 2) % 4;
			//Determine the non-empty pairs' closest test distance
			if(wayIndex % 2 == 0)
				distanceToExtract = 240;
			else
				distanceToExtract = 320;
			break;
		}
	}
	if(wayIndex == 0 || wayIndex == 3)
	{
		xIndex = 0;
		yIndex = 0;
	}
	else
	{
		xIndex = 3;
		yIndex = 3;
	}
		
	//Find whether empty side has white extension or not
	int whiteOrBlack = findExtensionColor(singleCorners[ways[crossEmptySideIndex]],singleCorners[x], imgResult);

	//Find the closest one to center between non-empty pair corners
	if (abs(singleCorners[ways[(crossEmptySideIndex+1)%4]].y - distanceToExtract) < abs(singleCorners[ways[(crossEmptySideIndex +3)%4]].y - distanceToExtract)) {
		selections[(crossEmptySideIndex+1)%4] = true;
		verhor[(crossEmptySideIndex) % 2] = findExtensionColor(singleCorners[x],singleCorners[ways[(crossEmptySideIndex+1)%4]], imgResult);
	} else {
		selections[(crossEmptySideIndex+3)%4] = true;
		verhor[(crossEmptySideIndex) % 2] = findExtensionColor(singleCorners[x], singleCorners[ways[(crossEmptySideIndex+3)%4]], imgResult);
	}

	if (whiteOrBlack == 1) {
		selections[crossEmptySideIndex] = true;
		verhor[(crossEmptySideIndex+1) % 2] = findExtensionColor(singleCorners[x], singleCorners[ways[crossEmptySideIndex]], imgResult);

		if (verhor[0] >= 0  && verhor[1] >= 0) {
			tagMainCorner(selections, verhor, x, ways);
		}
	} else if (whiteOrBlack == 0) {
		if(crossEmptySideIndex % 2 == 0)
		{
			if ((selections[(crossEmptySideIndex + 1) % 4] && verhor[crossEmptySideIndex%2] == 1) || (selections[(crossEmptySideIndex + 3) % 4] && verhor[crossEmptySideIndex%2] == 0)) {
				if(wayIndex == 0)
					identificationIndex[x][0] = 2;
				else
					identificationIndex[x][0] = 1;
			} else if ((selections[(crossEmptySideIndex + 1) % 4] && verhor[crossEmptySideIndex%2] == 0) || (selections[(crossEmptySideIndex + 3) % 4] && verhor[crossEmptySideIndex%2] == 1)) {
				if(wayIndex == 0)
					identificationIndex[x][0] = 1;
				else
					identificationIndex[x][0] = 2;
			}
			identificationIndex[x][1] = yIndex;
			identified[x] = true;
			willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]] = true;
			places[identificationIndex[x][0]][identificationIndex[x][1]]= singleCorners[x];
		}
		else
		{
			if ((selections[(crossEmptySideIndex + 1) % 4] && verhor[crossEmptySideIndex%2] == 1) || (selections[(crossEmptySideIndex + 3) % 4] && verhor[crossEmptySideIndex%2] == 0)) {
				if(wayIndex == 1)
					identificationIndex[x][1] = 2;
				else
					identificationIndex[x][1] = 1;
			} else if ((selections[(crossEmptySideIndex + 1) % 4] && verhor[crossEmptySideIndex%2] == 0) || (selections[(crossEmptySideIndex + 3) % 4] && verhor[crossEmptySideIndex%2] == 1)) {
				if(wayIndex == 1)
					identificationIndex[x][1] = 1;
				else
					identificationIndex[x][1] = 2;
			}
			identificationIndex[x][0] = xIndex;
			identified[x] = true;
			willBeFilled[identificationIndex[x][0]][identificationIndex[x][1]] = true;
			places[identificationIndex[x][0]][identificationIndex[x][1]]= singleCorners[x];
		}


	}

}

void initializeWays(int *ways,int x,int indexAndDegrees[][9])
{
	for (int i = 0; i < 4; i++) {
		ways[i] = -1;
	}
	for (int i = 0; i < indexAndDegrees[x][0]; i++) {

		if (indexAndDegrees[x][2 * i + 2] > 135 && indexAndDegrees[x][2* i + 2] <= 225) {
			ways[2] = indexAndDegrees[x][2 * i + 1];
		} else if (indexAndDegrees[x][2 * i + 2] > 45 && indexAndDegrees[x][2 * i + 2] <= 135) {
			ways[3] = indexAndDegrees[x][2 * i + 1];
		} else if (indexAndDegrees[x][2 * i + 2] > 315 || indexAndDegrees[x][2 * i + 2] <= 45) {
			ways[0] = indexAndDegrees[x][2 * i + 1];
		} else if (indexAndDegrees[x][2 * i + 2] > 225 && indexAndDegrees[x][2 * i + 2] <= 315) {
			ways[1] = indexAndDegrees[x][2 * i + 1];
		}

	}
}


void findEdgeAndCornerFeatures(int turn)
{
	CvSize img_sz = cvGetSize(imgResult);
	if (tmp_image != NULL) {
		cvReleaseImage(&tmp_image);
		tmp_image = NULL;
	}
	tmp_image = cvCreateImage(img_sz, IPL_DEPTH_32F, 1);
	if (eig_image != NULL) {
		cvReleaseImage(&eig_image);
		eig_image = NULL;
	}
	eig_image = cvCreateImage(img_sz, IPL_DEPTH_32F, 1);
	featuresPerTurn = new CvPoint2D32f[corner_count];
	//cvGoodFeaturesToTrack(imgResult, eig_image, tmp_image, featuresPerTurn, &corner_countPerTurn, 0.05, 3, 0, 2, 0, 0.4);
	//BEttercvGoodFeaturesToTrack(imgResult, eig_image, tmp_image, featuresPerTurn, &corner_countPerTurn, 0.001, 5, NULL, 10, 0, 0.04);
	cvGoodFeaturesToTrack(imgResult, eig_image, tmp_image, featuresPerTurn, &corner_countPerTurn, 0.1, 5, NULL, 10, 1, 0.04);
	
	int c = 0;
	for(int a = turn*600; a < 600*(turn+1);a++)
	{
		features[a].x =  featuresPerTurn[c].x;
		features[a].y = featuresPerTurn[c].y;
		c++;
	}
	
	//for(int a = 0; a < corner_count;a++)
	//{
		//cvCircle(copyOfGlobBgr, cvPoint(features[a].x, features[a].y), 4, cvScalar(0, 255, 0), -1, 8);
	//}
	//cvShowImage("Features",copyOfGlobBgr);
	//cvWaitKey();
	printf("Number of Features:%d\n",corner_count);
}
void validateFeaturesAsRealCorners()
{
	int validationNos[corner_count];
	
	double degrees[corner_count][2];
	
	for (int i = 0; i < corner_count; i++) {
		validationNos[i] = 0;
	}
	int white;
	double deg = 0.0;
	countOfCorners = 0;
	
	for (int i = 0; i < corner_count; i++) {
		for (int j = 0; j < corner_count; j++) {
			if (i != j && (getEuclidianDistance(features[i], features[j]) > 70 + features[i].y * 0.05)) {//80 and 0.05
				white = findEdgeColorForVerification(features[i], features[j], imgResult);
				if (white == 1) {
					deg = (int) ((atan2((features[i].y - features[j].y),(features[i].x - features[j].x)) * 180)/ PI + 360) % 360;

					if (validationNos[i] == 0) {
						degrees[i][validationNos[i]] = deg;
						validationNos[i]++;
					} else {
						int degreeDiff = abs(degrees[i][validationNos[i] - 1] - deg);
						if ((degreeDiff > 45 && degreeDiff < 135) || (degreeDiff > 225 && degreeDiff< 315)) {

							degrees[i][validationNos[i]] = deg;
							validationNos[i]++;
							countOfCorners++;
						}
					}
				}
				white = 0;
			}
			if (validationNos[i] == 2)
				break;
		}
	}
	
	newCornerFeatures = new CvPoint2D32f[countOfCorners];
	int index = 0;
	for (int i = 0; i < corner_count; i++) {
		if (validationNos[i] == 2) {
			newCornerFeatures[index].x = features[i].x;
			newCornerFeatures[index].y = features[i].y;
			index++;
		}

	}
	
	//printf("Number of Corners:%d\n",countOfCorners);
	cvCopyImage(bgr_frameLeftCopy,copyOfGlobBgr);
	for(int a = 0; a < countOfCorners;a++)
	{
		cvCircle(copyOfGlobBgr, cvPoint(newCornerFeatures[a].x, newCornerFeatures[a].y), 4, cvScalar(255, 255, 0), -1, 8);
	}
	cvShowImage("Corners",copyOfGlobBgr);
	//cvWaitKey();
}
void groupCloserCorners()
{
	singleCorners = new CvPoint2D32f[countOfCorners];
	for (int i = 0; i < countOfCorners; i++) {
		singleCorners[i].x = 0;
		singleCorners[i].y = 0;
	}
	int grouping[countOfCorners];
	for (int i = 0; i < countOfCorners; i++) {
		grouping[i] = -1;
	}

	counter = 0;
	int no = 1;
	for (int i = 0; i < countOfCorners; i++) {
		int group = counter;
		if (grouping[i] == -1) {
			grouping[i] = group;
			singleCorners[counter].x += (int) newCornerFeatures[i].x;
			singleCorners[counter].y += (int) newCornerFeatures[i].y;

			for (int x = i + 1; x < countOfCorners; x++) {
				if (grouping[x] == -1 && getEuclidianDistance(newCornerFeatures[i], newCornerFeatures[x]) < 30 + newCornerFeatures[i].y * 0.15) {//)+ (newCornerFeatures[i].y/480)*90 ) {//35 and 0.13
					grouping[x] = group;
					singleCorners[counter].x += (int) newCornerFeatures[x].x;
					singleCorners[counter].y += (int) newCornerFeatures[x].y;
					no++;
				}
			}
			singleCorners[counter].x = (int) (singleCorners[counter].x / no);
			singleCorners[counter].y = (int) (singleCorners[counter].y / no);
			no = 1;
			counter++;
		}
	}
	cvCopyImage(bgr_frameLeftCopy,copyOfGlobBgr);
	for(int a = 0; a < counter;a++)
	{
		cvCircle(copyOfGlobBgr, cvPoint(singleCorners[a].x, singleCorners[a].y), 4, cvScalar(255, 255, 0), -1, 8);
	}
	
	cvShowImage("SingleCorners",copyOfGlobBgr);
	//cvWaitKey();
	printf("Noktalar:%d\n",counter);
}
void findNeighbourOfEachCorner(int indexAndDegrees[][9])
{
	int white;
	for (int i = 0; i < counter; i++) {
		indexAndDegrees[i][0] = 0;
		for (int j = 1; j < 9; j++) {
			indexAndDegrees[i][j] = -1;
		}
	}

	for (int i = 0; i < counter; i++) {
		for (int j = 0; j < counter; j++) {
			if (i != j) {
				white = findEdgeColorForVerification(singleCorners[i],singleCorners[j], imgResult);
				if (white == 1) {
					int degree = (int) ((atan2((singleCorners[i].y - singleCorners[j].y), (singleCorners[i].x - singleCorners[j].x)) * 180) / PI + 360) % 360;
					int minIndex = j;
					int minDegree = degree;
					double minValue = getEuclidianDistance( singleCorners[i], singleCorners[j]);
					bool flag = false;
					if ((degree > 45 && degree <= 135) || (degree > 205 && degree < 325)) {
						if (minValue < 150)//220
							flag = true;
					} else if ((degree > 315 || degree < 45) || (degree > 135 && degree < 215)) {
						if (minValue < 220)//300
							flag = true;
					}

					int degreeToCompare = -1;
					for (int k = 0; k < counter; k++) {
						if (!(i == k || j == k)) {
							white = findEdgeColorForVerification( singleCorners[i], singleCorners[k], imgResult);
							if (white == 1) {
								degreeToCompare = (int) ((atan2((singleCorners[i].y - singleCorners[k].y),(singleCorners[i].x - singleCorners[k].x))* 180) / PI + 360) % 360;

								if (degreeToCompare > 45 && degreeToCompare < 315 && degree > 45 && degree < 315 && degreeToCompare < degree + 10 && degreeToCompare > degree - 10) {
									if (getEuclidianDistance( singleCorners[i],singleCorners[k]) < minValue &&
											((degreeToCompare > 35 && degreeToCompare <= 145) ||
											(degreeToCompare > 205 && degreeToCompare < 325))
											&& getEuclidianDistance(singleCorners[i],singleCorners[k]) < 150) {
										minValue = getEuclidianDistance(singleCorners[i], singleCorners[k]);
										minIndex = k;
										minDegree = degreeToCompare;

									} else if (getEuclidianDistance( singleCorners[i], singleCorners[k]) < minValue &&
											(degreeToCompare > 135 && degreeToCompare < 215) &&
											getEuclidianDistance( singleCorners[i], singleCorners[k]) < 220) {
										minValue = getEuclidianDistance(singleCorners[i], singleCorners[k]);
										minIndex = k;
										minDegree = degreeToCompare;
									}
								} else if ((degreeToCompare > 315 || degreeToCompare < 45)
										&& (degree < 45 || degree > 315)) {
									if (getEuclidianDistance(singleCorners[i],singleCorners[k]) < minValue &&
											getEuclidianDistance(singleCorners[i], singleCorners[k]) < 220) {
										minValue = getEuclidianDistance( singleCorners[i], singleCorners[k]);
										minIndex = k;
										minDegree = degreeToCompare;
									}
								}
							}
						}

					}
					int noOfCorner = indexAndDegrees[i][0];
					if (noOfCorner != 4) {
						for (int x = 0; x < noOfCorner + 1; x++) {
							if (indexAndDegrees[i][2 * x + 1] == minIndex) {
								break;
							} else if (indexAndDegrees[i][2 * x + 1] == -1 && flag) {
								indexAndDegrees[i][2 * x + 1] = minIndex;
								indexAndDegrees[i][2 * x + 2] = minDegree;
								indexAndDegrees[i][0]++;
							}
						}
					}

				}

			}

		}
	}
	printf("=======================\n");
	for(int i = 0; i < counter; i++)
	{
		printf("Neighbours: %d; ",indexAndDegrees[i][0]);
		for (int x = 0; x < indexAndDegrees[i][0]; x++) {
			printf("%d %d;",indexAndDegrees[i][2*x+1],indexAndDegrees[i][2*x+2]);
		}
		printf("\n");
	}
	printf("=======================\n");
	cvWaitKey();
}
void tagTableCornersAndAreas()
{
	//for (int x = 0; x < counter; x++) {
		//if (identified[x]) {
			//cvCircle(tempImg1, cvPoint(singleCorners[x].x, singleCorners[x].y), 4, cvScalar(0, 255, 0), -1, 8);
			//cvPutText(tempImg1, positions[identificationIndex[x][0]][identificationIndex[x][1]], cvPoint(singleCorners[x].x, singleCorners[x].y),&font, cvScalar(0, 255, 0, 0));
		//}
	//}
	
	for (int x = 0; x < 4; x++) {
		for (int y = 0; y < 4; y++) {
			if(willBeFilled[x][y])
			{
				cvCircle(tempImg1, cvPoint(places[x][y].x, places[x][y].y), 4, cvScalar(0, 255, 0), -1, 8);
				cvPutText(tempImg1, positions[x][y], cvPoint(places[x][y].x, places[x][y].y),&font, cvScalar(0, 255, 0, 0));
			}
		}
	}

	for (int x = 0; x < 3; x++) {
		for (int y = 0; y < 3; y++) {
			if (willBeFilled[x][y] && willBeFilled[x + 1][y + 1] &&
					(places[x][y].x + places[x + 1][y + 1].x) / 2 < tempImg->width && (places[x][y].x + places[x + 1][y + 1].x) / 2 > 0 &&
					(places[x][y].y + places[x + 1][y + 1].y) / 2 < tempImg->height && (places[x][y].y+ places[x + 1][y + 1].y) / 2 > 0) {

				cvCircle(tempImg, cvPoint((places[x][y].x + places[x + 1][y + 1].x) / 2, (places[x][y].y + places[x + 1][y + 1].y) / 2), 4, cvScalar(0, 255, 0),-1, 8);
				cvPutText(tempImg, areas[x][y], cvPoint((places[x][y].x + places[x + 1][y + 1].x) / 2, (places[x][y].y + places[x + 1][y + 1].y) / 2), &font, cvScalar(0, 255, 0, 0));
			} else if (willBeFilled[x][y + 1] && willBeFilled[x + 1][y] &&
					(places[x][y + 1].x + places[x + 1][y].x) / 2 < tempImg->width && (places[x][y + 1].x + places[x + 1][y].x) / 2 > 0 &&
					(places[x][y + 1].y + places[x + 1][y].y) / 2 < tempImg->height && (places[x][y + 1].y + places[x + 1][y].y) / 2 > 0) {
				cvCircle(tempImg, cvPoint((places[x][y + 1].x + places[x + 1][y].x) / 2, (places[x][y + 1].y + places[x + 1][y].y) / 2), 4, cvScalar(0, 255, 0), -1, 8);
				cvPutText(tempImg, areas[x][y], cvPoint((places[x][y + 1].x + places[x + 1][y].x) / 2, (places[x][y + 1].y + places[x + 1][y].y) / 2), &font,cvScalar(0, 255, 0, 0));
			}
		}
	}
}
void segmentObjectsOld(IplImage* bgr_frameLeft)
{
	if (copyBgr != NULL) {
		cvReleaseImage(&copyBgr);
		copyBgr = NULL;
	}
	copyBgr = cvCreateImage(cvSize(bgr_frameLeft->width, bgr_frameLeft->height), IPL_DEPTH_8U,3);
	cvCopyImage(bgr_frameLeft,copyBgr);
	if (ROIGrayAndBinary != NULL) {
		cvReleaseImage(&ROIGrayAndBinary);
		ROIGrayAndBinary = NULL;
	}
	ROIGrayAndBinary = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);
	cvCvtColor(copyBgr, ROIGrayAndBinary, CV_BGR2GRAY);
	cvThreshold(ROIGrayAndBinary, ROIGrayAndBinary, 210, 255, CV_THRESH_BINARY);
	cvSmooth(ROIGrayAndBinary, ROIGrayAndBinary, CV_GAUSSIAN, 3, 3, 0, 0);
	cvDilate(ROIGrayAndBinary, ROIGrayAndBinary, NULL, 4);

	CvScalar ptr;
	CvScalar ptr1;
	for (int a = 0; a < ROIGrayAndBinary->width; a++) {
		for (int b = 0; b < ROIGrayAndBinary->height; b++) {
			if (b < ROIGrayAndBinary->height && b > 0 && a < ROIGrayAndBinary->width && a > 0) {
				ptr = cvGet2D(ROIGrayAndBinary, b, a);
				if (ptr.val[0] > 200.0 && b < copyBgr->height && b > 0 && a < copyBgr->width && a > 0)
					cvSet2D(copyBgr, b, a, cvScalarAll(0));
			}
		}

	}

	//++Ygt
	hsvImage= cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 3);
	hue = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);
	hue1 = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);
	hue2 = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);
	hue3 = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);

	cvCvtColor(copyBgr, hsvImage, CV_BGR2HSV);
	cvSetImageCOI(hsvImage, 1);
	cvCopy(hsvImage, hue);

	CvScalar hsv_min = cvScalar(5, 0, 0, 0);
	CvScalar hsv_max = cvScalar(40, 0, 0, 0);
	cvInRangeS(hue, hsv_min, hsv_max, hue1);

	CvScalar hsv_min1 = cvScalar(70, 0, 0, 0);
	CvScalar hsv_max1 = cvScalar(120, 0, 0, 0);
	cvInRangeS(hue, hsv_min1, hsv_max1, hue2);

	cvSmooth(hue1, hue1, CV_MEDIAN, 9, 9, 0, 0);
	cvSmooth(hue2, hue2, CV_MEDIAN, 9, 9, 0, 0);
	for (int a = 0; a < hue1->width; a++) {
		for (int b = 0; b < hue1->height; b++) {
			if (b < hue1->height && b > 0 && a < hue1->width && a > 0 && b < hue2->height && b > 0 && a < hue2->width && a > 0) {
				ptr = cvGet2D(hue1, b, a);
				ptr1 = cvGet2D(hue2, b, a);
				int g = ((int) ptr.val[0] | (int) ptr1.val[0]);
				if (b < hue3->height && b > 0 && a < hue3->width && a > 0)
					cvSet2D(hue3, b, a, cvScalarAll(g));
			}
		}

	}

	cvSmooth(hue3, hue3, CV_MEDIAN, 9, 9, 0, 0);
	for (int a = 0; a < copyBgr->width; a++) {
		for (int b = 0; b < copyBgr->height; b++) {
			if (b < copyBgr->height && b > 0 && a < copyBgr->width && a > 0) {
				ptr = cvGet2D(hue3, b, a);
				if (ptr.val[0] == 0 && b < copyBgr->height && b > 0 && a < copyBgr->width && a > 0)
					cvSet2D(copyBgr, b, a, cvScalarAll(255));
			}
		}

	}
	cvReleaseImage(&hsvImage);
	cvReleaseImage(&hue);
	cvReleaseImage(&hue1);
	cvReleaseImage(&hue2);
	hsvImage = NULL;
	hue = NULL;
	hue1 = NULL;
	hue2 = NULL;
}


//void segmentAreaObjects()
//{
	//int minY, maxY, minX, maxX;
	//int result = 0;
	//for (int x = 2; x >= 0; x--) {
		//for (int y = 2; y >= 0; y--) {
			//if (willBeFilled[x][y] && willBeFilled[x + 1][y + 1]) {
				//minY = 480, maxY = 0, minX = 640, maxX = 0;
				//total[0] = 0;
				//total[1] = 0;
				//total[2] = 0;
				//diff = 0;
				//countForPixels = 0;
				//cvZero(Image2);
				//result = floodFill((places[x][y].x + places[x + 1][y + 1].x) / 2, (places[x][y].y + places[x + 1][y + 1].y) / 2, &minX, &minY, &maxX, &maxY);
				////cout<<minX<<","<<minY<<","<<maxX<<","<<maxY<<endl;
				//if (result == 3) {
					//if (minX < copyBgr->width && minX >= 0 && maxX < copyBgr->width && maxX >= 0 && minY < copyBgr->height && minY >= 0 && maxY < copyBgr->height && maxY >= 0) {
						//if (x == 2) {
							//if (places[x][y].y - minY > (places[x + 1][y + 1].y - places[x][y].y) / 4)
								//setOrReset[x][y] = 1;
						    //else
								//setOrReset[x][y] = 0;
						//} else if (x == 1) {
							//if (setOrReset[x + 1][y] == -1 || setOrReset[x + 1][y] == 0) {
								//if (places[x][y].y - minY > (places[x + 1][y + 1].y - places[x][y].y) / 4)
									//setOrReset[x][y] = 1;
								//else if (setOrReset[x + 1][y] == -1 && (maxY - places[x + 1][y + 1].y > (places[x + 1][y + 1].y - places[x][y].y)/ 4))
									//setOrReset[x][y] = -1;
								//else if (setOrReset[x + 1][y] != 1)
									//setOrReset[x][y] = 0;
							//}
						//} else {
							//if (setOrReset[x + 1][y] == -1 && (maxY - places[x + 1][y + 1].y > (places[x + 1][y + 1].y - places[x][y].y) / 4))
								//setOrReset[x][y] = -1;
							//else if (setOrReset[x + 1][y] != 1)
								//setOrReset[x][y] = 0;
						//}
						//if (images[y + x * 3] != NULL) {
							//cvReleaseImage(&images[y + x * 3]);
							//images[y + x * 3] = NULL;
						//}
						//images[y + x * 3] = cvCreateImage(cvSize(maxX - minX, maxY - minY), copyBgr->depth, copyBgr->nChannels);
						//if (x != 2 && setOrReset[x + 1][y] != 1)
							//willBeSent.boundingBoxes[y + x * 3] = cvRect(minX, minY, maxX - minX, maxY - minY);
						//else if (x == 2)
							//willBeSent.boundingBoxes[y + x * 3] = cvRect(minX, minY, maxX - minX, maxY - minY);

						//cvSetImageROI(copyBgr, cvRect(minX, minY, maxX - minX, maxY - minY));
						//if (cvGetImageROI(copyBgr).width == maxX - minX && cvGetImageROI(copyBgr).height == maxY - minY) {
							//if (x != 2 && setOrReset[x + 1][y] != 1) {
								//cvCopy(copyBgr, images[y + x * 3]);
								//if (!seenOrNot[y + x * 3])
									//seenOrNot[y + x * 3] = true;
							//} else if (x == 2) {
								//cvCopy(copyBgr, images[y + x * 3]);
								//if (!seenOrNot[y + x * 3])
									//seenOrNot[y + x * 3] = true;
							//}
						//}
						//cvResetImageROI(copyBgr);
					//}

				//} else if (result == -1 || result == -2) {
					//if (seenOrNot[y + x * 3]) {
						//willBeSent.boundingBoxes[y + x * 3] = cvRect(0, 0, 0, 0);
						//willBeSent.segmented_areas[y + x * 3] = -1;
						//cvClearHist(willBeSent.blueHists[y + x * 3]);
						//cvClearHist(willBeSent.redHists[y + x * 3]);
						//cvClearHist(willBeSent.greenHists[y + x * 3]);
						//setOrReset[x][y] = -1;
						//seenOrNot[y + x * 3] = false;
						//if (images[y + x * 3] != NULL) {
							//cvReleaseImage(&images[y + x * 3]);
							//images[y + x * 3] = NULL;
						//}
					//}
				//}
			//} else if (willBeFilled[x][y + 1] && willBeFilled[x + 1][y]) {

				//minY = 480, maxY = 0, minX = 640, maxX = 0;
				//total[0] = 0;
				//total[1] = 0;
				//total[2] = 0;
				//diff = 0;
				//countForPixels = 0;
				//cvZero(Image2);
				//result = floodFill((places[x][y + 1].x + places[x + 1][y].x) / 2, (places[x][y + 1].y + places[x + 1][y].y) / 2, &minX, &minY, &maxX, &maxY);
				//if (result == 3) {
					//if (minX < copyBgr->width && minX >= 0 && maxX
							//< copyBgr->width && maxX >= 0 && minY
							//< copyBgr->height && minY >= 0 && maxY
							//< copyBgr->height && maxY >= 0) {
						//if (x == 2) {
							//if (places[x][y + 1].y - minY > (places[x + 1][y].y - places[x][y + 1].y) / 4)
								//setOrReset[x][y] = 1;
							//else
								//setOrReset[x][y] = 0;

						//} else if (x == 1) {
							//if (setOrReset[x + 1][y] == -1 || setOrReset[x + 1][y] == 0) {
								//if (places[x][y + 1].y - minY > (places[x + 1][y].y - places[x][y + 1].y) / 4)
									//setOrReset[x][y] = 1;
								//else if (setOrReset[x + 1][y] == -1 && (maxY - places[x + 1][y].y > (places[x + 1][y].y - places[x][y + 1].y) / 4))
									//setOrReset[x][y] = -1;
								//else if (setOrReset[x + 1][y] != 1)
									//setOrReset[x][y] = 0;
							//}
						//} else {
							//if (setOrReset[x + 1][y] == -1 && (maxY - places[x + 1][y].y > (places[x + 1][y].y - places[x][y + 1].y) / 4))
								//setOrReset[x][y] = -1;
							//else if (setOrReset[x + 1][y] != 1)
								//setOrReset[x][y] = 0;
						//}

						//if (images[y + x * 3] != NULL) {
							//cvReleaseImage(&images[y + x * 3]);
							//images[y + x * 3] = NULL;

						//}
						//images[y + x * 3] = cvCreateImage(cvSize(maxX - minX, maxY - minY), copyBgr->depth, copyBgr->nChannels);

						//if (x != 2 && setOrReset[x + 1][y] != 1)
							//willBeSent.boundingBoxes[y + x * 3] = cvRect(minX, minY, maxX - minX, maxY - minY);
						//else if (x == 2)
							//willBeSent.boundingBoxes[y + x * 3] = cvRect(minX, minY, maxX - minX, maxY - minY);


						//cvSetImageROI(copyBgr, cvRect(minX, minY, maxX - minX, maxY - minY));
						//if (cvGetImageROI(copyBgr).width == maxX - minX && cvGetImageROI(copyBgr).height == maxY - minY) {
							//if (x != 2 && setOrReset[x + 1][y] != 1) {
								//cvCopy(copyBgr, images[y + x * 3]);
								//if (!seenOrNot[y + x * 3])
									//seenOrNot[y + x * 3] = true;
							//} else if (x == 2) {
								//cvCopy(copyBgr, images[y + x * 3]);
								//if (!seenOrNot[y + x * 3])
									//seenOrNot[y + x * 3] = true;
							//}
						//}

						//cvResetImageROI(copyBgr);
					//}

				//} else if (result == -1 || result == -2) {
					//if (seenOrNot[y + x * 3]) {
						//willBeSent.boundingBoxes[y + x * 3] = cvRect(0,0, 0, 0);
						//willBeSent.segmented_areas[y + x * 3] = -1;
						//cvClearHist(willBeSent.blueHists[y + x * 3]);
						//cvClearHist(willBeSent.redHists[y + x * 3]);
						//cvClearHist(willBeSent.greenHists[y + x * 3]);
						//setOrReset[x][y] = -1;
						//seenOrNot[y + x * 3] = false;
						//if (images[y + x * 3] != NULL) {
							//cvReleaseImage(&images[y + x * 3]);
							//images[y + x * 3] = NULL;

						//}
					//}
				//}
			//}
		//}
	//}
//}

IplImage * ContrastBrightness(IplImage *src, int Contrast, int Brightness)
{

	if(Contrast > 100) Contrast = 100;
	if(Contrast < -100) Contrast = -100;
	if(Brightness > 100) Brightness = 100;
	if(Brightness < -100) Brightness = -100;

	uchar lut[256];

	CvMat* lut_mat;
	int hist_size = 256;
	float range_0[]={0,256};
	float* ranges[] = { range_0 };

	int i;

	IplImage * dest = cvCloneImage(src);

	IplImage * GRAY = cvCreateImage(cvGetSize(src),src->depth,1);

	if (src->nChannels ==3)
	{
		cvCvtColor(src,GRAY,CV_RGB2GRAY);
	}
	else
	{
		cvCopyImage(src,GRAY);
	}

    lut_mat = cvCreateMatHeader( 1, 256, CV_8UC1 );
    cvSetData( lut_mat, lut, 0 );

	 /*
     * The algorithm is by Werner D. Streidt
     * (http://visca.com/ffactory/archives/5-99/msg00021.html)
     */

	if( Contrast > 0 )

    {
        double delta = 127.* Contrast/100;
        double a = 255./(255. - delta*2);
        double b = a*(Brightness - delta);
        for( i = 0; i < 256; i++ )
        {
            int v = cvRound(a*i + b);

            if( v < 0 )
                v = 0;
            if( v > 255 )
                v = 255;
            lut[i] = v;
        }
    }
    else
    {
        double delta = -128.* Contrast/100;

        double a = (256.-delta*2)/255.;
        double b = a* Brightness + delta;
        for( i = 0; i < 256; i++ )
        {
            int v = cvRound(a*i + b);
            if( v < 0 )
                v = 0;

            if( v > 255 )
                v = 255;
            lut[i] = v;
        }
    }

	if (src->nChannels ==3)
	{

		IplImage * R = cvCreateImage(cvGetSize(src),src->depth,1);

		IplImage * G = cvCreateImage(cvGetSize(src),src->depth,1);
		IplImage * B = cvCreateImage(cvGetSize(src),src->depth,1);

		cvCvtPixToPlane(src,R,G,B,NULL);

		// PERFORM IT ON EVERY CHANNEL
		cvLUT( R, R, lut_mat );

		cvLUT( G, G, lut_mat );
		cvLUT( B, B, lut_mat );

		cvCvtPlaneToPix(R,G,B,NULL,dest);

		cvReleaseImage(&R);
		cvReleaseImage(&G);
		cvReleaseImage(&B);

	}
	else
	{

		//PERFORM IT ON THE CHANNEL
		cvLUT( GRAY, dest, lut_mat );
	}

	cvReleaseImage(&GRAY);
	cvReleaseMat( &lut_mat);

	return dest;

}

void identifyDynamicStructures()
{
	identificationIndex = new int*[counter];
	for (int y = 0; y < counter; y++) {
		identificationIndex[y] = new int[2];
	}

	identified = new bool[counter];
	for (int x = 0; x < counter; x++) {
		identified[x] = false;
	}

	for (int x = 0; x < counter; x++) {
		for (int y = 0; y < 2; y++) {
			identificationIndex[x][y] = -1;
		}
	}

	for (int x = 0; x < 4; x++) {
		for (int y = 0; y < 4; y++) {
			willBeFilled[x][y] = false;
		}
	}
}
void segmentTabletopObjects(int* confidenceCounter) {
	frameLeft = imagePortLeft.read();
	if (frameLeft != NULL) {
		countt++;
		cout << countt << endl;
		IplImage *bgr_frameLeft = NULL;
		if (bgr_frameLeft != NULL) {
			cvReleaseImage(&bgr_frameLeft);
			bgr_frameLeft = NULL;
		}
		bgr_frameLeft = (IplImage*) frameLeft->getIplImage();
		//cvSmooth(bgr_frameLeft, bgr_frameLeft, CV_GAUSSIAN, 9, 9, 3, 3);
		bgr_frameLeftBrighted = ContrastBrightness(bgr_frameLeft, 70, 0);
		//cvShowImage("Contrast",bgr_frameLeft);
		//cvWaitKey();
		countForIdentified = 0;
		cvCvtColor(bgr_frameLeftBrighted, tempImgG, CV_BGR2GRAY);
		cvCopyImage(bgr_frameLeft,tempImg);
		cvCopyImage(bgr_frameLeft,bgr_frameLeftCopy);
		//cvSmooth(tempImgG, tempImgG, CV_GAUSSIAN, 3, 3, 0, 0);
		cvThreshold(tempImgG, imgResult, 254, 255, CV_THRESH_BINARY);
		cvDilate(imgResult, imgResult, NULL, 2);
		for(int y = 0; y < 480;y = y + 6)
			for(int x = 0; x < 640;x = x + 6)
			{
				cvSet2D(imgResult,y,x,cvScalarAll(0));
				if(x+1 < 640)
					cvSet2D(imgResult,y,x + 1,cvScalarAll(0));
				if(y+1 < 480)
					cvSet2D(imgResult,y+1,x,cvScalarAll(0));
				if(x-1 >= 0)
					cvSet2D(imgResult,y,x - 1,cvScalarAll(0));
				if(y-1 >= 0)
					cvSet2D(imgResult,y-1,x,cvScalarAll(0));
			}
		//cvShowImage("noise",imgResult);
		//cvWaitKey(0);
		//cvSmooth(imgResult, imgResult, CV_MEDIAN, 9, 9, 0, 0);
		//-============================================
		
		//segmentObjectsNew(bgr_frameLeft);
		
		//=============================================
		//cvShowImage("Filtered",copyBgr);
		//cvWaitKey(0);
		//cvErode(imgResult, imgResult, NULL,1);
		igaze->getAngles(angles);

		//inBot = motionSense.read();
		//if (inBot == NULL) continue;

		//motion = inBot->get(0).asInt();
		//cout<<"motion: "<<motion<<endl;
		//igaze->checkMotionDone(&motionDone);
		//cout<<"Motion Done: "<<motionDone<<endl;

		if (angles[1] < -26 && !justForFeatureExtraction) /*&& !(exp_state_.experiment_state
		 == aff_msgs::ExperimentState::LET_HUMAN_ACT || exp_state_.experiment_state
		 == aff_msgs::ExperimentState::ACTION))//&& motion != 1)//Internal motion da eklenecek//exp_state_*/
		{
			if(*confidenceCounter < 5 && *confidenceCounter != 0)
			{
				releaseMemory();

				cvCopyImage(bgr_frameLeft,willBeSent.raw_image);
				cvCopyImage(bgr_frameLeft,copyOfGlobBgr);
				
				
				
				features = new CvPoint2D32f[corner_count];
				//Feature Detection
				//for(int a = 0;a < 3;a++)
					findEdgeAndCornerFeatures(0);
				
				cvDilate(imgResult, imgResult, NULL, 1);
				for(int a = 0; a < corner_count;a++)
				{
					cvCircle(copyOfGlobBgr, cvPoint(features[a].x, features[a].y), 4, cvScalar(0, 255, 0), -1, 8);
				}
				cvShowImage("Features",copyOfGlobBgr);
				//cvWaitKey();
				
				
				//cvShowImage("Thresholded",imgResult);
				//cvWaitKey();

				validateFeaturesAsRealCorners();

				groupCloserCorners();
				

				int indexAndDegrees[counter][9];
				findNeighbourOfEachCorner(indexAndDegrees);

				identifyDynamicStructures();
				
				int width = bgr_frameLeft->width;
				int height = bgr_frameLeft->height;
				if (width > 0 && height > 0) {
					segmentObjectsNew(bgr_frameLeft,true);
					cvShowImage("Filtered",copyBgr);
					cvWaitKey();
				}
				
				findAreasAndCorners(indexAndDegrees);
				
				
				for(int a = 0; a < 4;a++)
					for(int b = 0; b < 4;b++)
					{
						willBeFilledCountsPerTurn[*confidenceCounter][a][b] = willBeFilled[a][b];
						singleCornersPerTurn[*confidenceCounter][a][b].x = places[a][b].x;
						singleCornersPerTurn[*confidenceCounter][a][b].y = places[a][b].y;
						if(willBeFilled[a][b])
						{
							willBeFilledCountsTotal[a][b] += 1;
							
							singleCornersMean[a][b].x += singleCornersPerTurn[*confidenceCounter][a][b].x;
							singleCornersMean[a][b].y += singleCornersPerTurn[*confidenceCounter][a][b].y;
							
						}
							
					}
			}
			else if(*confidenceCounter >= 5)
			{
				if(*confidenceCounter == 5)
				{
					int eliminated = 0;
					int count;
					
					for(int a = 0; a < 4;a++)
						for(int b = 0; b < 4;b++)
						{
							if(willBeFilledCountsTotal[a][b] > 3)
							{
								for(int c =0; c<5;c++)
								{
									if(willBeFilledCountsPerTurn[c][a][b] == 1)
									{
										count = 0;
										for(int d = 0; d<5;d++)
										{
											if(c != d && willBeFilledCountsPerTurn[d][a][b] == 1)
											{
												if(getEuclidianDistance(singleCornersPerTurn[c][a][b],singleCornersPerTurn[d][a][b]) > 30.0)
												{
													count++;
												}
											}
										}
										if(count > 2)
										{
											eliminated++;
											//singleCornersPerTurn[c][a][b].x = 0;
											//singleCornersPerTurn[c][a][b].y = 0;
											willBeFilledCountsPerTurn[c][a][b] = 0;
											willBeFilledCountsTotal[a][b]--;
										}
									}
									
										
								}
							}
							
						}
					
					printf("Eliminated: %d\n",eliminated);
					int cornerIdentified = 0;
					int cnt = 0;
					float totalX = 0;
					float totalY = 0;
					for(int a = 0; a < 4;a++)
						for(int b = 0; b < 4;b++)
						{
							if(willBeFilledCountsTotal[a][b] > 2)
							{
								printf("%d %d:",a,b);
								cnt = 0;
								willBeFilled[a][b] = true;
								totalX = 0;
								totalY = 0;
								for(int c =0; c<5;c++)
								{
									if(willBeFilledCountsPerTurn[c][a][b] == 1)
									{
										cnt++;
										printf("%d %d====",(int)singleCornersPerTurn[c][a][b].x,(int)singleCornersPerTurn[c][a][b].y);
										totalX += singleCornersPerTurn[c][a][b].x;
										totalY += singleCornersPerTurn[c][a][b].y;		
									}
								}
								
								
								cornerIdentified++;
								places[a][b].x = totalX/cnt;
								places[a][b].y = totalY/cnt;
								printf("total:%f %f\n",places[a][b].x,places[a][b].y);
								
								
							}
							else
							{
								willBeFilled[a][b] = false;
								places[a][b].x = 0.0;
								places[a][b].y = 0.0;
							}
						}
					
					printf("Giriom\n");
	
					for(int a = 0; a < 4;a++)
						for(int b = 0; b < 4;b++)
						{
							if(willBeFilled[a][b])
							{
								for(int c = 0; c < 4;c++)
									for(int d = 0; d < 4;d++)
									{
										if(willBeFilled[c][d] && a != c && b != d)
										{
											if(getEuclidianDistance(places[a][b],places[c][d]) < 40.0)
											{
												printf("a:%d, b:%d gitti\n",c,d);
												willBeFilled[c][d] = false;
												cornerIdentified--;
											}
											
										}
										
									}
									
										
								
							}
							
						}
						
					printf("Places:\n");
					for(int a = 0; a < 4;a++)
					{
						for(int b = 0; b < 4;b++)
						{
							printf("%d ",willBeFilled[a][b]);
						}
						printf("\n");
					}
					
					printf("Coordinates:\n");
					for(int a = 0; a < 4;a++)
					{
						for(int b = 0; b < 4;b++)
						{
							printf("%f %f===",places[a][b].x,places[a][b].y);
						}
						printf("\n");
					}
					printf("Identified Corners: %d out of 16\n",cornerIdentified);
					for(int a = 0; a < 4;a++)
						for(int b = 0; b < 4;b++)
						{
							if(willBeFilled[a][b])
							{
								printf("%f, %f,\n",places[a][b].x,places[a][b].y);
							}
						}
						
					printf("================\n");
					for(int a = 0; a < 4;a++)
						for(int b = 0; b < 4;b++)
						{
							if(willBeFilled[a][b])
							{
								printf("%f, %f,\n",homograpySource[2*(4*a +b)],homograpySource[2*(4*a +b)+1]);
							}
						}
					printf("Corner Identified: %d\n",cornerIdentified);	
					if(cornerIdentified > 3 && cornerIdentified < 16)
					{
						float world[cornerIdentified*2];
						float image[cornerIdentified*2];
						int counter = 0;
						for(int a = 0; a < 4;a++)
							for(int b = 0; b < 4;b++)
							{
								if(willBeFilled[a][b])
								{
									world[2*counter] = homograpySource[2*(4*a +b)];
									world[2*counter + 1] = homograpySource[2*(4*a +b) + 1];
									image[2*counter] = places[a][b].x;
									image[2*counter + 1] = places[a][b].y;
									counter++;
								}
							}
						CvMat mImage, mWorld;
						cvInitMatHeader(&mImage,  cornerIdentified, 2, CV_32FC1, image);
						cvInitMatHeader(&mWorld, cornerIdentified, 2, CV_32FC1, world); 
						CvMat *h1 = cvCreateMat(3, 3, CV_32FC1); 			
						cvFindHomography(&mWorld, &mImage, h1); 
						//cvInvert(h1,h1,CV_SVD);
						CvMat *pointsOnSource,*destOnPlane;
						pointsOnSource = cvCreateMat(3,1,CV_32FC1);
						destOnPlane = cvCreateMat(3,1,CV_32FC1);
					
						*( (float*)CV_MAT_ELEM_PTR( *pointsOnSource, 2, 0 ) ) = 1.0;
						
						for(int a = 0; a < 4;a++)
							for(int b = 0; b < 4;b++)
							{
								if(!willBeFilled[a][b])
								{
									*( (float*)CV_MAT_ELEM_PTR( *pointsOnSource, 0, 0 ) ) = homograpySource[2*(4*a +b)];
									*( (float*)CV_MAT_ELEM_PTR( *pointsOnSource, 1, 0 ) ) = homograpySource[2*(4*a +b) + 1];
									
									cvGEMM(h1,pointsOnSource,1,NULL,0,destOnPlane,0);
									float element_0_0 = CV_MAT_ELEM( *destOnPlane, float, 0, 0 );
									float element_1_0 = CV_MAT_ELEM( *destOnPlane, float, 1, 0 );
									float element_2_0 = CV_MAT_ELEM( *destOnPlane, float, 2, 0 );
									printf("For %d %d:\n",a,b);
									printf("xCoord: %f\n",element_0_0);
									printf("yCoord: %f\n",element_1_0);
									printf("homoCoord: %f\n",element_2_0);
									if((element_0_0/element_2_0) >= 0 && (element_0_0/element_2_0) < 640 && (element_1_0/element_2_0) >= 0 && (element_1_0/element_2_0) < 480)
									{
										willBeFilled[a][b] = true;
										places[a][b].x = element_0_0/element_2_0;
										places[a][b].y = element_1_0/element_2_0;
										cornerIdentified++;
										printf("Ahanda buldum yeni corner: %d ,%d\n",a,b);
									}
									//cvShowImage("image",bgr_frameLeft);
									//cvWaitKey();
								}
							}
						cvReleaseMat(&h1); 
						cvReleaseMat(&pointsOnSource); 
						cvReleaseMat(&destOnPlane); 
						//cvWaitKey();
					}
					else if(cornerIdentified < 4)
					{
						printf("Counter Sifirlanacak\n");	
						*confidenceCounter = 0;
						for(int a = 0; a < 4;a++)
							for(int b = 0; b < 4;b++)
							{
								willBeFilledCountsTotal[a][b] = 0;
								singleCornersMean[a][b].x = 0.0;
								singleCornersMean[a][b].y = 0.0;
							}
							
						for(int c = 0; c < 5;c++)	
							for(int a = 0; a < 4;a++)
								for(int b = 0; b < 4;b++)
								{
									willBeFilledCountsPerTurn[c][a][b] = 0;
									singleCornersPerTurn[c][a][b].x = 0.0;
									singleCornersPerTurn[c][a][b].y = 0.0;
								}
						
					}
				}

				if(*confidenceCounter != 0)
				{
					//Just tagging on image
					cvCopyImage(bgr_frameLeft,tempImg);
					cvCopyImage(bgr_frameLeft,tempImg1);
					tagTableCornersAndAreas();
					

					//Prepare the Image
					int width = bgr_frameLeft->width;
					int height = bgr_frameLeft->height;
					if (width > 0 && height > 0) {
						segmentObjectsNew(bgr_frameLeft,false);
						//cvShowImage("Filtered",copyBgr);
						//cvWaitKey();
					}

					if (Image2 != NULL) {
						cvReleaseImage(&Image2);
						Image2 = NULL;
					}
					Image2 = cvCreateImage(cvSize(copyBgr->width, copyBgr->height),
							IPL_DEPTH_8U, 3);
					cvZero(Image2);
					cvCopyImage(copyBgr,willBeSent.filtered_image);
					
					 
					//float world[cornerIdentified*2];
					//float image[cornerIdentified*2];
					//int counter = 0;
					//for(int a = 0; a < 4;a++)
						//for(int b = 0; b < 4;b++)
						//{
							//if(willBeFilled[a][b])
							//{
								//world[2*counter] = homograpyVisualization[2*(4*a +b)];
								//world[2*counter + 1] = homograpyVisualization[2*(4*a +b) + 1];
								//image[2*counter] = places[a][b].x;
								//image[2*counter + 1] = places[a][b].y;
								//counter++;
							//}
						//}
					//CvMat mImage, mWorld;
					//cvInitMatHeader(&mImage,   cornerIdentified, 2, CV_32FC1, image);
					//cvInitMatHeader(&mWorld, cornerIdentified, 2, CV_32FC1, world);
					//CvMat *h1 = cvCreateMat(3, 3, CV_32FC1); 			
					//cvFindHomography(&mImage, &mWorld, h1); 
					
					//segmentAndHomographVisualize(h1);
					//cvReleaseMat(&h1); 
					
					//segmentAreaObjects();

					cvCopyImage(bgr_frameLeft,tempImgForROI);
					drawWireFrame(wireFrame, willBeFilled, angles[1], tempImgForROI);//,motion);

					imgOut.wrapIplImage(wireFrame);
					portImgOut->prepare() = imgOut;
					portImgOut->write();

					denemeOut.wrapIplImage(tempImg);
					portDenemeOut->prepare() = denemeOut;
					portDenemeOut->write();

					denemeOut1.wrapIplImage(tempImg1);
					portDenemeOut1->prepare() = denemeOut1;
					portDenemeOut1->write();
					
					
				}
				
			}
			

		} else if(angles[1] >= -26){
			cvCopyImage(bgr_frameLeft,tempImgForROI);
			drawWireFrame(wireFrame, willBeFilled, angles[1], tempImgForROI);//,motion);

			imgOut.wrapIplImage(wireFrame);
			portImgOut->prepare() = imgOut;
			portImgOut->write();
			cvCopyImage(bgr_frameLeft,tempImg);

			cvCopyImage(bgr_frameLeft,tempImg1);

			denemeOut.wrapIplImage(tempImg);
			portDenemeOut->prepare() = denemeOut;
			portDenemeOut->write();

			denemeOut1.wrapIplImage(tempImg1);
			portDenemeOut1->prepare() = denemeOut1;
			portDenemeOut1->write();
		}
	}
}

//void findSizeAndHistograms(int x,int y,IplImage *image)
//{
	//int size = cvOverlayImage(image, images[y + x * 3],wireFramePoints[y + x * 3], false);
	//willBeSent.segmented_areas[y + x * 3] = size;

	//cvClearHist(willBeSent.blueHists[y + x * 3]);
	//cvClearHist(willBeSent.redHists[y + x * 3]);
	//cvClearHist(willBeSent.greenHists[y + x * 3]);
	//imgRed = cvCreateImage(cvGetSize(images[y + x* 3]), 8, 1);
	//imgGreen = cvCreateImage(cvGetSize(images[y + x* 3]), 8, 1);
	//imgBlue = cvCreateImage(cvGetSize(images[y + x* 3]), 8, 1);

	//cvSplit(images[y + x * 3], imgBlue, imgGreen,imgRed, NULL);

	//cvCalcHist(&imgRed, willBeSent.redHists[y + x* 3], 0, 0);
	//cvCalcHist(&imgGreen, willBeSent.greenHists[y+ x * 3], 0, 0);
	//cvCalcHist(&imgBlue, willBeSent.blueHists[y + x	* 3], 0, 0);

	//cvReleaseImage(&imgRed);
	//cvReleaseImage(&imgGreen);
	//cvReleaseImage(&imgBlue);
	//imgRed = NULL;
	//imgGreen = NULL;
	//imgBlue = NULL;
//}

//void segmentAndHomographVisualize(CvMat *h1)
//{
	//cvInvert(h1,h1,CV_SVD);
	
	//IplImage *image = cvCreateImage(cvSize(480,360),IPL_DEPTH_8U,1);
	//cvZero(image);
	//cvDrawLine(image, cvPoint(60, 50), cvPoint(420, 50), cvScalar(255, 255,255), 2);
	//cvDrawLine(image, cvPoint(420, 50), cvPoint(465, 320), cvScalar(255, 255,255), 2);
	//cvDrawLine(image, cvPoint(465, 320), cvPoint(15,320), cvScalar(255, 255,255), 2);
	//cvDrawLine(image, cvPoint(15,320), cvPoint(60,50), cvScalar(255, 255,255), 2);
	//cvDrawLine(image, cvPoint(180,50), cvPoint(165, 320), cvScalar(255, 255,255), 2);
	//cvDrawLine(image, cvPoint(300, 50), cvPoint(315, 320), cvScalar(255, 255,255), 2);
	//cvDrawLine(image, cvPoint(45, 130), cvPoint(435, 130), cvScalar(255, 255,255), 2);
	//cvDrawLine(image, cvPoint(30, 220), cvPoint(450, 220), cvScalar(255, 255,255), 2);
	
	//CvMat *pointsOnSource,*destOnPlane;
	//pointsOnSource = cvCreateMat(3,1,CV_32FC1);
	//destOnPlane = cvCreateMat(3,1,CV_32FC1);
	//*( (float*)CV_MAT_ELEM_PTR( *pointsOnSource, 2, 0 ) ) = 1.0;
	//int result = 0;
	//int minY = 480, maxY = 0, minX = 640, maxX = 0;
	//for(int y= 0;y < 480;y++)
		//for(int x = 0; x < 640;x++)
		//{
			//CvScalar pt = cvGet2D(copyBgr,x,y);
			//if(pt.val[0] != 255)
			//{
				//result = floodFill(x,y,minX,minY,maxX,maxY);
				//if(result == 3)
				//{
					//if((maxX-minX)*(maxY-minY) > 2500)//CAN BE CHANGED
					//{
						//for(int a= minY;a < maxY;a++)
							//for(int b = minX; b < maxX;b++)
							//{
								//pt = cvGet2D(copyBgr,b,a);
								//if(pt.val[0] != 255)
								//{
									//*( (float*)CV_MAT_ELEM_PTR( *pointsOnSource, 0, 0 ) ) = b;
									//*( (float*)CV_MAT_ELEM_PTR( *pointsOnSource, 1, 0 ) ) = a;
									//cvGEMM(h1,pointsOnSource,1,NULL,0,destOnPlane,0);
									//float element_0_0 = CV_MAT_ELEM( *destOnPlane, float, 0, 0 );
									//float element_1_0 = CV_MAT_ELEM( *destOnPlane, float, 1, 0 );
									//float element_2_0 = CV_MAT_ELEM( *destOnPlane, float, 2, 0 );
									//if((element_0_0/element_2_0) >= 0 && (element_0_0/element_2_0) < 480 && (element_1_0/element_2_0) >= 0 && (element_1_0/element_2_0) < 360)
									//{
										
									//}
									//cvSet2D(copyBgr, b, a, cvScalarAll(255));
								//}
								
							//}
						
					//}
				//}
			//}
				
		//}
	
	
	
	
	//cvReleaseMat(&pointsOnSource); 
	//cvReleaseMat(&destOnPlane); 
//}

void segmentObjectsNew(IplImage* bgr_frameLeft,bool forIdentification)
{
	CvScalar ptr;
	CvScalar ptr1;
	if (copyBgr != NULL) {
		cvReleaseImage(&copyBgr);
		copyBgr = NULL;
	}
	copyBgr = cvCreateImage(cvSize(bgr_frameLeft->width, bgr_frameLeft->height), IPL_DEPTH_8U,3);
	cvCopyImage(bgr_frameLeft,copyBgr);
	if (ROIGrayAndBinary != NULL) {
		cvReleaseImage(&ROIGrayAndBinary);
		ROIGrayAndBinary = NULL;
	}
	ROIGrayAndBinary = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);
	cvCvtColor(copyBgr, ROIGrayAndBinary, CV_BGR2GRAY);
	
	for (int a = 0; a < ROIGrayAndBinary->width; a++) {
		for (int b = 0; b < ROIGrayAndBinary->height; b++) {
			if (b < ROIGrayAndBinary->height && b >= 0 && a
					< ROIGrayAndBinary->width && a >= 0) {
				ptr = cvGet2D(ROIGrayAndBinary, b, a);
				if (ptr.val[0] < 200.0 && b < copyBgr->height && b > 0 && a
						< copyBgr->width && a > 0)
					cvSet2D(ROIGrayAndBinary, b, a, cvScalarAll(255));
			}
		}

	}
	
	cvThreshold(ROIGrayAndBinary, ROIGrayAndBinary, 190, 255, CV_THRESH_BINARY);
	cvSmooth(ROIGrayAndBinary, ROIGrayAndBinary, CV_MEDIAN, 9, 9, 0, 0);
	
	hsvImage= cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 3);
	sat = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);
	sat1 = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);
	sat2 = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);
	sat3 = cvCreateImage(cvSize(copyBgr->width, copyBgr->height), IPL_DEPTH_8U, 1);

	cvCvtColor(copyBgr, hsvImage, CV_BGR2HSV);
	cvSetImageCOI(hsvImage, 2);
	cvCopy(hsvImage, sat);
	
	CvScalar sat_min = cvScalar(70, 0, 0, 0);
	CvScalar sat_max = cvScalar(255, 0, 0, 0);
	cvInRangeS(sat, sat_min, sat_max, sat1);
	CvScalar sat_min1 = cvScalar(0, 0, 0, 0);
	CvScalar sat_max1 = cvScalar(1, 0, 0, 0);
	cvInRangeS(sat, sat_min1, sat_max1, sat2);
	cvSmooth(sat1, sat1, CV_MEDIAN, 9, 9, 0, 0);
	cvSmooth(sat2, sat2, CV_MEDIAN, 9, 9, 0, 0);
	
	for (int a = 0; a < sat1->width; a++) {
		for (int b = 0; b < sat1->height; b++) {
			if (b < sat1->height && b > 0 && a < sat1->width && a > 0 && b < sat2->height && b > 0 && a < sat2->width && a > 0) {
				ptr = cvGet2D(sat1, b, a);
				ptr1 = cvGet2D(sat2, b, a);
				int g = ((int) ptr.val[0] | (int) ptr1.val[0]);
				if (b < sat3->height && b > 0 && a < sat3->width && a > 0)
					cvSet2D(sat3, b, a, cvScalarAll(g));
			}
		}

	}
	cvSmooth(sat3, sat3, CV_MEDIAN, 9, 9, 0, 0);

	for (int a = 0; a < copyBgr->width; a++) {
		for (int b = 0; b < copyBgr->height; b++) {
			if (b < copyBgr->height && b > 0 && a < copyBgr->width && a > 0) {
				ptr = cvGet2D(sat3, b, a);
				if (ptr.val[0] == 0 && b < copyBgr->height && b > 0 && a < copyBgr->width && a > 0)
					cvSet2D(copyBgr, b, a, cvScalarAll(255));
			}
		}

	}
	//IplImage *combination = cvCreateImage()
	//int minX,minY,maxX,maxY;
	//if(forIdentification)
	//{
	//	for
	//}
	
	//cvSmooth(copyBgr, copyBgr, CV_MEDIAN, 9, 9, 0, 0);
	cvReleaseImage(&hsvImage);
	cvReleaseImage(&sat);
	cvReleaseImage(&sat1);
	cvReleaseImage(&sat2);
	hsvImage = NULL;
	sat = NULL;
	sat1 = NULL;
	sat2 = NULL;
}

//=================OUT
void percept(int region_id, tabletop_2D_segmentation::Perception2D::Request& request, tabletop_2D_segmentation::Perception2D::Response& response) {

	//		std::cout<<(int)request.arg<<" "<<(int)request.arg2<<" "<<region_id<<std::endl;

	//		percept(region_id);
	//		cv_raw_image_ptr->image = cv::Mat(raw_image);
	//		sensor_msgs::CvBridge::fromIpltoRosImage(willBeSent.raw_image, response.raw_image, "bgr8");
	
	frameLeft = imagePortLeft.read();
	while(frameLeft == NULL)
		frameLeft = imagePortLeft.read();
	IplImage *bgr_frameLeftClone = NULL;
	bool found = false;
	if(region_id != -1 ) {
		if (bgr_frameLeftClone != NULL) {
			cvReleaseImage(&bgr_frameLeftClone);
			bgr_frameLeftClone = NULL;
			
		}
		bgr_frameLeftClone = (IplImage*) frameLeft->getIplImage();
		
		initializePacket(&willBeSent);
		
		segmentObjectsNew(bgr_frameLeftClone,false);
		found = floodFillCenterBased(request.arg, request.arg2, 0, 0);
		int countForRect = 0;
		if (!found) {
			for (int x = -100; x <= 100; x++) {
				for (int y = -100; y <= 100; y++) {
					countForRect++;
					if (copyBgr->height / 2 + y < copyBgr->height && copyBgr->height / 2 + y > 0 && copyBgr->width / 2 + x < copyBgr->width && copyBgr->width / 2 + x > 0) {
						found = floodFillCenterBased(request.arg, request.arg2, x, y);
						if (found) {
							x = 101;
							y = 101;
						}
					}
				}
			}
		}
		willBeSent.faceDetected = detectedFace;
		cvCopyImage(bgr_frameLeftClone,willBeSent.raw_image);
		cvCopyImage(copyBgr,willBeSent.filtered_image);
		
		sensor_msgs::Image::Ptr img_ptr = bridge_->cvToImgMsg(willBeSent.raw_image, "bgr8");
		response.raw_image = *img_ptr;
		img_ptr = bridge_->cvToImgMsg(willBeSent.filtered_image, "bgr8");
		response.filtered_image = *img_ptr;

		sensor_msgs::RegionOfInterest roi;
		roi.width = willBeSent.boundingBox.width;
		roi.height = willBeSent.boundingBox.height;
		roi.x_offset = willBeSent.boundingBox.x;
		roi.y_offset = willBeSent.boundingBox.y;
		response.roi = roi;
		
		response.face_detected = willBeSent.faceDetected;
		response.ooi_area = willBeSent.segmented_area;
		response.ooi_color_r_hist.resize(N_BINS);
		response.ooi_color_g_hist.resize(N_BINS);
		response.ooi_color_b_hist.resize(N_BINS);

		for (uint8_t i = 0; i < N_BINS; i++) {
			response.ooi_color_r_hist[i] = cvQueryHistValue_1D(willBeSent.redHist, i);
			response.ooi_color_g_hist[i] = cvQueryHistValue_1D(willBeSent.greenHist, i);
			response.ooi_color_b_hist[i] = cvQueryHistValue_1D(willBeSent.blueHist, i);
		}
		
		if (willBeSent.segmented_area != -1)
			found = true;
		
	
	}
	else if(region_id == -1)
	{
		cout << "=========OBJECT NOT PRESENT===========" << endl;
	}
	
	
	if (region_id == -1 && request.task == tabletop_2D_segmentation::Perception2D::Request::EXTRACT_EFFECT) {
		cout << "region -1" << endl;

		cout << "SIZES:" << willBeSentOld.segmented_area
				<< "and " << willBeSent.segmented_area << endl;
		cout << "REGION x:" << willBeSentOld.boundingBox.x
				<< "and " << willBeSent.boundingBox.x << endl;
	}
	if (found && request.task == tabletop_2D_segmentation::Perception2D::Request::EXTRACT_EFFECT) {

		cout<<"Final Object Found and Copied:"<<endl;
		cout<<"Face Detected"<<willBeSentOld.faceDetected<<endl;
		cout<<"Object Size"<<willBeSent.segmented_area<<endl;
		cout<<"Object Region X: "<<willBeSent.boundingBox.x<<", Y:"<<willBeSent.boundingBox.y<<", Height:"<<willBeSent.boundingBox.height<<", Width:"<<willBeSent.boundingBox.width<<endl;
		cout << "SIZES:" << willBeSentOld.segmented_area
				<< "and " << willBeSent.segmented_area << endl;
		cout << "REGION x:" << willBeSentOld.boundingBox.x
				<< "and " << willBeSent.boundingBox.x << endl;
	}
	if (found && request.task == tabletop_2D_segmentation::Perception2D::Request::DO_PERCEPT) {
		
		initializePacket(&willBeSentOld);
		
		willBeSentOld.faceDetected = detectedFace;
		cvCopyImage(bgr_frameLeftClone,willBeSentOld.raw_image);
		cvCopyImage(copyBgr,willBeSentOld.filtered_image);
		
		willBeSentOld.blueHist = willBeSent.blueHist;
		willBeSentOld.redHist = willBeSent.redHist;
		willBeSentOld.greenHist = willBeSent.greenHist;
		
		willBeSentOld.segmented_area = willBeSent.segmented_area;
		willBeSentOld.boundingBox = willBeSent.boundingBox;
		oldRegionId = region_id;
		cout<<"INITIAL Object Found and Copied:"<<endl;
		cout<<"Face Detected"<<willBeSentOld.faceDetected<<endl;
		cout<<"Object Size"<<willBeSentOld.segmented_area<<endl;
		cout<<"Object Region X: "<<willBeSentOld.boundingBox.x<<", Y:"<<willBeSentOld.boundingBox.y<<", Height:"<<willBeSentOld.boundingBox.height<<", Width:"<<willBeSentOld.boundingBox.width<<endl;
		
		
	}
	if (!found && region_id != -1 ) {
		cout << "Region ID:" << region_id << endl;
		cout << "=========OBJECT NOT FOUND===========" << endl;
	}

}

bool perception2DCallback( tabletop_2D_segmentation::Perception2D::Request& request, tabletop_2D_segmentation::Perception2D::Response& response) {

	std::cout << "**********************" << std::endl;
	std::cout << " service called" << std::endl;
	std::cout << "**********************" << std::endl;
	if (request.task == tabletop_2D_segmentation::Perception2D::Request::DO_PERCEPT) {
		int region_id = request.arg * 3 + request.arg2;
		percept(region_id, request, response);
		sensor_msgs::Image::Ptr raw_ptr = bridge_->cvToImgMsg(willBeSentOld.raw_image, "bgr8");
		sensor_msgs::Image::Ptr filt_ptr = bridge_->cvToImgMsg(willBeSentOld.filtered_image, "bgr8");
		// ++Onur
		cout<<"REGION ID:"<<region_id<<endl;
		cout<<"FACEEEE:"<<willBeSentOld.faceDetected<<endl;
		std::cout << "T2D@ DO_PERCEPT: Logging started" << std::endl;
		imageLoggingOp(imgTuple,region_id,raw_ptr, filt_ptr);
		imageLogger->logSingleData(&imgTuple,experimentEpoch,0);
		std::cout << "Logging done" << std::endl;

		// --Onur
	} else if (request.task == tabletop_2D_segmentation::Perception2D::Request::EXTRACT_EFFECT) {
		int region_id = request.arg * 3 + request.arg2;
		bool objPresent = true;
		cout<<"REGION ID:"<<region_id<<endl;
		if (region_id != -1) {
			percept(region_id, request, response);
		} else {
			objPresent = false;
			newFace = willBeSent.faceDetected;
			cvCopyImage(willBeSent.raw_image,rawForDisappear);
			cvCopyImage(willBeSent.filtered_image,filteredForDisappear);
			initializePacket(&willBeSent);
			willBeSent.faceDetected =newFace;
			cvCopyImage(rawForDisappear,willBeSent.raw_image);
			cvCopyImage(filteredForDisappear,willBeSent.filtered_image);
			percept(region_id, request, response);
			region_id = oldRegionId;
		}
		cout<<"FACEEEE:"<<willBeSent.faceDetected<<endl;
		std::cout << "T2D@ EXTRACT_EFFECT: Final Logging started" << std::endl;
		// ++Onur: Final features & effect logging

		if(willBeSent.raw_image == NULL)
		{
			std::cout << "Houston; we can not have a null image. willBeSentOld has null raw_image" << std::endl;
		}
		if(willBeSent.filtered_image == NULL)
		{
			std::cout << "Houston; we can not have a null image. willBeSentOld has null filtered_image" << std::endl;
		}



		sensor_msgs::Image::Ptr raw_ptr = bridge_->cvToImgMsg(willBeSent.raw_image, "bgr8");
		sensor_msgs::Image::Ptr filt_ptr = bridge_->cvToImgMsg(willBeSent.filtered_image, "bgr8");

		if(region_id < 0)
		{
			region_id = oldRegionId;
		}

		std::cout << "T2D@ EXTRACT_EFFECT: Before imageLoggingOp with region id: " << region_id << std::endl;
		imageLoggingOp(imgEffectTuple,region_id,raw_ptr, filt_ptr);

		std::cout << "T2D@ EXTRACT_EFFECT: imageLoggingOp passed." << std::endl;
		//imageLogger->logSingleData(&imgEffectTuple,region_id);
		std::cout << "T2D@ EXTRACT_EFFECT: logging..." << std::endl;
//		if(region_id >= 0)
//			imageLogger->logSingleData(&imgTuple,region_id);
//		else
//			std::cout << "T2D@ EXTRACT_EFFECT: disappear?..." << std::endl;
		//Extract Features between willbesent and willbesentOld neye yollayacagim

		// The effect logging will commence here, i.e., when the object is properly found...
		std::cout << "T2D@ EXTRACT_EFFECT: Effect Logging started" << std::endl;
		if(objPresent)
		{
			computeEffect();
		}
		imageEffectLogger->logSingleData(&imgEffectTuple,experimentEpoch,1);

		experimentEpoch++;
		
		std::cout << "Experiment Epoch incremented to " << experimentEpoch << std::endl;
		//raw_ptr = 0;
		//filt_ptr = 0;
		// --Onur
	}
	std::cout<<"******** CIKTIM ***************"<<std::endl;
	return true;
}

//OUT
void imageLoggingOp(ImageFeatureTuple &imgTuple, int region_id,sensor_msgs::Image::Ptr raw, sensor_msgs::Image::Ptr filtered)
{
		/*
			The information to store are as follows:

			sensor_msgs/Image raw_image
			sensor_msgs/Image filtered_image
			sensor_msgs/RegionOfInterest rois
			bool face_detected
			int16 ooi_area
			int8[] ooi_color_r_hist
			int8[] ooi_color_g_hist
			int8[] ooi_color_b_hist
		*/

		// raw image
		/*imgTuple.rawimg_header_seq = raw->header.seq;
		imgTuple.rawimg_header_stamp = raw->header.stamp;
		imgTuple.rawimg_header_frame_id = raw->header.frame_id;*/

		imgTuple.rawimg_height = raw->height;
		imgTuple.rawimg_width = raw->width;
		imgTuple.rawimg_step = raw->step;

		if(imgTuple.rawimg_data == NULL)
		{
			imgTuple.rawimg_data = new int[raw->height*raw->step];
		}

		for(int i = 0; i < raw->height*raw->step; i++)
		{
			imgTuple.rawimg_data[i] = (int)raw->data[i];
		}
		/*imgTuple.rawimg_is_bigendian = raw->is_bigendian;
		imgTuple.rawimg_encoding = raw->encoding;*/

		// filtered image
		/*imgTuple.filimg_header_seq = filtered->header.seq;
		imgTuple.filimg_header_stamp = filtered->header.stamp;
		imgTuple.filimg_header_frame_id = filtered->header.frame_id;*/


		imgTuple.filimg_height = filtered->height;
		imgTuple.filimg_width = filtered->width;
		imgTuple.filimg_step = filtered->step;

		if(imgTuple.filimg_data == NULL)
		{
			imgTuple.filimg_data = new int[filtered->height*filtered->step];
		}

		for(int i = 0; i < filtered->height*filtered->step; i++)
		{
			imgTuple.filimg_data[i] = (int)filtered->data[i];
		}
				//(int*)&filtered->data[0];
		/*imgTuple.filimg_is_bigendian = filtered->is_bigendian;
		imgTuple.filimg_encoding = filtered->encoding;*/

		// region of interest information
		imgTuple.roi_width = willBeSent.boundingBox.width;
		imgTuple.roi_height = willBeSent.boundingBox.height;
		imgTuple.roi_x_offset = willBeSent.boundingBox.x;
		imgTuple.roi_y_offset = willBeSent.boundingBox.y;


		// Whether a face was detected at this instance:
		imgTuple.face_detected = willBeSent.faceDetected;

		// ooi_area
		imgTuple.ooi_area = willBeSent.segmented_area;

		imgTuple.ooi_color_r_hist.resize(N_BINS);
		imgTuple.ooi_color_g_hist.resize(N_BINS);
		imgTuple.ooi_color_b_hist.resize(N_BINS);


		for (uint8_t i = 0; i < N_BINS; i++)
		{
			imgTuple.ooi_color_r_hist[i]
					= cvQueryHistValue_1D(willBeSent.redHist, i);
			imgTuple.ooi_color_g_hist[i]
					= cvQueryHistValue_1D(willBeSent.greenHist, i);
			imgTuple.ooi_color_b_hist[i]
					= cvQueryHistValue_1D(willBeSent.blueHist, i);
		}

		// log the data using void DataLogger::logSingleData(ImageFeatureTuple *currentImgInfo, int label);

}
//OUT

//OUT
// Uses imgTuple as a reference to initial features of the object
void computeEffect()
{
		// TODO: Discuss for whether the size changes!!!!!


		if(imgEffectTuple.rawimg_height * imgEffectTuple.rawimg_step != imgTuple.rawimg_height * imgTuple.rawimg_step)
		{
			std::cout << "The data sizes for raw img of init & final img tuples do not match" << endl;
		}
		int i;
		for(i = 0; i < imgEffectTuple.rawimg_height * imgEffectTuple.rawimg_step; i++)
		{
			imgEffectTuple.rawimg_data[i] -= imgTuple.rawimg_data[i];
		}

		imgEffectTuple.rawimg_height 	-= imgTuple.rawimg_height;
		imgEffectTuple.rawimg_width 	-= imgTuple.rawimg_width;
		imgEffectTuple.rawimg_step 	-= imgTuple.rawimg_step;

		// TODO: Discuss for whether the size changes!!!!!

		if(imgEffectTuple.filimg_height * imgEffectTuple.filimg_step != imgTuple.filimg_height * imgTuple.filimg_step)
		{
			std::cout << "The data sizes for filtered img of init & final img tuples do not match" << endl;
		}

		for(i = 0; i < imgEffectTuple.filimg_height * imgEffectTuple.filimg_step; i++)
		{
			imgEffectTuple.filimg_data[i] -= imgTuple.filimg_data[i];
		}

		imgEffectTuple.filimg_height 	-= imgTuple.filimg_height;
		imgEffectTuple.filimg_width 	-= imgTuple.filimg_width;
		imgEffectTuple.filimg_step 	-= imgTuple.filimg_step;

		// region of interest information
		imgEffectTuple.roi_width 	-= imgTuple.roi_width;
		imgEffectTuple.roi_height 	-= imgTuple.roi_height;
		imgEffectTuple.roi_x_offset 	-= imgTuple.roi_x_offset;
		imgEffectTuple.roi_y_offset 	-= imgTuple.roi_y_offset;

		imgEffectTuple.face_detected = imgEffectTuple.face_detected ^ imgTuple.face_detected;

		imgEffectTuple.ooi_area = imgEffectTuple.ooi_area - imgTuple.ooi_area; // i.e., 0 or 1

		for (uint8_t i = 0; i < N_BINS; i++)
		{
			imgEffectTuple.ooi_color_r_hist[i] -= imgTuple.ooi_color_r_hist[i];
			imgEffectTuple.ooi_color_g_hist[i] -= imgTuple.ooi_color_g_hist[i];
			imgEffectTuple.ooi_color_b_hist[i] -= imgTuple.ooi_color_b_hist[i];
		}

}

void expStateCallback(aff_msgs::ExperimentState::ConstPtr exp_state) {
	first_srv_callback = true;
	exp_state_ = *exp_state;
}
//===========OUT

//OUT
//void perceptOld(int region_id,
		//tabletop_2D_segmentation::Perception2D::Request& request,
		//tabletop_2D_segmentation::Perception2D::Response& response) {

	////		std::cout<<(int)request.arg<<" "<<(int)request.arg2<<" "<<region_id<<std::endl;

	////		percept(region_id);
	////		cv_raw_image_ptr->image = cv::Mat(raw_image);
	////		sensor_msgs::CvBridge::fromIpltoRosImage(willBeSent.raw_image, response.raw_image, "bgr8");
	//bool found = false;
	//cout<<" hede hodoooo: "<<willBeSent.segmented_areas[region_id]<<endl;
	//if(willBeSent.segmented_areas[region_id] != -1)
		//found = true;

	//while (region_id != -1 && willBeSent.segmented_areas[region_id] == -1
			//&& nh->ok() && perceptTrying < 5) {
		//cout << "width: " << copyBgr->width << " , height: " << copyBgr->height
				//<< endl;
		//perceptTrying++;
		//segmentTabletopObjects();

		//sensor_msgs::Image::Ptr img_ptr = bridge_->cvToImgMsg(
				//willBeSent.raw_image, "bgr8");

		////cvShowImage("asd", bridge_->imgMsgToCv((sensor_msgs::Image::ConstPtr)img_ptr, "bgr8"));
		////		cvWaitKey(0);
		//response.raw_image = *img_ptr;
		//img_ptr = bridge_->cvToImgMsg(willBeSent.filtered_image, "bgr8");
		//response.filtered_image = *img_ptr;

		//response.rois.resize(9);
		//for (uint8_t i = 0; i < willBeSent.boundingBoxes.size(); i++) {
			//sensor_msgs::RegionOfInterest roi;
			//roi.width = willBeSent.boundingBoxes[i].width;
			//roi.height = willBeSent.boundingBoxes[i].height;
			//roi.x_offset = willBeSent.boundingBoxes[i].x;
			//roi.y_offset = willBeSent.boundingBoxes[i].y;
			//response.rois[i] = roi;
		//}

		//response.face_detected = willBeSent.faceDetected;
		//response.ooi_area = willBeSent.segmented_areas[region_id];
		//response.ooi_color_r_hist.resize(N_BINS);
		//response.ooi_color_g_hist.resize(N_BINS);
		//response.ooi_color_b_hist.resize(N_BINS);

		//for (uint8_t i = 0; i < N_BINS; i++) {
			//response.ooi_color_r_hist[i]
					//= cvQueryHistValue_1D(willBeSent.redHists[region_id], i);
			//response.ooi_color_g_hist[i]
					//= cvQueryHistValue_1D(willBeSent.greenHists[region_id], i);
			//response.ooi_color_b_hist[i]
					//= cvQueryHistValue_1D(willBeSent.blueHists[region_id], i);
		//}
		//if (willBeSent.segmented_areas[region_id] != -1)
			//found = true;
		//ros::spinOnce();
	//}
	//while (perceptTrying > 4 && !found && region_id != -1 && nh->ok()) {
		//cout << "viy viy" << endl;
		//found = floodFillCenterBased(request.arg, request.arg2, 0, 0);
		//cout<<"center flood fill sonucu: "<<found<<endl;
		//int countForRect = 0;
		//if (!found) {
			//for (int x = -100; x <= 100; x++) {
				//for (int y = -100; y <= 100; y++) {
					//countForRect++;
					//if (copyBgr->height / 2 + y < copyBgr->height
							//&& copyBgr->height / 2 + y > 0 && copyBgr->width
							/// 2 + x < copyBgr->width && copyBgr->width / 2 + x
							//> 0) {
						//found = floodFillCenterBased(request.arg, request.arg2,
								//x, y);
						//if (found) {
							//x = 101;
							//y = 101;
						//}
					//}
					////cvSet2D(src, y+location.y - overlay->height/2, x+location.x- overlay->width/2, over);
				//}
			//}
		//}
		//cout<<"countForRect: "<<countForRect<<endl;
		//ros::spinOnce();
	//}
	//if (region_id == -1 && request.task
			//== tabletop_2D_segmentation::Perception2D::Request::EXTRACT_EFFECT) {
		//cout << "region -1" << endl;

		//cout << "SIZES:" << willBeSentOld.segmented_areas[oldRegionId]
				//<< "and " << willBeSent.segmented_areas[region_id] << endl;
		//cout << "REGION x:" << willBeSentOld.boundingBoxes[oldRegionId].x
				//<< "and " << willBeSent.boundingBoxes[region_id].x << endl;
	//}
	//if (found && request.task
			//== tabletop_2D_segmentation::Perception2D::Request::EXTRACT_EFFECT) {

		//cout << "SIZES:" << willBeSentOld.segmented_areas[oldRegionId]
				//<< "and " << willBeSent.segmented_areas[region_id] << endl;
		//cout << "REGION x:" << willBeSentOld.boundingBoxes[oldRegionId].x
				//<< "and " << willBeSent.boundingBoxes[region_id].x << endl;
	//}
	//if (request.task
			//== tabletop_2D_segmentation::Perception2D::Request::DO_PERCEPT
			//&& found) {
		//cout<<"Found and Copied"<<endl;
		//oldRegionId = region_id;
		////memcpy(&willBeSentOld, &willBeSent, sizeof(Packet));
		//willBeSentOld.faceDetected = willBeSent.faceDetected;

		//cvCopyImage(willBeSentOld.raw_image,willBeSent.raw_image);
		//cvCopyImage(willBeSentOld.filtered_image,willBeSent.filtered_image);
		//for(int x = 0;x < willBeSentOld.blueHists.size();x++)
		//{
			//willBeSentOld.blueHists[x] = willBeSent.blueHists[x];
		//}
		//for(int x = 0;x < willBeSentOld.segmented_areas.size();x++)
		//{
			//willBeSentOld.segmented_areas[x] = willBeSent.segmented_areas[x];
		//}
		//for(int x = 0;x < willBeSentOld.redHists.size();x++)
		//{
			//willBeSentOld.redHists[x] = willBeSent.redHists[x];
		//}

		//for(int x = 0;x < willBeSentOld.greenHists.size();x++)
		//{
			//willBeSentOld.greenHists[x] = willBeSent.greenHists[x];
		//}

		//for(int x = 0;x < willBeSentOld.boundingBoxes.size();x++)
		//{
			//willBeSentOld.boundingBoxes[x] = willBeSent.boundingBoxes[x];
		//}

	//}
	//while (!found && region_id != -1 && nh->ok()) {
		//cout << "Region ID:" << region_id << endl;
		//cout << "bulamadim hic bisi" << endl;
		//ros::spinOnce();
	//}

//}
//OUT

////OUT
//void expStateCallback(aff_msgs::ExperimentState::ConstPtr exp_state) {
	//first_srv_callback = true;
	//exp_state_ = *exp_state;
//}
//OUT

//OUT
//bool perception2DCallbackOld(
//		tabletop_2D_segmentation::Perception2D::Request& request,
//		tabletop_2D_segmentation::Perception2D::Response& response) {
//
//	std::cout << "**********************" << std::endl;
//	std::cout << " service called" << std::endl;
//	std::cout << "**********************" << std::endl;
//	if (request.task
//			== tabletop_2D_segmentation::Perception2D::Request::DO_PERCEPT) {
//		int region_id = request.arg * 3 + request.arg2;
//		perceptTrying = 0;
//		percept(region_id, request, response);
//		sensor_msgs::Image::Ptr raw_ptr = bridge_->cvToImgMsg(willBeSentOld.raw_image, "bgr8");
//		sensor_msgs::Image::Ptr filt_ptr = bridge_->cvToImgMsg(willBeSentOld.filtered_image, "bgr8");
//		// ++Onur
//		cout<<"REGION ID:"<<region_id<<endl;
//		cout<<"FACEEEE:"<<willBeSentOld.faceDetected<<endl;
//		std::cout << "T2D@ DO_PERCEPT: Logging started" << std::endl;
//		imageLoggingOp(imgTuple,region_id,raw_ptr, filt_ptr);
//		imageLogger->logSingleData(&imgTuple,region_id);
//		std::cout << "Logging done" << std::endl;
//
//		// --Onur
//	} else if (request.task
//			== tabletop_2D_segmentation::Perception2D::Request::EXTRACT_EFFECT) {
//		int region_id = request.arg * 3 + request.arg2;
//		bool objPresent = true;
//	cout<<"REGION ID:"<<region_id<<endl;
//		if (region_id != -1) {
//			perceptTrying = 0;
//			percept(region_id, request, response);
//
//
//		} else {
//
//			objPresent = false;
//			perceptTrying = 0;
//			newFace = willBeSent.faceDetected;
//			cvCopyImage(willBeSent.raw_image,rawForDisappear);
//			cvCopyImage(willBeSent.filtered_image,filteredForDisappear);
//			initializePacket(&willBeSent);
//			willBeSent.faceDetected =newFace;
//			cvCopyImage(rawForDisappear,willBeSent.raw_image);
//			cvCopyImage(filteredForDisappear,willBeSent.filtered_image);
//			percept(region_id, request, response);
//			region_id = oldRegionId;
//		}
//		cout<<"FACEEEE:"<<willBeSent.faceDetected<<endl;
//		std::cout << "T2D@ EXTRACT_EFFECT: Final Logging started" << std::endl;
//		// ++Onur: Final features & effect logging
//
//		if(willBeSent.raw_image == NULL)
//		{
//			std::cout << "Houston; we can not have a null image. willBeSentOld has null raw_image" << std::endl;
//		}
//		if(willBeSent.filtered_image == NULL)
//		{
//			std::cout << "Houston; we can not have a null image. willBeSentOld has null filtered_image" << std::endl;
//		}
//
//
//
//		sensor_msgs::Image::Ptr raw_ptr = bridge_->cvToImgMsg(willBeSent.raw_image, "bgr8");
//		sensor_msgs::Image::Ptr filt_ptr = bridge_->cvToImgMsg(willBeSent.filtered_image, "bgr8");
//
//		if(region_id < 0)
//		{
//			region_id = oldRegionId;
//		}
//
//		std::cout << "T2D@ EXTRACT_EFFECT: Before imageLoggingOp with region id: " << region_id << std::endl;
//		imageLoggingOp(imgEffectTuple,region_id,raw_ptr, filt_ptr);
//
//		std::cout << "T2D@ EXTRACT_EFFECT: imageLoggingOp passed." << std::endl;
//		imageLogger->logSingleData(&imgEffectTuple,region_id);
//		std::cout << "T2D@ EXTRACT_EFFECT: logging..." << std::endl;
////		if(region_id >= 0)
////			imageLogger->logSingleData(&imgTuple,region_id);
////		else
////			std::cout << "T2D@ EXTRACT_EFFECT: disappear?..." << std::endl;
//		//Extract Features between willbesent and willbesentOld neye yollayacagim
//
//		// The effect logging will commence here, i.e., when the object is properly found...
//		std::cout << "T2D@ EXTRACT_EFFECT: Effect Logging started" << std::endl;
//		if(objPresent)
//		{
//			computeEffect();
//		}
//		imageEffectLogger->logSingleData(&imgEffectTuple,region_id);
//
//		//raw_ptr = 0;
//		//filt_ptr = 0;
//		// --Onur
//	}
//	std::cout<<"******** CIKTIM ***************"<<std::endl;
//	return true;
//}
//OUT


