#include "ImageDetector.h"

using namespace cv;
using namespace std;



ImageDetector::ImageDetector(int threshold_, int scoret1_, int scoret2_)
{
	threshold = threshold_;
	scoret1 = scoret1_;
	scoret2 = scoret2_;

	//When People detector is created, we initialize our HOG detector
	hog = new HOGDescriptor(/*Size(64,128), Size(16,16), Size(8,8), Size(8,8)*/);
	hog->setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

	//Load classifiers Tier1
	classifTier1.push_back(CascadeClassifier("/usr/share/opencv/haarcascades/haarcascade_upperbody.xml"));
	classifTier1.push_back(CascadeClassifier("/usr/share/opencv/haarcascades/haarcascade_mcs_upperbody.xml"));

	//Load classifiers Tier2
	classifTier2.push_back(CascadeClassifier("/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"));
	classifTier2.push_back(CascadeClassifier("/usr/share/opencv/haarcascades/haarcascade_profileface.xml"));
	classifTier2.push_back(CascadeClassifier("/usr/share/opencv/haarcascades/haarcascade_mcs_nose.xml"));
	classifTier2.push_back(CascadeClassifier("/usr/share/opencv/haarcascades/haarcascade_mcs_mouth.xml"));
	classifTier2.push_back(CascadeClassifier("/usr/share/opencv/haarcascades/haarcascade_mcs_eyepair_small.xml"));

	
	return;
}

ImageDetector::~ImageDetector()
{
	delete hog;
	return;
}

//detectPeople runs different kind of detector given a ROI from the laser data
//As soon as one detector give us a positive on the ROI we assume there is a person
int ImageDetector::detectPerson(Mat frame)
{
	int score = 0;
	vector<Rect> found;

	//---------------FIRST STAGE: HOG-----------
	//Run the hog detector with a smaller windows size. We check first if the ROI is big enough to run the detection
	if(frame.cols >= 64 && frame.rows >= 128)
		hog->detectMultiScale(frame, found, 0, Size(8,8));

	//If the HOG was not succesfull run cascade upperbody cascade classifier
	if(!found.empty() || detectROI(frame(Rect(0,0,frame.cols,(int) (3.0/4.0*(float)frame.rows))),classifTier1))
	{
		score++;
		score += detectROI(frame(Rect(0,0,frame.cols, (int) (1.0/4.0*((float)frame.rows)))),classifTier2);
		
	}
	return score;
}

//detectPeole runs different kind of detectors in the whole picture
//Used when there are no possible candidates from laser data 
void ImageDetector::detectPeople(Mat frame, vector<Rect>& found)
{
	//---------------FIRST STAGE: HOG-----------
	//Run the hog detector with default parameters
	hog->detectMultiScale(frame, found, 0, Size(8,8),Size(0,0),1.05,3);
	found = filterRects(found);

	return;
}

int ImageDetector::detectROI(Mat roi, vector<CascadeClassifier>& classifiers)
{
	int score = 0;
	vector<Rect> foundRoi;
	cv::Mat frame_gray;
	
	
	cvtColor(roi, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	for(int i = 0; i<classifiers.size(); i++)
	{
		classifiers[i].detectMultiScale(frame_gray, foundRoi, 1.1, 3, 0|CV_HAAR_DO_CANNY_PRUNING, Size(30,30));
		score += (foundRoi.empty()) ? 0:1;
	}
	return score;
}

inline void ImageDetector::printROI(Mat roi, vector<Rect>& found)
{
	for(int i=0; i< found.size(); i++)
		rectangle(roi,found[i], Scalar(0,255,0));
	return;	
}

vector<cv::Rect> ImageDetector::filterRects(vector<cv::Rect>& ROI)
{
	vector<cv::Rect> ROI_filtered;

	int i,j;
	for(i=0; i< ROI.size(); i++)
	{
		cv::Rect r = ROI[i];
		for(j=0; j< ROI.size(); j++)
			if(j!=i && (r & ROI[j]) == r)
				break;
		if(j== ROI.size())
			ROI_filtered.push_back(r);
	}
	return ROI_filtered;
}

//Compare two histograms using the Bhattacharyya distance
//Returns the similarity as a number between 0 and 1
double ImageDetector::compareHistogram(cv::Mat img1, cv::Mat img2)
{
	Mat img1_hsv, img2_hsv;

	//Convert images to HSV format
	cvtColor(img1, img1_hsv, COLOR_BGR2HSV); 
	cvtColor(img2, img2_hsv, COLOR_BGR2HSV);

	int h_bins=50;
	int s_bins=60;
	int histsize[] = {h_bins, s_bins};
	
	float h_ranges[] = {0,180};
	float s_ranges[] = {0,256};

	const float* ranges[] = { h_ranges, s_ranges };

    	// Use the o-th and 1-st channels
    	int channels[] = { 0, 1 };

	MatND hist1, hist2;

	//Calculate the histograms for the HSV images
	calcHist(&img1_hsv, 1, channels, Mat(), hist1, 2, histsize, ranges, true, false);	
	normalize(hist1, hist1, 0,1, NORM_MINMAX, -1, Mat());

	calcHist(&img2_hsv, 1, channels, Mat(), hist2, 2, histsize, ranges, true, false);	
	normalize(hist2, hist2, 0,1, NORM_MINMAX, -1, Mat());

	return compareHist(hist1, hist2, CV_COMP_BHATTACHARYYA);
}

