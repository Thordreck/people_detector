#ifndef IMAGE_DETECTOR_H
#define IMAGE_DETECTOR_H 

#include "opencv2/opencv.hpp"

class ImageDetector
{
		cv::HOGDescriptor*			hog;					//Hog descriptor used for full body detection
		int 					threshold, scoret1, scoret2;
		std::vector<cv::CascadeClassifier>	classifTier1, classifTier2;
		std::vector<cv::Rect>			found;					//Rectangle containing found person
		int					detectROI(cv::Mat roi, std::vector<cv::CascadeClassifier>& classifiers);
		inline void 				printROI(cv::Mat roi, std::vector<cv::Rect>& found);
		std::vector<cv::Rect>			filterRects(std::vector<cv::Rect>& ROI);

	public:
			ImageDetector(int threshold = 1, int scoret1 = 1, int scoret2 = 1);
			~ImageDetector();
		void	detectPeople(cv::Mat frame, std::vector<cv::Rect>& found);
		int	detectPerson(cv::Mat frame);
		double	compareHistogram(cv::Mat img1, cv::Mat img2);			

};

#endif
