#ifndef FACE_TRACKER_H
#define FACE_TRACKER_H

#include "../FaceTracker-master/include/FaceTracker/Tracker.h"

class FaceTracker
{

public:
	FaceTracker () {
		face_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
		eyes_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
		nose_cascade_name = "../cascades/Nose.xml";
		mouth_cascade_name = "../cascades/Mouth.xml";
	}

	void detectFace (cv::Mat frame) {
		//parse command line arguments
		string filesDir = "../FaceTracker-master/model/";
		string ftFile = filesDir + "face2.tracker";
		string triFile = filesDir + "face.con";
		string conFile = filesDir + "face.tri";
		bool fcheck = false; double scale = 1; int fpd = -1; bool show = true;

		//set other tracking parameters
		std::vector<int> wSize1(1); wSize1[0] = 7;
		std::vector<int> wSize2(3); wSize2[0] = 11; wSize2[1] = 9; wSize2[2] = 7;
		int nIter = 5; double clamp=3,fTol=0.01; 
		FACETRACKER::Tracker model(ftFile);
		cv::Mat tri=FACETRACKER::IO::LoadTri(triFile);
		cv::Mat con=FACETRACKER::IO::LoadCon(conFile);

		//initialize camera and display window
		cvNamedWindow("Face Tracker",1);
		std::cout << "Hot keys: "        << std::endl
		<< "\t ESC - quit"     << std::endl
		<< "\t d   - Redetect" << std::endl;

		//loop until quit (i.e user presses ESC)
		bool failed = true;
		cv::Mat im;
		while(1){ 
			//grab image, resize and flip
			if(scale == 1)im = frame; 
			else cv::resize(frame,im,cv::Size(scale*frame.cols,scale*frame.rows));
			cv::flip(im,im,1); cv::cvtColor(im,gray,CV_BGR2GRAY);

			//track this image
			std::vector<int> wSize; if(failed)wSize = wSize2; else wSize = wSize1; 
			if(model.Track(gray,wSize,fpd,nIter,clamp,fTol,fcheck) == 0){
				int idx = model._clm.GetViewIdx(); failed = false;
				Draw(im,model._shape,con,tri,model._clm._visi[idx]); 
			}else{
				if(show){cv::Mat R(im,cvRect(0,0,150,50)); R = cv::Scalar(0,0,255);}
				model.FrameReset(); failed = true;
			}     
			
			//show image and check for user input
			imshow("Face Tracker",im); 
			int c = cvWaitKey(10);
			if(c == 27)break; else if(char(c) == 'd')model.FrameReset();
		}return 0;
	}
};

#endif