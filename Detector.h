#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "MyFaceTracker/FaceTracker/Tracker.h"	// detecção de features em imagens usando programa externo

#define COLOR_BLUE		cv::Scalar(255, 0, 0)
#define COLOR_GREEN		cv::Scalar(0, 255, 0)
#define COLOR_RED		cv::Scalar(0, 0, 255)
#define COLOR_YELLOW	cv::Scalar(0, 255, 255)

class Detector
{
	FACETRACKER::Tracker* tracker;
	cv::RNG rng;

	// realiza tracking da face com parâmetros default
	int trackFace(cv::Mat gray_image, vector<int> wSize) {
		return tracker->Track(gray_image, wSize, -1, 10, 3.0, 0.01, true);
	}

public:
	Detector() {
		tracker = new FACETRACKER::Tracker("../src/MyFaceTracker/model/face2.tracker");
	}

	// faz tracking da face e retorna bounding box do rosto, caso encontre
	cv::Rect trackFace(cv::Mat &frameBGR) {
		cv::Rect face;
		cv::Mat gray_image;
		cv::cvtColor(frameBGR, gray_image, CV_BGR2GRAY);
		vector<int> wSize1(1); wSize1[0] = 7;
		vector<int> wSize2(3); wSize2[0] = 11; wSize2[1] = 9; wSize2[2] = 7;

		if (trackFace(gray_image, wSize1) == 0) {
			int idx = tracker->_clm.GetViewIdx();
			Util::Draw(frameBGR, tracker->_shape, tracker->_clm._visi[idx]);
			face = tracker->_rect;
			// cv::rectangle(frameBGR, face, COLOR_BLUE);
		} else if (trackFace(gray_image, wSize2) == 0) {
			int idx = tracker->_clm.GetViewIdx();
			Util::Draw(frameBGR, tracker->_shape, tracker->_clm._visi[idx]);
			face = tracker->_rect;
			// cv::rectangle(frameBGR, face, COLOR_BLUE);
		} else {
			int x_bottom_left = 40;
			int y_bottom_left = rng.uniform(frameBGR.rows/4, frameBGR.rows/3);
			putText(frameBGR, "NO FACE!", cv::Point(x_bottom_left, y_bottom_left), cv::FONT_HERSHEY_TRIPLEX, 3, COLOR_RED, 3);
			tracker->FrameReset();
		}

		return face;
	}

	// retorna pontos de interesse encontrados pelo FaceTracker
	vector<cv::Point> findLandmarks(vector<int> landmarkIndices) {
		vector<cv::Point> landmarks;

		int idx = tracker->_clm.GetViewIdx();
		cv::Mat visi = tracker->_clm._visi[idx];
		cv::Mat shape = tracker->_shape;
		cv::Rect face = tracker->_rect;

		int n = shape.rows/2;
		for (int i = 0; i < landmarkIndices.size(); ++i)
		{
			int index = landmarkIndices[i];

			if (visi.at<int>(index,0) == 0) continue;
			// cout << shape.at<double>(index,0) << " // " << shape.at<double>(index+n,0)
			int x = shape.at<double>(index,0)/* - face.x*/;
			int y = shape.at<double>(index+n,0)/* - face.y*/;

			landmarks.push_back(cv::Point(x,y));
		}

		return landmarks;
	}

	// retorna apenas pontos pertencentes à face
	vector<cv::Point> getFacePoints(cv::Mat &frameBGR, vector<int> landmarkIndices) {
		vector<cv::Point> facePoints;

		/* encontra pontos de contorno (com indices do facetracker) */
		vector<cv::Point> facePolygonPoints = findLandmarks(landmarkIndices);

		if (facePolygonPoints.size() != landmarkIndices.size())
			return facePoints;

		int menorX = 9999, menorY = 9999;
		int maiorX = 0, maiorY = 0;
		for (int i = 0; i < landmarkIndices.size(); ++i)
		{
			int index = landmarkIndices[i];
			if (index == 26 or index == 25 or index == 24 or index == 19 or index == 18 or index == 17)
			{
				facePolygonPoints[i].y -= 15;
			}

			// para otimizar as verificações do poligono
			if (facePolygonPoints[i].x < menorX) menorX = facePolygonPoints[i].x;
			if (facePolygonPoints[i].y < menorY) menorY = facePolygonPoints[i].y;

			if (facePolygonPoints[i].x > maiorX) maiorX = facePolygonPoints[i].x;
			if (facePolygonPoints[i].y > maiorY) maiorY = facePolygonPoints[i].y;
		}

		/* cria poligono com estes pontos */
		const cv::Point *pts = (const cv::Point*) cv::Mat(facePolygonPoints).data;
		int npts = cv::Mat(facePolygonPoints).rows;
		cv::polylines(frameBGR, &pts, &npts, 1,
		    		true, 			// draw closed contour (i.e. joint end to start)
		            cv::Scalar(0,255,0)// colour RGB ordering (here = green)
		            );

		/* verifica quais pontos da imagem estão dentro deste poligono */
		for (int x = menorX; x < maiorX+1; ++x)
		{
			for (int y = menorY; y < maiorY+1; ++y)
			{
				cv::Point2f test_pt(x, y);
				if (cv::pointPolygonTest(cv::Mat(facePolygonPoints), test_pt, true) > 0){
					// cv::rectangle(frameBGR, test_pt, test_pt, cv::Scalar(0, 0, 255), 3, 8, 0); // RED point
					facePoints.push_back(test_pt);
				}
			}
		}

		return facePoints;
	}

	void reset() {
		tracker = new FACETRACKER::Tracker("../src/MyFaceTracker/model/face2.tracker");
	}
};

#endif