#ifndef EYE_LANDMARKS_H
#define EYE_LANDMARKS_H

#include <vector>
using namespace std;

class EyeLandmarks
{
public:
	// indices de pontos na nuvem
	int leftEye_left, leftEye_right, rightEye_left, rightEye_right;

	EyeLandmarks() {}
	
	EyeLandmarks(int leftEye_left, int leftEye_right, int rightEye_left, int rightEye_right) {
		this->leftEye_left = leftEye_left;
		this->leftEye_right = leftEye_right;
		this->rightEye_left = rightEye_left;
		this->rightEye_right = rightEye_right;
	}
	
	vector<int> allIndices() {
		vector<int> indices;

		indices.push_back(leftEye_left);
		indices.push_back(leftEye_right);
		indices.push_back(rightEye_left);
		indices.push_back(rightEye_right);

		return indices;
	}
};

#endif