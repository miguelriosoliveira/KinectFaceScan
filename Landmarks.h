#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <vector>
using namespace std;

class Landmarks
{
public:
	// indices de pontos na nuvem
	int leftEye_left, leftEye_right, rightEye_left, rightEye_right;
	int noseTip, nose_left, nose_right, noseBase;
	int mouth_up, mouth_left, mouth_right, mouth_down;
	int chin;

	Landmarks() {}

	Landmarks(int leftEye_left, int leftEye_right, int rightEye_left, int rightEye_right,
		int noseTip, int nose_left, int nose_right, int noseBase,
		int mouth_up, int mouth_left, int mouth_right, int mouth_down,
		int chin)
	{
		this->leftEye_left = leftEye_left;
		this->leftEye_right = leftEye_right;
		this->rightEye_left = rightEye_left;
		this->rightEye_right = rightEye_right;

		this->noseTip = noseTip;
		this->nose_left = nose_left;
		this->nose_right = nose_right;
		this->noseBase = noseBase;

		this->mouth_up = mouth_up;
		this->mouth_left = mouth_left;
		this->mouth_right = mouth_right;
		this->mouth_down = mouth_down;

		this->chin = chin;
	}

	vector<int> eyeIndices() {
		vector<int> eyeIndices;

		if (leftEye_left == 0 or leftEye_right == 0 or rightEye_left == 0 or rightEye_right == 0)
			return eyeIndices;

		eyeIndices.push_back(leftEye_left);
		eyeIndices.push_back(leftEye_right);
		eyeIndices.push_back(rightEye_left);
		eyeIndices.push_back(rightEye_right);

		return eyeIndices;
	}

	vector<int> noseIndices() {
		vector<int> noseIndices;

		if (noseTip == 0 or nose_left == 0 or nose_right == 0 or noseBase == 0)
			return noseIndices;

		noseIndices.push_back(noseTip);
		noseIndices.push_back(nose_left);
		noseIndices.push_back(nose_right);
		noseIndices.push_back(noseBase);

		return noseIndices;
	}

	vector<int> mouthIndices() {
		vector<int> mouthIndices;

		if (mouth_up == 0 or mouth_left == 0 or mouth_right == 0 or mouth_down == 0)
			return mouthIndices;

		mouthIndices.push_back(mouth_up);
		mouthIndices.push_back(mouth_left);
		mouthIndices.push_back(mouth_right);
		mouthIndices.push_back(mouth_down);

		return mouthIndices;
	}

	int chinIndex() {
		return chin;
	}

	vector<int> allIndices() {
		vector<int> allIndices = this->eyeIndices();
		vector<int> noseIndices = this->noseIndices();
		vector<int> mouthIndices = this->mouthIndices();

		allIndices.insert( allIndices.end(), noseIndices.begin(), noseIndices.end() );
		allIndices.insert( allIndices.end(), mouthIndices.begin(), mouthIndices.end() );
		if (chin)
			allIndices.push_back(chin);

		return allIndices;
	}
};

#endif