#ifndef NOSE_LANDMARKS_H
#define NOSE_LANDMARKS_H

class NoseLandmarks
{
public:
	// indice de pontos na nuvem
	int noseTip, nose_left, nose_right, noseBase;
	
	NoseLandmarks() {}

	NoseLandmarks(int noseTip, int nose_left, int nose_right, int noseBase) {
		this->noseTip = noseTip;
		this->nose_left = nose_left;
		this->nose_right = nose_right;
		this->noseBase = noseBase;
	}

	vector<int> allIndices() {
		vector<int> indices;

		indices.push_back(noseTip);
		indices.push_back(nose_left);
		indices.push_back(nose_right);
		indices.push_back(noseBase);

		return indices;
	}
};

#endif