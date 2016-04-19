#ifndef UTIL_H
#define UTIL_H

#include "Scan3D.h"

class Util
{
public:

	/* string operations */

	static vector<string> split(string str, string sep) {
		// char* cstr = const_cast<char*>(str.c_str());
		char cstr[str.size()];
		str.copy(cstr, str.size());
		char* current;
		vector<string> arr;
		current = strtok(cstr, sep.c_str());

		while(current != NULL)
		{
			arr.push_back(current);
			current = strtok(NULL, sep.c_str());
		}

		return arr;
	}

	static string join(vector<string> parts, string glue = "") {
		string frankenstein;
		for (int i = 0; i < parts.size()-1; ++i)
		{
			frankenstein += parts[i] + glue;
		}
		return frankenstein + parts.back();
	}

	static vector<string> split(string str, char delimiter) {
		vector<string> internal;
		stringstream ss(str);
		string tok;

		while(getline(ss, tok, delimiter)) {
			internal.push_back(tok);
		}

		return internal;
	}

	static string getFileExtension(string fileName) {
		return split(fileName, '.').back();
	}

	static string removeFileExtension(string fileName) {
		// vector<string> temp = split(fileName, ".");
		Tokenizer tok(fileName, '.');
		vector<string> temp = tok.remaining();

		string nameWithoutExtension;
		for (int i = 0; i < temp.size()-1; ++i)
		{
			nameWithoutExtension = nameWithoutExtension + temp[i];
		}

		return nameWithoutExtension;
	}

	static string addSuffix(string str, string suffix) {
		return str + suffix;
	}

	static string changeSuffix(string filePath, string suffix) {
		// vector<string> path = Util::split(filePath, "/");
		Tokenizer tok(filePath, '/');
		vector<string> path = tok.remaining();

		string fileNameRaw = Util::removeFileExtension(path.back());
		string newFileName = Util::addSuffix(fileNameRaw, suffix);

		if (path.size() > 1)
		{
			vector<string> previousPath(path.begin(), path.end() - 1);
			return join(previousPath, "/") + "/" + newFileName;
		}

		return newFileName;
	}

	/* other */

	// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
	static string currentDateTime() {
		time_t     now = time(0);
		struct tm  tstruct;
		char       buf[80];
		tstruct = *localtime(&now);

		// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
		// for more information about date/time format
		strftime(buf, sizeof(buf), "%Y-%m-%d_%T", &tstruct);

		return buf;
	}

	// desenha pontos de facetracker
	static void Draw(cv::Mat &image, cv::Mat &shape, cv::Mat &visi) {
		int n = shape.rows/2;
		cv::Point p1;
		cv::Scalar c;

  		//draw points
		int j = 0;
		for(int i = 0; i < n; i++) {
			if (visi.at<int>(i,0) == 0) continue;
			p1 = cv::Point(shape.at<double>(i,0), shape.at<double>(i+n,0));
			c = cv::Scalar(0, 255, 0);
			cv::circle(image, p1, 2, c);

			// ostringstream ss;
			// ss << i;
			// putText(image, ss.str(), p1, cv::FONT_HERSHEY_SIMPLEX, .3, cv::Scalar(0, 0, 255));
		}
	}

	// distancia entre dois pontos (de verdade)
	static float euclideanDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
		return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
	}

	// distancia entre dois pontos (apenas para descobrir o ponto mais próximo)
	static float euclideanDistance2(float x1, float y1, float z1, float x2, float y2, float z2) {
		return pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2);
	}

	// cria vector inicializado com valores vindos de um array
	static vector<int> initVector(int array[]) {
		int arrLength = sizeof(array) / sizeof(array[0]);
		vector<int> result(array, array + arrLength);
		return result;
	}

	static cv::Point findMinimumXY(std::vector<cv::Point> points) {
		cv::Point minimumXY(9999, 9999);

		for (int i = 0; i < points.size(); ++i)
		{
			cv::Point cvp = points[i];
			if (cvp.x < minimumXY.x) minimumXY.x = cvp.x;
			if (cvp.y < minimumXY.y) minimumXY.y = cvp.y;
		}

		return minimumXY;
	}

	static void printPoints(std::vector<cv::Point> points) {
		cout << "[ ";

		for (int i = 0; i < points.size(); ++i)
		{
			cv::Point cvp = points[i];
			cout << "(" << cvp.x << "," << cvp.y << ") ";
		}

		cout << "]" << endl;
	}
};

#endif