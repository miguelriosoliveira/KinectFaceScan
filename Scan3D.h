#ifndef SCAN_3D_H
#define SCAN_3D_H

#include <pcl/registration/transformation_estimation_svd.h>	// TransformationEstimationSVD
#include <pcl/registration/icp.h>							// IterativeClosestPoint
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>								// removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>					// eifilter
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include "EyeLandmarks.h"
#include "NoseLandmarks.h"
#include "CloudOp.h"
#include "CloudIO.h"
#include "Util.h"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> CloudType;

class Scan3D
{
public:
	
	CloudType::Ptr cloud;
	vector<pcl::Vertices> faces;
	EyeLandmarks eyeLandmarks;
	NoseLandmarks noseLandmarks;

	/*======================= Constructors =======================*/

	Scan3D() {
		this->cloud = CloudType::Ptr(new CloudType);
	}

	Scan3D(CloudType::Ptr cloud) {
		this->cloud = cloud;
	}

	Scan3D(CloudType::Ptr cloud, EyeLandmarks eyeLandmarks, NoseLandmarks noseLandmarks) {
		this->cloud = cloud;
		this->eyeLandmarks = eyeLandmarks;
		this->noseLandmarks = noseLandmarks;
	}

	Scan3D(string cloudFileName, string landmarksFileName = "") {
		this->cloud = CloudType::Ptr(new CloudType);
		if (landmarksFileName != "")
			load(cloudFileName, landmarksFileName);
	}

	/*=================== Cloud main operations ===================*/

	float zDistanceEyeToNose() {
		PointType eyePoint = cloud->at(eyeLandmarks.leftEye_right);
		PointType nosePoint = cloud->at(noseLandmarks.noseTip);
		return fabs(eyePoint.z - nosePoint.z);
	}

	void removeDistantPoints() {
		// calcula diferença, em Z, dos olhos até o nariz
		float distEyeToNose = zDistanceEyeToNose();

		// seleciona todos os pontos que estejam mto longe
		vector<int> facePointIndices;
		PointType eyePoint = cloud->at(eyeLandmarks.leftEye_left);
		for (int i = 0; i < cloud->size(); ++i)
		{
			// critério para saber se ponto está longe:
			// ter Z menor que [Z do olho + (diferença dos Z do olho e do nariz)*2]
			if (cloud->at(i).z < eyePoint.z + distEyeToNose) {
				facePointIndices.push_back(i);
			}
		}

		// filtra apenas pontos do rosto de acordo com o critério estipulado antes
		CloudOp::filterPointsFromPointCloud(cloud, facePointIndices);
	}

	void removePointsOutsidePolygon(vector<cv::Point> polygonPoints) {
		/* criar poligono e adicionar pontos a ele */
		
		CloudType polygon;
		for (int i = 0; i < polygonPoints.size(); ++i)
		{
			cv::Point cvp = polygonPoints[i];			
			PointType p = this->cloud->at(cvp.x, cvp.y);
			polygon.push_back(p);
		}
		
		/* guardar indices dos pontos que estejam dentro do poligono */

		vector<int> faceIndices;
		for (int i = 0; i < this->cloud->size(); ++i)
		{
			PointType p = this->cloud->at(i);
			if (pcl::isXYPointIn2DXYPolygon(p, polygon))
			{
				faceIndices.push_back(i);
			}
		}

		// filtra apenas pontos do rosto de acordo com o critério estipulado antes
		CloudOp::filterPointsFromPointCloud(this->cloud, faceIndices);
	}

	void getPointsInsidePolygon(vector<cv::Point> cvFacePoints) {
		int menorX = 9999, menorY = 9999;
		int maiorX = -9999, maiorY = -9999;
		for (int i = 0; i < cvFacePoints.size(); ++i)
		{
			cv::Point cvp = cvFacePoints[i];

			if (cvp.x < menorX) menorX = cvp.x;
			if (cvp.y < menorY) menorY = cvp.y;

			if (cvp.x > maiorX) maiorX = cvp.x;
			if (cvp.y > maiorY) maiorY = cvp.y;
		}

		CloudType::Ptr faceCloud(new CloudType);
		faceCloud->height = abs(maiorX - menorX);
		faceCloud->width = abs(maiorY - menorY);
		faceCloud->is_dense = false;
		faceCloud->points.resize(faceCloud->height * faceCloud->width);
		
		std::vector<int> facePointIndices;
		for (int i = 0; i < cvFacePoints.size(); ++i)
		{
			cv::Point cvp = cvFacePoints[i];
			faceCloud->at(i) = this->cloud->at(cvp.x, cvp.y);
			facePointIndices.push_back(findPoint(cvp.x, cvp.y));
		}

		// this->cloud = faceCloud;
		CloudOp::filterPointsFromPointCloud(this->cloud, facePointIndices);
	}

	void cropFaceFromAmbient(const CloudType::ConstPtr fullCloud, int faceX, int faceY, int faceHeight, int faceWidth) {
		cloud.reset(new CloudType);
		cloud->height = faceHeight;
		cloud->width = faceWidth;
		cloud->is_dense = false;
		cloud->points.resize(cloud->height * cloud->width);

		for (int y = faceY, yf = 0; y < faceY+faceHeight; y++, yf++)
		{
			for (int x = faceX, xf = 0; x < faceX+faceWidth; x++, xf++)
			{
				cloud->at(xf, yf) = fullCloud->at(x, y);
			}
		}
	}

	/*
	int findClosestPoint(float x, float y, float z) {
		float difX, difY, difZ, minDifX, minDifY, minDifZ;
		minDifX = minDifY = minDifZ = 9999;
		int pos;

		for (int i = 0; i < cloud->size(); ++i)
		{
			PointType p = cloud->at(i);
			difX = fabs(p.x - x);
			difY = fabs(p.y - y);
			difZ = fabs(p.z - z);

			if (difX < minDifX)
			{
				minDifX = difX;
				if (difY < minDifY)
				{
					minDifY = difY;
					if (difZ < minDifZ)
					{
						minDifZ = difZ;
						pos = i;
					}
				}
			}
		}

		return pos;
	}
	*/

	int findClosestPoint(float x, float y, float z) {
		float dist, minDist = 9999;
		int pos;

		for (int i = 0; i < this->cloud->size(); ++i)
		{
			PointType p = this->cloud->at(i);
			dist = Util::euclideanDistance2(x, y, z, p.x, p.y, p.z);
			if (dist < minDist)
			{
				minDist = dist;
				pos = i;
			}
		}

		return pos;
	}

	// retorna indice do ponto, como em uma matriz de pixels
	int findPoint(int x, int y) {
		return y*cloud->width + x;
	}

	int findPointIndex(int rectX, int rectY, int rectHeight, int rectWidth) {
		int width = 640;

		for (int y = 0;     y < rectY+rectHeight; y++)
		{
			for (int x = 0; x < rectX+rectWidth;  x++)
			{
				int fullCloudPos = y*width + x;

				if (cloud->at(fullCloudPos).x == rectX)
				{
					cloud->at(fullCloudPos).r = 0;
					cloud->at(fullCloudPos).g = 255;
					cloud->at(fullCloudPos).b = 0;
				}
			}
		}
	}

	// adiciona ponto na nuvem (CUIDADO: DEIXA NUVEM EM MODO "DESORGANIZADA")
	void addPoint(PointType p) {
		this->cloud->push_back(p);
	}
	
	/*==================== Landmark detection ====================*/

	// retorna indice da ponta do nariz
	int findNoseTip(vector<int> noseLandmarkIndices) {
		int lowerZ = std::numeric_limits<int>::max();
		int noseTip = -1;

		for (int i = 0; i < noseLandmarkIndices.size(); ++i)
		{
			int landmarkIndex = noseLandmarkIndices[i];
			PointType p = cloud->at(landmarkIndex);

			if (p.z < lowerZ)
			{
				lowerZ = p.z;
				noseTip = landmarkIndex;
			}
		}

		return noseTip;
	}

	// verifica se o ponto é um ponto de interesse já marcado (vermelho ou azul)
	bool isLandmark(PointType point) {
		if (CloudOp::isEyePoint(point)) // red
			return true;
		else
			return CloudOp::isNosePoint(point); // blue
	}

	void findLandmarks(vector<cv::Rect> eyes) {
		if (eyes.size() != 2)
			return;

		float x, y;
		cv::Rect leftEye = eyes[0];
		cv::Rect rightEye = eyes[1];

		Util::leftCenterPoint(leftEye, x, y);
		eyeLandmarks.leftEye_left = findPoint(x, y);
		// PointType &leftEye_leftPoint = cloud->at(eyeLandmarks.leftEye_left);
		// paintPoint(leftEye_leftPoint, "red");

		Util::rightCenterPoint(leftEye, x, y);
		eyeLandmarks.leftEye_right = findPoint(x, y);
		// PointType &leftEye_rightPoint = cloud->at(eyeLandmarks.leftEye_right);
		// paintPoint(leftEye_rightPoint, "red");

		Util::leftCenterPoint(rightEye, x, y);
		eyeLandmarks.rightEye_left = findPoint(x, y);
		// PointType &rightEye_leftPoint = cloud->at(eyeLandmarks.rightEye_left);
		// paintPoint(rightEye_leftPoint, "red");

		Util::rightCenterPoint(rightEye, x, y);
		eyeLandmarks.rightEye_right = findPoint(x, y);
		// PointType &rightEye_rightPoint = cloud->at(eyeLandmarks.rightEye_right);
		// paintPoint(rightEye_rightPoint, "red");
	}

	void findLandmarks(cv::Rect nose) {
		/* find noseTip */
		float menorZ = 9999;

		for (int y = nose.y; y < nose.y+nose.height; y++)
		{
			for (int x = nose.x; x < nose.x+nose.width; x++)
			{
				PointType &p = cloud->at(x, y);
				if (p.z < menorZ)
				{
					menorZ = p.z;
					noseLandmarks.noseTip = y*cloud->width + x;
				}
			}
		}

		PointType &nosePoint = cloud->at(noseLandmarks.noseTip);
		// paintPoint(nosePoint, "green");

		/* find noseBase */
		float menorDifX = 9999;
		float e = 0.01;

		for (int y = nose.y+nose.height/2; y < nose.y+nose.height; y++)
		{
			for (int x = nose.x+nose.width/4; x < nose.x+3*nose.width/4; x++)
			{
				PointType &p = cloud->at(x, y);
				if (p.z > nosePoint.z + e)
				{
					float difX = fabs(p.x - nosePoint.x);
					if (difX < menorDifX)
					{
						menorDifX = difX;
						noseLandmarks.noseBase = y*cloud->width + x;
					}
				}
			}
		}

		PointType &noseBasePoint = cloud->at(noseLandmarks.noseBase);
		// paintPoint(noseBasePoint, "green");

		/* find nose_left */
		float menorDifY = 9999;
		for (int y = nose.y+nose.height/4; y < nose.y+3*nose.height/4; y++)
		{
			for (int x = nose.x; x < nose.x+nose.width/2; x++)
			{
				PointType &p = cloud->at(x, y);
				if (nosePoint.x - e > p.x or p.x > nosePoint.x + e)
				{
					if (noseBasePoint.z - e < p.z and p.z < noseBasePoint.z + e)
					{
						float difY = fabs(p.y - nosePoint.y);
						if (difY < menorDifY) {
							menorDifY = difY;
							noseLandmarks.nose_left = y*cloud->width + x;
						}
					}
				}
			}
		}

		// PointType &noseLeftPoint = cloud->at(noseLandmarks.nose_left);
		// paintPoint(noseLeftPoint, "green");

		/* find nose_right */
		menorDifY = 9999;
		for (int y = nose.y+nose.height/4; y < nose.y+3*nose.height/4; y++)
		{
			for (int x = nose.x+nose.width/2; x < nose.x+nose.width; x++)
			{
				PointType &p = cloud->at(x, y);
				if (nosePoint.x - e > p.x or p.x > nosePoint.x + e)
				{
					if (noseBasePoint.z - e < p.z and p.z < noseBasePoint.z + e)
					{
						float difY = fabs(p.y - nosePoint.y);
						if (difY < menorDifY) {
							menorDifY = difY;
							noseLandmarks.nose_right = y*cloud->width + x;
						}
					}
				}
			}
		}

		// PointType &noseRightPoint = cloud->at(noseLandmarks.nose_right);
		// paintPoint(noseRightPoint, "green");
	}

	void findLandmarks(vector<cv::Rect> eyes, cv::Rect nose) {
		findLandmarks(eyes);
		findLandmarks(nose);
	}

	// verifica se os pontos de interesse das duas nuvens batem
	bool gotRightLandmarks(vector<int> indices_src, vector<int> indices_tgt) {
		for (int i = 0; i < indices_src.size(); ++i)
		{
			if (indices_src[i] == -1 and indices_tgt[i] != -1 or indices_src[i] != -1 and indices_tgt[i] == -1)
			{
				return false;
			}
		}
		return true;
	}

	vector<int> getLandmarkIndices() {
		vector<int> eyeIndices = eyeLandmarks.allIndices();
		vector<int> noseIndices = noseLandmarks.allIndices();
		vector<int> indices;

		indices.insert(indices.begin(), eyeIndices.begin(), eyeIndices.end());
		indices.insert(indices.begin() + eyeIndices.size(), noseIndices.begin(), noseIndices.end());

		return indices;
	}

	void updateLandmarks(vector<int> landmarkIndices) {
		if (landmarkIndices.size() == 8)
		{
			eyeLandmarks = EyeLandmarks(landmarkIndices[0], landmarkIndices[1], landmarkIndices[2], landmarkIndices[3]);
			noseLandmarks = NoseLandmarks(landmarkIndices[4], landmarkIndices[5], landmarkIndices[6], landmarkIndices[7]);
		}
	}

	void updateLandmarks(vector<cv::Point> landmarks, cv::Rect face) {
		if (landmarks.size() != 8) return;

		// for (int i = 0; i < landmarks.size(); ++i)
		// {
		// 	cv::Point& cvp = landmarks[i];
		// 	cvp.x -= face.x;
		// 	cvp.y -= face.y;
		// 	paintPoint(this->cloud->at(cvp.x, cvp.y), "green");
		// }

		cv::Point p;
		p = landmarks[0]; eyeLandmarks.leftEye_left		= findPoint(p.x, p.y);
		p = landmarks[1]; eyeLandmarks.leftEye_right	= findPoint(p.x, p.y);
		p = landmarks[2]; eyeLandmarks.rightEye_left	= findPoint(p.x, p.y);
		p = landmarks[3]; eyeLandmarks.rightEye_right	= findPoint(p.x, p.y);

		p = landmarks[4]; noseLandmarks.noseTip		= findPoint(p.x, p.y);
		p = landmarks[5]; noseLandmarks.nose_left	= findPoint(p.x, p.y);
		p = landmarks[6]; noseLandmarks.noseBase	= findPoint(p.x, p.y);
		p = landmarks[7]; noseLandmarks.nose_right	= findPoint(p.x, p.y);
	}

	/*============================ IO ============================*/

	void load(string cloudFileName, string landmarksFileName) {
		loadScanFromFile(cloudFileName);
		loadLandmarksFromTXT(landmarksFileName);
	}

	void save(string cloudFileName = "point_cloud.off", string landmarksFileName = "point_cloud_landmarks.txt") {
		CloudIO::saveCloudToOFF(this->cloud, cloudFileName);
		saveLandmarksToTXT(landmarksFileName);
	}

	void saveLandmarksToTXT(string fileName = "point_cloud_landmarks.txt") {
		ofstream file(fileName.c_str());

		vector<int> landmarkIndices = this->eyeLandmarks.allIndices();
		vector<int> noseIndices = this->noseLandmarks.allIndices();
		landmarkIndices.insert(landmarkIndices.end(), noseIndices.begin(), noseIndices.end());

		for (int i = 0; i < landmarkIndices.size(); ++i)
		{
			PointType p = this->cloud->at(landmarkIndices[i]);
			file << p.x << endl;
			file << p.y << endl;
			file << p.z << endl;
		}

		file.close();
	}

	void loadLandmarksFromTXT(string fileName) {
		ifstream file(fileName.c_str());
		vector<PointType> points(8);
		vector<int> landmarkIndices;

		for (int i = 0; i < 8; ++i)
		{
			file >> points[i].x >> points[i].y >> points[i].z;
		}

		for (int i = 0; i < points.size(); ++i)
		{
			PointType p = points[i];
			landmarkIndices.push_back(findClosestPoint(p.x, p.y, p.z));
		}

		file.close();

		if (landmarkIndices.size() == points.size())
		{
			this->eyeLandmarks = EyeLandmarks(landmarkIndices[0], landmarkIndices[1], landmarkIndices[2], landmarkIndices[3]);
			this->noseLandmarks = NoseLandmarks(landmarkIndices[4], landmarkIndices[5], landmarkIndices[6], landmarkIndices[7]);
		} else {
			cerr << "--(!) ERROR: Landmarks read from file don't match points in the cloud!" << endl;
		}
	}

	void loadScanFromFile(string cloudFileName) {
		string fileExtension = Util::getFileExtension(cloudFileName);

		if (fileExtension == "pcd")
		{
			cloud = CloudIO::loadCloudFromPCD(cloudFileName);
		}
		else if (fileExtension == "off")
		{
			cloud = CloudIO::loadCloudFromOFF(cloudFileName);
			faces = CloudIO::loadFacesFromOFF(cloudFileName);
		}
		else {
			cerr << "--(!) ERROR: File extension must be \".pcd\" or \".off\"!" << endl;
			return;
		}
	}

	/*======================= Point markup =======================*/

	// pinta um ponto com a cor designada
	// red, green, blue podem ser passados como string
	void paintPoint(PointType &point, string color) {
		if (color == "red")
		{
			point.r = 255;
			point.g = 0;
			point.b = 0;
		}
		else if (color == "green")
		{
			point.r = 0;
			point.g = 255;
			point.b = 0;
		}
		else if (color == "blue")
		{
			point.r = 0;
			point.g = 0;
			point.b = 255;
		}
		else {
			cerr << "Color must be \"red\", \"green\" or \"blue\" (case sensitive)!" << endl;
		}
	}

	// pinta um ponto com a cor designada
	void paintPoint(PointType &point, float r, float g, float b) {
		point.r = r;
		point.g = g;
		point.b = b;
	}

	// marca o ponto de interesse do olho de vermelho 
	void markEyePoint(PointType &eyePoint) {
		paintPoint(eyePoint, "red");
	}

	// marca o ponto de interesse do nariz de verde 
	void markNosePoint(PointType &nosePoint) {
		paintPoint(nosePoint, "green");
	}

	void markEyePoints() {
		markEyePoint(cloud->at(eyeLandmarks.leftEye_left));
		markEyePoint(cloud->at(eyeLandmarks.leftEye_right));
		markEyePoint(cloud->at(eyeLandmarks.rightEye_left));
		markEyePoint(cloud->at(eyeLandmarks.rightEye_right));
	}

	void markNosePoints() {
		markNosePoint(cloud->at(noseLandmarks.noseTip));
		markNosePoint(cloud->at(noseLandmarks.nose_left));
		markNosePoint(cloud->at(noseLandmarks.noseBase));
		markNosePoint(cloud->at(noseLandmarks.nose_right));
	}

	// marca pontos de interesse na nuvem (olhos e nariz)
	void markFaceLandmarks() {
		markEyePoints();
		markNosePoints();
	}

	// "aplica textura" copiando as cores dos pontos de uma nuvem próxima
	void applyTextureFromCloud(Scan3D scan) {
		for (int i = 0; i < this->cloud->size(); ++i)
		{
			// encontra ponto mais proximo da nuvem
			PointType& p = this->cloud->at(i);
			int closestPtIndex = scan.findClosestPoint(p.x, p.y, p.z);
			PointType closestPt = scan.cloud->at(closestPtIndex);

			// copia RGB do ponto
			p.r = closestPt.r;
			p.g = closestPt.g;
			p.b = closestPt.b;
		}
	}

	/*==================== Cloud registration ====================*/

	// executa o rigid registration na nuvem THIS para a nuvem TARGET
	void rigidRegistration(Scan3D scan_tgt) {
		// calcula indices dos pontos de interesse (olhos, nariz e queixo)
		vector<int> indices_src = this->getLandmarkIndices();
		vector<int> indices_tgt = scan_tgt.getLandmarkIndices();

		// indices_src contains the index of the matched points in the first cloud
		// indices_tgt contains the index of it's corresponding pair in the second cloud
		if (indices_src.size() != indices_tgt.size()) {
			cerr << "Interest points are different! Make another cloud capture." << endl;
			return;
		}

		// estima a matriz de transformação de cloud_src para cloud_tgt
		Eigen::Matrix4f transformationMatrix;
		pcl::registration::TransformationEstimationSVD <PointType, PointType> te;
		te.estimateRigidTransformation(*(this->cloud), indices_src, *(scan_tgt.cloud), indices_tgt, transformationMatrix);

		// transforma a nuvem
		pcl::transformPointCloud(*(this->cloud), *(this->cloud), transformationMatrix);
	}

	// executa o icp na nuvem THIS para a nuvem TARGET
	void ICPRegistration(Scan3D scan_tgt) {
		//remove NAN points from the cloud (CUIDADO: A NUVEM PODE FICAR DESORGANIZADA!!!)
		std::vector<int> v;
		CloudType::Ptr thisCloud(new CloudType);
		CloudType::Ptr targetCloud(new CloudType);
		pcl::removeNaNFromPointCloud(*(this->cloud),	*(thisCloud),		v);
		pcl::removeNaNFromPointCloud(*(scan_tgt.cloud),	*(targetCloud),	v);

		pcl::IterativeClosestPoint<PointType, PointType> icp;
		icp.setInputSource(thisCloud);
		icp.setInputTarget(targetCloud);

		/*
		// boa calibração de parâmetros by Thiago Oliveira dos Santos
		icp.setMaximumIterations(5000);
		icp.setTransformationEpsilon(0.00000001);
		icp.setEuclideanFitnessEpsilon(0.00000001);
		icp.setRANSACIteration(5000);
		icp.setRANSACOutlierRejectionThreshold(1);
		*/

		// faz o alinhamento
		icp.align(*(thisCloud));

		// modifica a nuvem
		Eigen::Matrix4f transformationMatrix = icp.getFinalTransformation(); 
		pcl::transformPointCloud(*(this->cloud), *(this->cloud), transformationMatrix);
	}

	// executa o gicp na nuvem THIS para a nuvem TARGET
	void GICPRegistration(Scan3D scan_tgt) {
		//remove NAN points from the cloud (CUIDADO: FAZ COM QUE A NUVEM FIQUE DESORGANIZADA!!!)
		std::vector<int> v;
		pcl::removeNaNFromPointCloud(*(this->cloud),	*(this->cloud),		v);
		pcl::removeNaNFromPointCloud(*(scan_tgt.cloud),	*(scan_tgt.cloud),	v);

		pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
		gicp.setInputSource(this->cloud);
		gicp.setInputTarget(scan_tgt.cloud);
		gicp.align(*(this->cloud));
	}

	void rigidICPRegistration(Scan3D scan_tgt) {
		this->rigidRegistration(scan_tgt);
		this->ICPRegistration(scan_tgt);
	}

	void rigidICPRegistration(std::vector<Scan3D> scan_tgts) {
		for (int i = 0; i < scan_tgts.size(); ++i)
		{
			this->rigidICPRegistration(scan_tgts[i]);
		}
	}

	/*=========================== Other ===========================*/

	Scan3D clone() {
		return Scan3D(CloudOp::copyCloud(this->cloud), this->eyeLandmarks, this->noseLandmarks);
	}

	void print() {
		cout << "Scan3D:" << endl;
		cout << "\tcloud size: " << cloud->size() << endl;

		cout << "\teyeLandmarks: " << endl;
		cout << "\t\tleftEye_left: " << eyeLandmarks.leftEye_left << endl;
		cout << "\t\tleftEye_right: " << eyeLandmarks.leftEye_right << endl;
		cout << "\t\trightEye_left: " << eyeLandmarks.rightEye_left << endl;
		cout << "\t\trightEye_right: " << eyeLandmarks.rightEye_right << endl;

		cout << "\tnoseLandmarks: " << endl;
		cout << "\t\tnoseTip: " << noseLandmarks.noseTip << endl;
		cout << "\t\tnoseBase: " << noseLandmarks.noseBase << endl;
		cout << "\t\tnose_left: " << noseLandmarks.nose_left << endl;
		cout << "\t\tnose_right: " << noseLandmarks.nose_right << endl;
	}

	/*
	void bkp_function_nose_markup() {
		float menorZ = -9999;
		int nosetipIndex;
		for (int i = 0; i < faceCloud.cloud->size(); ++i)
		{
			PointType &p = faceCloud.cloud->at(i);
			if (CloudOp::isNosePoint(p)) //azul
			{
				p.r = 0;
				p.g = 0;
				p.b = 0;
			}

			if (p.z > menorZ)
			{
				menorZ = p.z;
				nosetipIndex = i;
			}
		}

		PointType &nosePoint = faceCloud.cloud->at(nosetipIndex);
		nosePoint.r = 255;
		nosePoint.g = 255;
		nosePoint.b = 0;

		float e = 0.001;
		for (int i = nosetipIndex; i < faceCloud.cloud->size(); ++i)
		{
			PointType &p = faceCloud.cloud->at(i);
			if (nosePoint.x - e < p.x and p.x < nosePoint.x + e)
			{
				if (p.z < nosePoint.z - e*10)
				{
					p.r = 255;
					p.g = 255;
					p.b = 0;
					break;
				}
			}
		}

		e = 0.005;
		float menorX = 9999, maiorX = -9999;
		int noseLeftIndex, noseRightIndex;
		for (int i = nosetipIndex - faceCloud.cloud->width; i < faceCloud.cloud->size(); ++i)
		{
			PointType &p = faceCloud.cloud->at(i);
			if (nosePoint.y - e < p.y and p.y < nosePoint.y + e)
			{
				if (nosePoint.x - e*3.5 > p.x)
				{
					if (p.x > maiorX)
					{
						maiorX = p.x;
						noseLeftIndex = i;
					}
				}

				if (p.x > nosePoint.x + e*3.5)
				{
					if (p.x < menorX)
					{
						menorX = p.x;
						noseRightIndex = i;
					}
				}
			}
		}

		PointType &noseLeftPoint = faceCloud.cloud->at(noseLeftIndex);
		noseLeftPoint.r = 255;
		noseLeftPoint.g = 255;
		noseLeftPoint.b = 0;

		PointType &noseRightPoint = faceCloud.cloud->at(noseRightIndex);
		noseRightPoint.r = 255;
		noseRightPoint.g = 255;
		noseRightPoint.b = 0;
	}
	*/
};

#endif