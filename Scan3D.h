#ifndef SCAN_3D_H
#define SCAN_3D_H

#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>	// TransformationEstimationSVD
#include <pcl/registration/icp.h>							// IterativeClosestPoint
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>								// removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>					// eifilter
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>

#include "Landmarks.h"
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
	pcl::TextureMesh texMesh;
	// EyeLandmarks eyeLandmarks;
	// NoseLandmarks noseLandmarks;
	Landmarks landmarks;

	/*======================= Constructors =======================*/

	Scan3D() {
		this->cloud = CloudType::Ptr(new CloudType);
	}

	Scan3D(CloudType::Ptr cloud) {
		this->cloud = cloud;
	}

	Scan3D(CloudType::Ptr cloud, Landmarks landmarks) {
		this->cloud = cloud;
		// this->eyeLandmarks = eyeLandmarks;
		// this->noseLandmarks = noseLandmarks;
		this->landmarks = landmarks;
	}

	Scan3D(string cloudFileName, string landmarksFileName = "") {
		this->cloud = CloudType::Ptr(new CloudType);

		if (landmarksFileName == "")
		{
			landmarksFileName = Util::changeSuffix(cloudFileName, "_landmarks.txt");
		}

		load(cloudFileName, landmarksFileName);
	}

	/*=================== Cloud main operations ===================*/

	float zDistanceEyeToNose() {
		// PointType eyePoint = cloud->at(eyeLandmarks.leftEye_right);
		// PointType nosePoint = cloud->at(noseLandmarks.noseTip);
		PointType eyePoint = cloud->at(landmarks.leftEye_right);
		PointType nosePoint = cloud->at(landmarks.noseTip);
		return fabs(eyePoint.z - nosePoint.z);
	}

	void removeDistantPoints() {
		// calcula diferença, em Z, dos olhos até o nariz
		float distEyeToNose = zDistanceEyeToNose();

		// seleciona todos os pontos que estejam mto longe
		vector<int> facePointIndices;
		// PointType eyePoint = cloud->at(eyeLandmarks.leftEye_left);
		PointType eyePoint = cloud->at(landmarks.leftEye_left);
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
		if (not cvFacePoints.size()) {
			this->cloud->clear();
			return;
		}

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

	/*
	void findLandmarks(vector<cv::Rect> eyes) {
		if (eyes.size() != 2)
			return;

		float x, y;
		cv::Rect leftEye = eyes[0];
		cv::Rect rightEye = eyes[1];

		Util::leftCenterPoint(leftEye, x, y);
		eyeLandmarks.leftEye_left = findPoint(x, y);

		Util::rightCenterPoint(leftEye, x, y);
		eyeLandmarks.leftEye_right = findPoint(x, y);

		Util::leftCenterPoint(rightEye, x, y);
		eyeLandmarks.rightEye_left = findPoint(x, y);

		Util::rightCenterPoint(rightEye, x, y);
		eyeLandmarks.rightEye_right = findPoint(x, y);
	}

	void findLandmarks(cv::Rect nose) {
		// find noseTip 
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

		// find noseBase 
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

		// find nose_left 
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

		// find nose_right 
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
	*/

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
		// vector<int> eyeIndices = this->eyeLandmarks.allIndices();
		// vector<int> noseIndices = this->noseLandmarks.allIndices();
		// vector<int> indices;

		// indices.insert(indices.begin(), eyeIndices.begin(), eyeIndices.end());
		// indices.insert(indices.begin() + eyeIndices.size(), noseIndices.begin(), noseIndices.end());

		// return indices;

		return this->landmarks.allIndices();
	}

	void updateLandmarks(vector<int> landmarkIndices) {
		// if (landmarkIndices.size() == 8)
		if (landmarkIndices.size() == 13)
		{
			// eyeLandmarks = EyeLandmarks(landmarkIndices[0], landmarkIndices[1], landmarkIndices[2], landmarkIndices[3]);
			// noseLandmarks = NoseLandmarks(landmarkIndices[4], landmarkIndices[5], landmarkIndices[6], landmarkIndices[7]);
			this->landmarks = Landmarks(landmarkIndices[0], landmarkIndices[1], landmarkIndices[2], landmarkIndices[3],
				landmarkIndices[4], landmarkIndices[5], landmarkIndices[6], landmarkIndices[7],
				landmarkIndices[8], landmarkIndices[9], landmarkIndices[10], landmarkIndices[11],
				landmarkIndices[12]);
		}
	}

	void updateLandmarks(vector<cv::Point> landmarks, cv::Rect face) {
		// if (landmarks.size() != 8) return;
		if (landmarks.size() != 13) return;

		// for (int i = 0; i < landmarks.size(); ++i)
		// {
		// 	cv::Point& cvp = landmarks[i];
		// 	cvp.x -= face.x;
		// 	cvp.y -= face.y;
		// 	paintPoint(this->cloud->at(cvp.x, cvp.y), "green");
		// }

		cv::Point p;
		std::vector<int> indices;
		// p = landmarks[0]; indices.push_back(findPoint(p.x, p.y)); //eyeLandmarks.leftEye_left		= findPoint(p.x, p.y);
		// p = landmarks[1]; indices.push_back(findPoint(p.x, p.y)); //eyeLandmarks.leftEye_right	= findPoint(p.x, p.y);
		// p = landmarks[2]; indices.push_back(findPoint(p.x, p.y)); //eyeLandmarks.rightEye_left	= findPoint(p.x, p.y);
		// p = landmarks[3]; indices.push_back(findPoint(p.x, p.y)); //eyeLandmarks.rightEye_right	= findPoint(p.x, p.y);

		// p = landmarks[4]; indices.push_back(findPoint(p.x, p.y)); //noseLandmarks.noseTip		= findPoint(p.x, p.y);
		// p = landmarks[5]; indices.push_back(findPoint(p.x, p.y)); //noseLandmarks.nose_left	= findPoint(p.x, p.y);
		// p = landmarks[6]; indices.push_back(findPoint(p.x, p.y)); //noseLandmarks.noseBase	= findPoint(p.x, p.y);
		// p = landmarks[7]; indices.push_back(findPoint(p.x, p.y)); //noseLandmarks.nose_right	= findPoint(p.x, p.y);

		// p = landmarks[8]; indices.push_back(findPoint(p.x, p.y));
		// p = landmarks[9]; indices.push_back(findPoint(p.x, p.y));

		for (int i = 0; i < landmarks.size(); ++i)
		{
			p = landmarks[i];
			indices.push_back(findPoint(p.x, p.y));
		}

		updateLandmarks(indices);
	}

	/*============================ IO ============================*/

	void load(string cloudFileName, string landmarksFileName) {
		loadScanFromFile(cloudFileName);
		loadLandmarksFromTXT(landmarksFileName);
	}

	void save(string cloudFileName = "point_cloud.off", string landmarksFileName = "") {
		if (landmarksFileName == "")
			landmarksFileName = Util::changeSuffix(cloudFileName, "_landmarks.txt");

		string fileExtension = Util::getFileExtension(cloudFileName);

		if (fileExtension == "off")
		{
			CloudIO::saveCloudToOFF(this->cloud, this->faces, cloudFileName);
			saveLandmarksToTXT(landmarksFileName);
		}
		else if (fileExtension == "obj")
		{
			CloudIO::saveCloudToOBJ(this->texMesh, cloudFileName);
			CloudIO::fixFaces(cloudFileName);
		}
	}

	void saveLandmarksToTXT(string fileName = "point_cloud_landmarks.txt") {
		ofstream file(fileName.c_str());

		// vector<int> landmarkIndices = this->eyeLandmarks.allIndices();
		// vector<int> noseIndices = this->noseLandmarks.allIndices();
		// landmarkIndices.insert(landmarkIndices.end(), noseIndices.begin(), noseIndices.end());
		vector<int> landmarkIndices = this->landmarks.allIndices();

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
		vector<PointType> points(13);
		vector<int> landmarkIndices;

		for (int i = 0; i < 13; ++i)
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
			// this->eyeLandmarks = EyeLandmarks(landmarkIndices[0], landmarkIndices[1], landmarkIndices[2], landmarkIndices[3]);
			// this->noseLandmarks = NoseLandmarks(landmarkIndices[4], landmarkIndices[5], landmarkIndices[6], landmarkIndices[7]);
			this->landmarks = Landmarks(landmarkIndices[0], landmarkIndices[1], landmarkIndices[2], landmarkIndices[3],
				landmarkIndices[4], landmarkIndices[5], landmarkIndices[6], landmarkIndices[7],
				landmarkIndices[8], landmarkIndices[9], landmarkIndices[10], landmarkIndices[11],
				landmarkIndices[12]);
		} else {
			cerr << "--(!) ERROR: Landmarks read from file don't match points in the cloud!" << endl;
		}
	}

	void loadScanFromFile(string cloudFileName) {
		string fileExtension = Util::getFileExtension(cloudFileName);

		if (fileExtension == "pcd")
		{
			this->cloud = CloudIO::loadCloudFromPCD(cloudFileName);
		}
		else if (fileExtension == "off")
		{
			this->cloud = CloudIO::loadCloudFromOFF(cloudFileName);
			this->faces = CloudIO::loadFacesFromOFF(cloudFileName);
			// pcl::toPCLPointCloud2(*(this->cloud), this->texMesh.cloud);
			// this->texMesh.tex_polygons.push_back(this->faces);
		}
		else if (fileExtension == "obj")
		{
			this->texMesh = CloudIO::loadCloudFromOBJ(cloudFileName);
			pcl::fromPCLPointCloud2(this->texMesh.cloud, *(this->cloud));
			this->faces = this->texMesh.tex_polygons[0];
		}
		else {
			cerr << "--(!) ERROR: File extension must be \".pcd\", \".off\" or \".obj\"!" << endl;
			return;
		}
	}

	static void massSaving(vector<Scan3D> scans, string fileName = "scan") {
		for (int i = 0; i < scans.size(); ++i)
		{
			std::ostringstream index;
			index << i+1;
			scans[i].save(fileName+index.str()+".off", fileName+index.str()+"_land.txt");
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

	// marca o ponto de interesse da boca de amarelo
	void markMouthPoint(PointType &mouthPoint) {
		paintPoint(mouthPoint, "yellow");
	}

	// marca o ponto de interesse da boca de ciano
	void markChinPoint(PointType &chinPoint) {
		paintPoint(chinPoint, "cyan");
	}

	void markEyePoints() {
		markEyePoint(cloud->at(landmarks.leftEye_left));
		markEyePoint(cloud->at(landmarks.leftEye_right));
		markEyePoint(cloud->at(landmarks.rightEye_left));
		markEyePoint(cloud->at(landmarks.rightEye_right));
	}

	void markNosePoints() {
		markNosePoint(cloud->at(landmarks.noseTip));
		markNosePoint(cloud->at(landmarks.nose_left));
		markNosePoint(cloud->at(landmarks.noseBase));
		markNosePoint(cloud->at(landmarks.nose_right));
	}

	void markMouthPoints() {
		markMouthPoint(cloud->at(landmarks.mouth_up));
		markMouthPoint(cloud->at(landmarks.mouth_left));
		markMouthPoint(cloud->at(landmarks.mouth_right));
		markMouthPoint(cloud->at(landmarks.mouth_down));
	}

	void markChinPoints() {
		markChinPoint(cloud->at(landmarks.chin));
	}

	// marca pontos de interesse na nuvem (olhos e nariz)
	void markFaceLandmarks() {
		markEyePoints();
		markNosePoints();
		markMouthPoints();
		markChinPoints();
	}

	// "aplica textura" copiando as cores dos pontos de uma nuvem próxima
	void applyTextureFromCloud(Scan3D cloud, int k = 1) {
		pcl::KdTree<PointType>::Ptr tree (new pcl::KdTreeFLANN<PointType>);
		tree->setInputCloud(cloud.cloud);

		for (int i = 0; i < this->cloud->size(); ++i)
		{
			PointType& p = this->cloud->at(i);

			std::vector<int> nn_indices (k);
			std::vector<float> nn_dists (k);
			tree->nearestKSearch(p, k, nn_indices, nn_dists);

			// int closestPtIndex = cloud.findClosestPoint(p.x, p.y, p.z);
			PointType closestPt = cloud.cloud->at(nn_indices[0]);

			// copia RGB do ponto
			p.r = closestPt.r;
			p.g = closestPt.g;
			p.b = closestPt.b;
		}
	}

	void applyTextureFromImage(Scan3D fullCloud, string blankImgFileName, string pathToSave) {
		CloudOp::fillBlankImage(this->cloud, fullCloud.cloud, blankImgFileName, pathToSave);
	}

	// PointType getLeftEyeLeftPoint() {
	// 	return this->cloud->at(this->eyeLandmarks.leftEye_left);
	// }

	// PointType getLeftEyeRightPoint() {
	// 	return this->cloud->at(this->eyeLandmarks.leftEye_right);
	// }

	// PointType getRightEyeLeftPoint() {
	// 	return this->cloud->at(this->eyeLandmarks.rightEye_left);
	// }

	// PointType getRightEyeRightPoint() {
	// 	return this->cloud->at(this->eyeLandmarks.rightEye_right);
	// }

	// PointType getNoseLeftPoint() {
	// 	return this->cloud->at(this->noseLandmarks.nose_left);
	// }

	// PointType getNoseTipPoint() {
	// 	return this->cloud->at(this->noseLandmarks.noseTip);
	// }

	// PointType getNoseRightPoint() {
	// 	return this->cloud->at(this->noseLandmarks.nose_right);
	// }

	// PointType getNoseBasePoint() {
	// 	return this->cloud->at(this->noseLandmarks.noseBase);
	// }

	std::vector<PointType> landmarkPoints() {
		std::vector<PointType> landmarkPoints;
		PointType p;

		// p = this->cloud->at(this->landmarks.leftEye_left);		landmarkPoints.push_back(p);
		// p = this->cloud->at(this->landmarks.leftEye_right);		landmarkPoints.push_back(p);
		// p = this->cloud->at(this->landmarks.rightEye_left);		landmarkPoints.push_back(p);
		// p = this->cloud->at(this->landmarks.rightEye_right);	landmarkPoints.push_back(p);

		// p = this->cloud->at(this->landmarks.nose_left);			landmarkPoints.push_back(p);
		// p = this->cloud->at(this->landmarks.noseTip);			landmarkPoints.push_back(p);
		// p = this->cloud->at(this->landmarks.nose_right);		landmarkPoints.push_back(p);
		// p = this->cloud->at(this->landmarks.noseBase);			landmarkPoints.push_back(p);

		// p = this->cloud->at(this->landmarks.mouth_left);		landmarkPoints.push_back(p);
		// p = this->cloud->at(this->landmarks.mouth_right);		landmarkPoints.push_back(p);

		// p = this->cloud->at(this->landmarks.chin);				landmarkPoints.push_back(p);

		std::vector<int> allIndices = this->landmarks.allIndices();
		for (int i = 0; i < allIndices.size(); ++i)
		{
			p = this->cloud->at(allIndices[i]);
			landmarkPoints.push_back(p);
		}

		return landmarkPoints;
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


		// boa calibração de parâmetros by Thiago Oliveira dos Santos
		icp.setMaximumIterations(5000);
		icp.setTransformationEpsilon(0.00000001);
		icp.setEuclideanFitnessEpsilon(0.00000001);
		icp.setRANSACIterations(5000);
		//icp.setRANSACOutlierRejectionThreshold(1);


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
		CloudType::Ptr thisCloud(new CloudType);
		CloudType::Ptr targetCloud(new CloudType);
		pcl::removeNaNFromPointCloud(*(this->cloud),	*(thisCloud),		v);
		pcl::removeNaNFromPointCloud(*(scan_tgt.cloud),	*(targetCloud),	v);

		pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
		gicp.setInputSource(thisCloud);
		gicp.setInputTarget(targetCloud);

		// boa calibração de parâmetros by Thiago Oliveira dos Santos
		// gicp.setMaximumIterations(5000);
		// gicp.setTransformationEpsilon(0.00000001);
		// gicp.setEuclideanFitnessEpsilon(0.00000001);
		// gicp.setRANSACIterations(5000);
		//gicp.setRANSACOutlierRejectionThreshold(1);

		gicp.align(*(thisCloud));

		// modifica a nuvem
		Eigen::Matrix4f transformationMatrix = gicp.getFinalTransformation(); 
		pcl::transformPointCloud(*(this->cloud), *(this->cloud), transformationMatrix);
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
		// return Scan3D(CloudOp::copyCloud(this->cloud), this->eyeLandmarks, this->noseLandmarks);
		return Scan3D(CloudOp::copyCloud(this->cloud), this->landmarks);
	}

	void print() {
		cout << "Scan3D:" << endl;
		cout << "\tcloud size: " << cloud->size() << endl;

		cout << "\teyeLandmarks: " << endl;
		cout << "\t\tleftEye_left: " << landmarks.leftEye_left << endl;
		cout << "\t\tleftEye_right: " << landmarks.leftEye_right << endl;
		cout << "\t\trightEye_left: " << landmarks.rightEye_left << endl;
		cout << "\t\trightEye_right: " << landmarks.rightEye_right << endl;

		cout << "\tnoseLandmarks: " << endl;
		cout << "\t\tnoseTip: " << landmarks.noseTip << endl;
		cout << "\t\tnose_left: " << landmarks.nose_left << endl;
		cout << "\t\tnose_right: " << landmarks.nose_right << endl;
		cout << "\t\tnoseBase: " << landmarks.noseBase << endl;

		cout << "\tmouthLandmarks: " << endl;
		cout << "\t\tmouth_up: " << landmarks.mouth_up << endl;
		cout << "\t\tmouth_left: " << landmarks.mouth_left << endl;
		cout << "\t\tmouth_right: " << landmarks.mouth_right << endl;
		cout << "\t\tmouth_down: " << landmarks.mouth_down << endl;

		cout << "\tchinLandmarks: " << endl;
		cout << "\t\tchin: " << landmarks.chin << endl;
	}

	void printTexMesh() {
		// cout << this->texMesh.cloud << endl;
		// cout << "Header: " << this->texMesh.header << endl;

		cout << "Tex Polygons (Fs): ";
		if (this->texMesh.tex_polygons.size())
		{
			cout << this->texMesh.tex_polygons[0].size() << endl;
			cout << this->texMesh.tex_polygons[0][0] << "..." << endl;
		} else {
			cout << this->texMesh.tex_polygons.size() << endl;
		}

		cout << endl;

		cout << "Tex Coordinates (VTs): ";
		if (this->texMesh.tex_coordinates.size())
		{
			cout << this->texMesh.tex_coordinates[0].size() << endl;
			cout << this->texMesh.tex_coordinates[0][0] << endl;
			cout << "..." << endl;
		} else {
			cout << this->texMesh.tex_coordinates.size() << endl;
		}

		cout << endl;

		cout << "Tex Materials (dados do arquivo MTL): " << this->texMesh.tex_materials.size() << endl;
		if (this->texMesh.tex_materials.size())
		{
			pcl::TexMaterial tm = this->texMesh.tex_materials[0];
			cout << "\tTex Name: " << tm.tex_name << endl;
			cout << "\tTex File: "  << tm.tex_file << endl;
			cout << "\tAmbient Color: "  << tm.tex_Ka.r << " " << tm.tex_Ka.g << " " << tm.tex_Ka.b << endl;
			cout << "\tDiffuse Color: "  << tm.tex_Kd.r << " " << tm.tex_Kd.g << " " << tm.tex_Kd.b << endl;
			cout << "\tSpecular Color: "  << tm.tex_Ks.r << " " << tm.tex_Ks.g << " " << tm.tex_Ks.b << endl;
			cout << "\tTransparency: " << tm.tex_d << endl;
			cout << "\tShininess: " << tm.tex_Ns << endl;
			cout << "\tIllumination Model: " << tm.tex_illum << endl;
			cout << "\t..." << endl;
		}
	}

	void removeOutliers() {
		/* salva landmarks */
		std::vector<PointType> landmarkPts;
		std::vector<int> landmarkIndices = getLandmarkIndices();
		for (int i = 0; i < landmarkIndices.size(); ++i)
		{
			PointType p = this->cloud->at(landmarkIndices[i]);
			landmarkPts.push_back(p);
		}

		/* retira outliers */
		pcl::StatisticalOutlierRemoval<PointType> sor;
		sor.setInputCloud(this->cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*(this->cloud));

		/* reinsere landmarks */
		for (int i = 0; i < landmarkPts.size(); ++i)
		{
			this->cloud->push_back(landmarkPts[i]);
		}

		// this->eyeLandmarks.leftEye_left		= this->cloud->size() - 8;
		// this->eyeLandmarks.leftEye_right	= this->cloud->size() - 7;
		// this->eyeLandmarks.rightEye_left	= this->cloud->size() - 6;
		// this->eyeLandmarks.rightEye_right	= this->cloud->size() - 5;
		// this->noseLandmarks.noseTip			= this->cloud->size() - 4;
		// this->noseLandmarks.nose_left		= this->cloud->size() - 3;
		// this->noseLandmarks.nose_right		= this->cloud->size() - 2;
		// this->noseLandmarks.noseBase		= this->cloud->size() - 1;
		this->landmarks.leftEye_left	= this->cloud->size() - 13;
		this->landmarks.leftEye_right	= this->cloud->size() - 12;
		this->landmarks.rightEye_left	= this->cloud->size() - 11;
		this->landmarks.rightEye_right	= this->cloud->size() - 10;

		this->landmarks.noseTip			= this->cloud->size() - 9;
		this->landmarks.nose_left		= this->cloud->size() - 8;
		this->landmarks.nose_right		= this->cloud->size() - 7;
		this->landmarks.noseBase		= this->cloud->size() - 6;

		this->landmarks.mouth_up		= this->cloud->size() - 5;
		this->landmarks.mouth_left		= this->cloud->size() - 4;
		this->landmarks.mouth_right		= this->cloud->size() - 3;
		this->landmarks.mouth_down		= this->cloud->size() - 2;

		this->landmarks.chin			= this->cloud->size() - 1;
	}

	void updateTexData(string meanFaceFileName, string texFile) {
		// copia todas as informações da face média
		Scan3D meanFace(meanFaceFileName);
		this->texMesh = meanFace.texMesh;

		/* FAZENDO CALCULOS DE NORMAIS DE TODOS OS PONTOS DA NUVEM */

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud (*(this->cloud), *cloud_xyz);
		ne.setInputCloud(cloud_xyz);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		ne.setSearchMethod (tree);

		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

		ne.setRadiusSearch (0.03);

		ne.compute (*cloud_normals);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud_xyz, *cloud_normals, *cloud_with_normals);

		/* FIM DOS CALCULOS */

		// nuvem de pontos não pode ser a mesma da face média
		pcl::toPCLPointCloud2(*cloud_with_normals, this->texMesh.cloud);

		// alterar arquivo de textura utilizado (também não será o mesmo da face média)
		this->texMesh.tex_materials[0].tex_file = texFile;
	}
};

#endif