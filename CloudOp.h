
#ifndef CLOUD_OP_H
#define CLOUD_OP_H

#include <algorithm>
#include <ctime>

#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>

#include "tinyxml2.h"
using namespace tinyxml2;

#include "Tokenizer.h"
using namespace br_ufes_inf_nemo_cpp_util;

#include "Util.h"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> CloudType;

class CloudOp
{
	static void resizeCloud(CloudType::Ptr cloudSource, CloudType::Ptr cloudTarget) {
		cloudSource->points.resize(cloudTarget->height * cloudTarget->width);

		for (int i = cloudSource->size(); i < cloudTarget->size(); ++i)
		{
			cloudSource->push_back(PointType());
		}
	}

public:

	static PointType createPoint(float x, float y, float z, int r = 255, int g = 255, int b = 255, int a = 255) {
		PointType p;

		p.x = x; p.y = y; p.z = z;
		p.r = r; p.g = g; p.b = b; p.a = a;

		return p;
	}

	static void addClouds(CloudType::Ptr cloud_tgt, CloudType::Ptr cloud_src) {
		*cloud_tgt += *cloud_src;
	}

	static void matchSizes(CloudType::Ptr cloud1, CloudType::Ptr cloud2) {
		if (cloud1->size() < cloud2->size())
		{
			resizeCloud(cloud1, cloud2);
		} else {
			resizeCloud(cloud2, cloud1);
		}
	}

	static CloudType::Ptr copyCloud(CloudType::Ptr cloud) {
		CloudType::Ptr result(new CloudType);
		*result = *cloud;
		return result;
	}

	static void adjustCloud (CloudType::Ptr cloud) {
		Eigen::Affine3f transformationMatrix = Eigen::Affine3f::Identity();
		transformationMatrix.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
		// transformationMatrix.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));
		transformPointCloud(*cloud, *cloud, transformationMatrix);
	}

	static void filterPointsFromPointCloud (CloudType::Ptr cloud, vector<int> indicesToFilter) {
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		inliers->indices = indicesToFilter;

		pcl::ExtractIndices<PointType> eiFilter(true); // Initializing with true will allow us to extract the removed indices
		eiFilter.setInputCloud(cloud);
		eiFilter.setIndices(inliers);
		eiFilter.setKeepOrganized(true);
		eiFilter.filter(*cloud);
	}

	static vector<int> getIndicesOfPoints (CloudType::Ptr cloud) {
		CloudType::Ptr cloudCopy = copyCloud(cloud);

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		std::vector<int> indices;
		inliers->indices = indices;

		pcl::ExtractIndices<PointType> eiFilter(true); // Initializing with true will allow us to extract the removed indices
		eiFilter.setInputCloud(cloudCopy);
		eiFilter.setIndices(inliers);
		eiFilter.setKeepOrganized(true);
		eiFilter.filter(*cloudCopy);
	}

	static bool isEyePoint(PointType point) {
		return (point.r == 255 and point.g == 0 and point.b == 0); // red
	}

	static bool isNosePoint(PointType point) {
		return (point.r == 0 and point.g == 0 and point.b == 255); // blue
	}

	static vector<int> findLandmarkIndices(CloudType::Ptr cloud) {
		vector<int> eyeIndices, noseIndices;

		for (int i = 0; i < cloud->size(); ++i)
		{
			PointType p = cloud->at(i);

			if (isEyePoint(p)) {
				eyeIndices.push_back(i);
			}
			else if (isNosePoint(p)) {
				noseIndices.push_back(i);
			}
		}

		vector<int> indices(8, -1);
		indices.insert(indices.begin(), eyeIndices.begin(), eyeIndices.end());
		indices.insert(indices.begin() + eyeIndices.size(), noseIndices.begin(), noseIndices.end());

		return indices;
	}

	/*static void cloudsRigidRegistration (CloudType::Ptr cloud_src, CloudType::Ptr cloud_tgt) {
	// // caso as nuvens tenham tamanhos diferentes
	// matchSizes(cloud_src, cloud_tgt);

	// calcula indices dos pontos de interesse (olhos, nariz e queixo)
	vector<int> indices_src = findLandmarkIndices(cloud_src);
	vector<int> indices_tgt = findLandmarkIndices(cloud_tgt);

	// indices_src contains the index of the matched points in the first cloud
	// indices_tgt contains the index of it's corresponding pair in the second cloud
	if (not gotRightLandmarks(indices_src, indices_tgt)) {
		cerr << "Interest points are different! Make another cloud capture." << endl;
		return;
	}

	// estima a matriz de transformação de cloud_src para cloud_tgt
	Eigen::Matrix4f transformationMatrix;
	registration::TransformationEstimationSVD <PointType, PointType> te;
	te.estimateRigidTransformation(*cloud_src, indices_src, *cloud_tgt, indices_tgt, transformationMatrix);

	// transforma a nuvem
	transformPointCloud(*cloud_src, *cloud_src, transformationMatrix);

	// unmarkFaceInterestPoints(output);
	}*/

	/*
	static int findClosestPoint(PointType p, CloudType::Ptr cloudTarget) {
		int closestPointIndex = 0;
		// float minDist = 999999999999;

		PointType p0 = cloudTarget->at(0);
		float minDist = Util::euclideanDistance2(p.x, p.y, p.z, p0.x, p0.y, p0.z);

		for (int i = 1; i < cloudTarget->size(); ++i)
		{
			PointType p2 = cloudTarget->at(i);
			float dist = Util::euclideanDistance2(p.x, p.y, p.z, p2.x, p2.y, p2.z);
			if (dist < minDist)
			{
				minDist = dist;
				closestPointIndex = i;
			}
		}

		return closestPointIndex;
	}
	*/

	static int findClosestPoint(PointType p, CloudType::Ptr cloudTarget) {
		pcl::KdTree<PointType>::Ptr tree (new pcl::KdTreeFLANN<PointType>);
		tree->setInputCloud(cloudTarget);
		int k = 1;

		std::vector<int> nn_indices (k);
		std::vector<float> nn_dists (k);

		tree->nearestKSearch(p, k, nn_indices, nn_dists);

		return nn_indices[0];
	}

	static bool comparePoints(cv::Point p1, cv::Point p2) {
		return p1.x == p2.x and p1.y == p2.y;
	}

	static std::vector<cv::Point> achaPontosUV(string XMLFileName) {
		std::vector<cv::Point> uvPoints;

		XMLDocument doc;
		// doc.LoadFile( "../testCode1_writeVTs/uv.svg" );
		doc.LoadFile(XMLFileName.c_str());

		XMLElement* child = doc.FirstChildElement(); //<svg>
		child = child->FirstChildElement(); //<desc>
		// child = child->NextSiblingElement(); //<polygon>

		for (child = child->NextSiblingElement(); child != NULL; child = child->NextSiblingElement())
		{
			string pointsStr = child->Attribute("points");

			Tokenizer tokenizer1(pointsStr, ' ');
			std::vector<string> polygonVerticesStr = tokenizer1.remaining();
			polygonVerticesStr.pop_back();

			// varrendo pontos (u,v)
			for (int i = 0; i < polygonVerticesStr.size(); ++i)
			{
				string ptCoord = polygonVerticesStr[i];
				Tokenizer tokenizer2(ptCoord, ',');
				std::vector<string> pointCoordinatesStr = tokenizer2.remaining();

				string xCoordinateStr = pointCoordinatesStr[0];
				float x = atof(xCoordinateStr.c_str());
				// float xNormalized = x/1024.0;

				string yCoordinateStr = pointCoordinatesStr[1];
				float y = atof(yCoordinateStr.c_str());
				// float yNormalized = y/1024.0;

				cv::Point pt(x, y);

				// std::vector<cv::Point>::iterator found = std::find_if(uvPoints.begin(), uvPoints.end(), [&](cv::Point& p)
				// {
				// 	return pt.x == p.x and pt.y == p.y;
				// });

				// std::vector<cv::Point>::iterator found = std::find_if(uvPoints.begin(), uvPoints.end(), comparePoints(pt));
				std::vector<cv::Point>::iterator found = std::find(uvPoints.begin(), uvPoints.end(), pt);

				if ( found == uvPoints.end() ) {
					uvPoints.push_back(pt);
				}
			}
		}

		return uvPoints;
	}

	static std::vector<int> achaIndices(string meanFaceFileName) {
		string line;
		// ifstream file("../testCode1_writeVTs/meanFace.obj");
		ifstream file(meanFaceFileName.c_str());
		std::vector<int> indices;
		int i = 0;

		if (file.is_open())
		{
			while ( getline (file,line) )
			{
				Tokenizer tokenizer(line, ' ');
				std::vector<string> lineSplited = tokenizer.remaining();

				if (lineSplited[0] == "f")
				{
					Tokenizer subTokenizer1(lineSplited[1], '/');
					string indexStr = subTokenizer1.remaining()[0];
					int index = atoi(indexStr.c_str());

					if ( std::find(indices.begin(), indices.end(), index) == indices.end() ) {
						indices.push_back(index);
					}

					Tokenizer subTokenizer2(lineSplited[2], '/');
					indexStr = subTokenizer2.remaining()[0];
					index = atoi(indexStr.c_str());

					if ( std::find(indices.begin(), indices.end(), index) == indices.end() ) {
						indices.push_back(index);
					}

					Tokenizer subTokenizer3(lineSplited[3], '/');
					indexStr = subTokenizer3.remaining()[0];
					index = atoi(indexStr.c_str());

					if ( std::find(indices.begin(), indices.end(), index) == indices.end() ) {
						indices.push_back(index);
					}
				}
			}
			file.close();
		}
		else cout << "Unable to open file";

		return indices;
	}

	static std::map<int, cv::Point> createMap(std::vector<cv::Point> points, std::vector<int> indices) {
		std::map<int, cv::Point> map;

		for (int i = 0; i < points.size(); ++i)
		{
			cv::Point p = points[i];
			int index = indices[i];
			map.insert(std::map<int, cv::Point>::value_type(index-1, p));
		}

		return map;
	}

	// preenche imagem vazia a partir de uma nuvem de pontos colorida
	static void fillBlankImage(CloudType::Ptr faceMesh, CloudType::Ptr fullCloud, string blankImgFileName, string pathToSave) {
		ifstream file("../resources/correspondencias.txt");

		if (not file.is_open())
		{
			cerr << "(!!!) Erro ao tentar abrir arquivo de correspondências!" << endl;
			return;
		}

		cv::Mat blankImage = cv::imread(blankImgFileName);
		cv::Mat distancesImage = blankImage;

		if (blankImage.empty())
		{
			cout << "!!! Failed imread(): \"" << blankImgFileName << "\" not found" << endl;
			return;
		}

		std::vector<cv::Point> points = achaPontosUV("../resources/uvMap.svg");
		std::vector<int> indices = achaIndices("../resources/meanFace.obj");
		std::map<int, cv::Point> myMap = createMap(points, indices);
		std::map<float, cv::Point> map;

		clock_t begin = clock();

		pcl::KdTree<PointType>::Ptr tree (new pcl::KdTreeFLANN<PointType>);
		tree->setInputCloud(fullCloud);
		int k = 20;

		std::vector<int> nn_indices (k);
		std::vector<float> nn_dists (k);

		std::setlocale(LC_NUMERIC, "C");
		string line;
		// int cont = 0;
		while ( getline (file,line) )
		{
			// cout << i << " -> " << line << endl;
			// ler alfas e indices (indices começam em 1, cuidado!!!)
			// linha do arquivo de correspondencias: "p alfa1 alfa2 alfa3 indiceV1 indiceV2 indiceV3"
			Tokenizer tokenizer(line, ' ');
			std::vector<string> lineSplited = tokenizer.remaining();

			float alpha1 = atof(lineSplited[1].c_str());
			float alpha2 = atof(lineSplited[2].c_str());
			float alpha3 = atof(lineSplited[3].c_str());

			if (isnan(alpha1) or isnan(alpha2) or isnan(alpha3))
			{
				continue;
			}

			int v1Index = atoi(lineSplited[4].c_str()) - 1;
			int v2Index = atoi(lineSplited[5].c_str()) - 1;
			int v3Index = atoi(lineSplited[6].c_str()) - 1;

			// pegar (do modelo modificado) vertices indicados pelos indices
			PointType v1 = faceMesh->at(v1Index);
			PointType v2 = faceMesh->at(v2Index);
			PointType v3 = faceMesh->at(v3Index);

			// calcular ponto P com esses vertices (p = alpha1*v1 + alpha2*v2 + alpha3*v3)
			float x = alpha1*v1.x + alpha2*v2.x + alpha3*v3.x;
			float y = alpha1*v1.y + alpha2*v2.y + alpha3*v3.y;
			float z = alpha1*v1.z + alpha2*v2.z + alpha3*v3.z;

			PointType p = createPoint(x, y, z);

			// acha pixel correspondente a este ponto

			int pixelX = alpha1*myMap[v1Index].x + alpha2*myMap[v2Index].x + alpha3*myMap[v3Index].x;
			int pixelY = alpha1*myMap[v1Index].y + alpha2*myMap[v2Index].y + alpha3*myMap[v3Index].y;

			// cout << "pixel: (" << pixelX << "," << pixelY << ")" << endl;

			// procurar (na nuvem cheia) ponto mais proximo a P



			// clock_t temp_begin = clock();
			// int index = findClosestPoint(p, fullCloud);
			tree->nearestKSearch(p, k, nn_indices, nn_dists);

			pcl::CentroidPoint<PointType> cp;

			for (int i = 0; i < k; ++i)
			{
				int index = nn_indices[i];
				// clock_t temp_end = clock();

				PointType closestPoint = fullCloud->at(index);

				cp.add(closestPoint);

				// pintar pixel (na imagem vazia) com as cores do ponto mais proximo
				cv::Vec3b& pt = blankImage.at<cv::Vec3b>(cv::Point(pixelX, pixelY));
				pt[0] += (int)closestPoint.b / k;
				pt[1] += (int)closestPoint.g / k;
				pt[2] += (int)closestPoint.r / k;
			}

			PointType centroid;
			cp.get(centroid);

			float distance = pcl::euclideanDistance(p, centroid);

			map.insert(std::map<float, cv::Point>::value_type(distance, cv::Point(pixelX, pixelY)));

			// cont++;
			// if (cont % 10000 == 0)
			// {
			// 	clock_t end = clock();
			// 	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
			// 	cout << "linha " << cont << ": " << elapsed_secs << " segundos (" << elapsed_secs/60 << " minutos)" << endl;
			// }
		}

		// espelhar imagem em torno do eixo X
		cv::Mat filledImage;
		cv::flip(blankImage, filledImage, 0);
		imwrite(pathToSave + "texture.png", filledImage);

		float maxDist = 1.0;
		float groupSize = maxDist/255;
		for (std::map<float, cv::Point>::iterator iterator = map.begin(); iterator != map.end(); ++iterator)
		{
			// distancia dos pontos vem em metros, converter em centímetros
			float dist = iterator->first * 100;
			int redLevel = dist / groupSize;
			if (redLevel > 255)
				redLevel = 255;
			int blueLevel = 255 - redLevel;

			cv::Vec3b& pt = distancesImage.at<cv::Vec3b>(iterator->second);

			// BGR
			pt[0] = blueLevel;
			pt[1] = 0;
			pt[2] = redLevel;
		}

		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		cout << "Terminou em " << elapsed_secs << " segundos (" << elapsed_secs/60 << " minutos)" << endl;

		cv::flip(distancesImage, filledImage, 0);
		imwrite(pathToSave + "distancesTexture.png", filledImage);

		file.close();
	}

	static void colorDistances(CloudType::Ptr faceMesh, CloudType::Ptr fullCloud, string imageFileName) {
		ifstream file("../resources/correspondencias.txt");

		if (not file.is_open())
		{
			cerr << "(!!!) Erro ao tentar abrir arquivo de correspondências!" << endl;
			return;
		}

		cv::Mat blankImage = cv::imread(imageFileName);

		if (blankImage.empty())
		{
			cout << "!!! Failed imread(): \"" << imageFileName << "\" not found" << endl;
			return;
		}

		std::vector<cv::Point> points = achaPontosUV("../resources/uvMap.svg");
		std::vector<int> indices = achaIndices("../resources/meanFace.obj");
		std::map<int, cv::Point> myMap = createMap(points, indices);

		clock_t begin = clock();




		pcl::KdTree<PointType>::Ptr tree (new pcl::KdTreeFLANN<PointType>);
		tree->setInputCloud(fullCloud);
		int k = 10;

		std::vector<int> nn_indices (k);
		std::vector<float> nn_dists (k);


		std::map<float, cv::Point> map;


		std::setlocale(LC_NUMERIC, "C");
		string line;
		int cont = 0;
		while ( getline (file,line) )
		{
			// cout << i << " -> " << line << endl;
			// ler alfas e indices (indices começam em 1, cuidado!!!)
			// linha do arquivo de correspondencias: "p alfa1 alfa2 alfa3 indiceV1 indiceV2 indiceV3"
			Tokenizer tokenizer(line, ' ');
			std::vector<string> lineSplited = tokenizer.remaining();

			float alpha1 = atof(lineSplited[1].c_str());
			float alpha2 = atof(lineSplited[2].c_str());
			float alpha3 = atof(lineSplited[3].c_str());

			if (isnan(alpha1) or isnan(alpha2) or isnan(alpha3))
			{
				continue;
			}

			int v1Index = atoi(lineSplited[4].c_str()) - 1;
			int v2Index = atoi(lineSplited[5].c_str()) - 1;
			int v3Index = atoi(lineSplited[6].c_str()) - 1;

			// pegar (do modelo modificado) vertices indicados pelos indices
			PointType v1 = faceMesh->at(v1Index);
			PointType v2 = faceMesh->at(v2Index);
			PointType v3 = faceMesh->at(v3Index);

			// calcular ponto P com esses vertices (p = alpha1*v1 + alpha2*v2 + alpha3*v3)
			float x = alpha1*v1.x + alpha2*v2.x + alpha3*v3.x;
			float y = alpha1*v1.y + alpha2*v2.y + alpha3*v3.y;
			float z = alpha1*v1.z + alpha2*v2.z + alpha3*v3.z;

			PointType p = createPoint(x, y, z);

			// acha pixel correspondente a este ponto

			int pixelX = alpha1*myMap[v1Index].x + alpha2*myMap[v2Index].x + alpha3*myMap[v3Index].x;
			int pixelY = alpha1*myMap[v1Index].y + alpha2*myMap[v2Index].y + alpha3*myMap[v3Index].y;

			// cout << "pixel: (" << pixelX << "," << pixelY << ")" << endl;

			// procurar (na nuvem cheia) ponto mais proximo a P



			// clock_t temp_begin = clock();
			// int index = findClosestPoint(p, fullCloud);
			tree->nearestKSearch(p, k, nn_indices, nn_dists);

			pcl::CentroidPoint<PointType> cp;

			for (int i = 0; i < k; ++i)
			{
				int pointIndex = nn_indices[i];
				PointType p = fullCloud->at(pointIndex);
				cp.add(p);
			}

			PointType centroid;
			cp.get(centroid);

			float distance = pcl::euclideanDistance(p, centroid);

			map.insert(std::map<float, cv::Point>::value_type(distance, cv::Point(pixelX, pixelY)));

		}

		cout << map.size() << endl;

		int groupSize = map.size()/256;
		int i = 0, j = 0;
		for (std::map<float, cv::Point>::iterator iterator = map.begin(); iterator != map.end(); ++iterator)
		{
			cv::Vec3b& pt = blankImage.at<cv::Vec3b>(iterator->second);
			// BGR
			pt[0] = 255 - j;
			pt[1] = 0;
			pt[2] = j;

			i++;
			if (i >= groupSize) {
				i = 0;
				j++;
			}
		}

		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		cout << "Terminou em " << elapsed_secs << " segundos (" << elapsed_secs/60 << " minutos)" << endl;

		// espelhar imagem em torno do eixo X
		cv::Mat filledImage;
		cv::flip(blankImage, filledImage, 0);

		imwrite("../MM_Restricted_PCA/automatic_tests/texture.png", filledImage);
		file.close();
	}
};

#endif