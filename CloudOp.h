#ifndef CLOUD_OP_H
#define CLOUD_OP_H

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
	
	static int findClosestPoint(PointType p, CloudType::Ptr cloudTarget) {
		int closestPointIndex = -1;
		float minDist = 9999;

		for (int i = 0; i < cloudTarget->size(); ++i)
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
};

#endif