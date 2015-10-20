/* pcl includes */
#include <pcl/io/openni_grabber.h>							// kinect grabber
#include <pcl/visualization/pcl_visualizer.h>				// pcl visualizer

/* opencv includes */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/* standard includes */
#include <iostream>
#include <sstream>
#include <string>
using namespace std;

/* my project includes */
#include "Scan3D.h"			// nuvem de pontos e outras informações
#include "CloudOp.h"		// operações entre nuvens
#include "CloudIO.h"		// leitura e escrita de nuvens em .pcd e .off
#include "Detector.h"		// detecção de features em imagens com opencv
#include "Util.h"			// funções uteis em situações variadas

#define MAX_CLOUDS_NUMBER 20 // número máximo de nuvens salvas para fazer o rigid/icp

typedef pcl::visualization::PCLVisualizer Viewer;
typedef boost::shared_ptr<Viewer> ViewerPtr;

// globais
bool realtime = true;
int waitTime = 100;
Scan3D* faceScan = new Scan3D();		// nuvem corrente
Scan3D* fullCloudScan = new Scan3D();	// nuvem cheia/densa
Scan3D* faceMesh = new Scan3D();		// malha corrente
cv::Mat frameBGR(480, 640, CV_8UC3);	// frame RGB corrente (que no caso é BGR)
vector<Scan3D> partialFaces;
ViewerPtr viewer(new Viewer("PCL Visualizer"));

int cont = 0;

void showCloud(CloudType::Ptr cloud, string cloudId, int viewport) {
	if (not viewer->updatePointCloud(cloud, cloudId))
		viewer->addPointCloud(cloud, cloudId, viewport);
}

void showMesh(CloudType::Ptr points, vector<pcl::Vertices> faces, string meshId, int viewport) {
	if (not viewer->updatePolygonMesh<PointType>(points, faces, meshId))
		viewer->addPolygonMesh<PointType>(points, faces, meshId, viewport);
}

// keyboard event handler
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
	/*ViewerPtr viewer = *static_cast<ViewerPtr *> (viewer_void);*/

	if (event.keyDown()) {
		// tecla ESC para sair do cloud viewer
		if (event.getKeyCode() == 27) {
			cout << "ESC [" << event.getKeySym() << "] was pressed! Closing the program..." << endl;
			exit(0);
		}

		// tecla ENTER para salvar (em memória) a nuvem de pontos corrente
		if (event.getKeyCode() == 13) {
			cout << "ENTER [" << event.getKeySym() << "] was pressed! Saving point cloud...";
			partialFaces.push_back(faceScan->clone());
			cout << " -> Nuvens salvas: " << partialFaces.size() << endl;
		}

		// tecla P para pausar
		if (event.getKeySym() == "p") waitTime == 0 ? waitTime = 100 : waitTime = 0;

		// tecla T para alternar entre modos "realtime" e "cloud loader"
		if (event.getKeySym() == "t") realtime = !realtime;

		// tecla R para registrar nuvens salvas em memória
		if (event.getKeySym() == "r") {
			cout << "R was pressed! Applying registration to clouds saved...";
			
			if (partialFaces.size() < 2)
			{
				cerr << " -> I need at least 2 (two) clouds saved to apply registration! Clouds saved: " << partialFaces.size() << endl;
				return;
			}

			Scan3D basis = partialFaces.front();
			showCloud(basis.cloud, "partialFace0_vp2", 2);
			showCloud(basis.cloud, "partialFace0", 3);

			for (int i = 1; i < partialFaces.size(); ++i)
			{
				Scan3D partialFace = partialFaces[i];

				std::ostringstream ss;
				ss << "partialFace" << i;

				showCloud(partialFace.cloud, ss.str()+"_vp2", 2);

				// partialFace.rigidICPRegistration(basis);
				partialFace.rigidRegistration(basis);

				showCloud(partialFace.cloud, ss.str(), 3);
			}

			cout << " -> All point clouds registered (using first as basis)!" << endl;
		}

		// tecla C para apagar todas as nuvens
		if (event.getKeySym() == "c") {
			cout << "C was pressed! Removing every point cloud in the viewer...";
			viewer->removeAllPointClouds();
			partialFaces.clear();
			// fullCloudScan = new Scan3D();
			cout << " -> All point clouds removed!" << endl;
		}

		// tecla S para salvar (em arquivo) a nuvem de pontos corrente
		if (event.getKeySym() == "s") {
			cout << "S was pressed! Exporting point cloud and landmarks list...";
			
			string offFileName = "../MM_Restricted_PCA/automatic_tests/point_cloud.off";
			string fullCloudFileName = Util::changeSuffix(offFileName, "_cheia.off");
			string landmarksFileName = Util::changeSuffix(offFileName, "_landmarks.txt");

			// std::ostringstream ss;
			// ss << cont;
			// offFileName = Util::changeSuffix(offFileName, ss.str()+".off");
			// landmarksFileName = Util::changeSuffix(offFileName, "_landmarks.txt");

			// faceScan->save(offFileName, landmarksFileName);
			// faceScan->save(fullCloudFileName, landmarksFileName); // <- deveria ser a nuvem cheia!
			partialFaces.front().save(offFileName, landmarksFileName); // nuvem base
			fullCloudScan->save(fullCloudFileName, landmarksFileName); // nuvem cheia

			cout << " -> Finished cloud saving!" << endl;

			/* chamar o programa que cria o modelo, passando a nuvem salva como parâmetro */

			if (event.isCtrlPressed())
			{
				string modelFileName = Util::changeSuffix(offFileName, "_modelo.off");
				// string cmdLine = "sh ../modelAlign.sh " + offFileName + " " + landmarksFileName + " " + modelFileName;
				string cmdLine = "sh ../modelAlign.sh " + fullCloudFileName + " " + landmarksFileName + " " + modelFileName;
				system(cmdLine.c_str());

				/* mostrar nuvem corrente salva, nuvem cheia e modelo criado nos viewports */
				
				Scan3D baseCloud(offFileName, landmarksFileName);
				Scan3D fullCloud(fullCloudFileName, landmarksFileName);
				faceMesh = new Scan3D(modelFileName, landmarksFileName);

				// viewport 2 (nuvem base salva)
				showCloud(baseCloud.cloud, "basis", 2);
				
				// viewport 3 (nuvem cheia)
				// viewer->removeAllPointClouds(3);
				// showCloud(fullCloud.cloud, "fullCloud", 3);
				// showCloud(fullCloudScan->cloud, "fullCloud", 3);
				
				// viewport 4 (modelo)
				faceMesh->applyTextureFromCloud(fullCloud);
				faceMesh->save(Util::changeSuffix(modelFileName, "_colorido.off"));
				
				cout << " -> Finished mesh saving!" << endl;
				
				showMesh(faceMesh->cloud, faceMesh->faces, "mesh", 4);
			}

			cont++;
		}
	}
}

class KinectScanner
{
	string rgbWindowName;
	Detector* detector;
	
	cv::Rect face;
	vector<cv::Point> landmarks, cvFacePoints;

public:

	// construtor
	KinectScanner() {
		viewer->setSize(1024, 768);
		viewer->registerKeyboardCallback(keyboardEventOccurred);

		rgbWindowName = "Imagem RGB";
		cv::namedWindow(rgbWindowName);
		detector = new Detector();
	}

	// callback da nuvem de pontos
	void cloudCallback(const CloudType::ConstPtr& ambientCloud) {
		if (not realtime or not face.area()) return;

		// faceScan->cropFaceFromAmbient(ambientCloud, face.x, face.y, face.height, face.width);
		*(faceScan->cloud) = *ambientCloud;
		faceScan->getPointsInsidePolygon(cvFacePoints);
		faceScan->updateLandmarks(landmarks, face);
		// faceScan->markFaceLandmarks();
		showCloud(faceScan->cloud, "cloud", 1);

		tryRegistration();
	}

	// callback da imagem rgb
	void rgbCallback(const boost::shared_ptr<openni_wrapper::Image>& img) {
		if (not realtime) return;

		cv::Mat frameRGB = cv::Mat(img->getHeight(), img->getWidth(), CV_8UC3);
		img->fillRGB(frameRGB.cols, frameRGB.rows, frameRGB.data, frameRGB.step);
		cv::cvtColor(frameRGB, frameBGR, CV_RGB2BGR);

		face = detector->trackFace(frameBGR);

		if (face.area())
		{
		// // landmarks dos olhos e do nariz
			int arr1[] = {36,39, 42,45, 30,31,35,33};
			vector<int> landmarkIndices(arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );
			landmarks = detector->findLandmarks(landmarkIndices);

		// landmarks para separar face do resto
			int arr[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16, 26,25,24, 19,18,17, 0};
			int arrLength = sizeof(arr) / sizeof(arr[0]);
			vector<int> faceContourIndices(arr, arr + arrLength);

			cvFacePoints = detector->getFacePoints(frameBGR, faceContourIndices);
		}
	}

	/*
	CloudType::Ptr cloudsRegistrationFromFile(string cloudFileName1, string cloudFileName2, bool withICP = false) {
		Scan3D scan1(cloudFileName1, Util::changeSuffix(cloudFileName1, "_landmarks.txt"));
		Scan3D scan2(cloudFileName2, Util::changeSuffix(cloudFileName2, "_landmarks.txt"));

		// remove pontos muito distantes da face
		scan1.removeDistantPoints();
		scan2.removeDistantPoints();

		// rigid registration (scan2 é modificada)
		scan1.rigidRegistration(scan2);

		// mostrar resultado da rigid
		// CloudType::Ptr rigidRegisteredCloud = CloudOp::copyCloud(scan1.cloud);
		viewer->addPointCloud(scan2.cloud, "rigid");

		// icp registration
		if (withICP) {
			scan1.ICPRegistration(scan2);
		}

		return scan1.cloud;
	}
	*/

	ViewerPtr viewportsViewer(CloudType::Ptr cloud1, CloudType::Ptr cloud2) {
		ViewerPtr viewer(new Viewer("Nuvem de Pontos"));

		viewer->initCameraParameters ();

		int v1(0);
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor (0, 0, 0, v1);
		viewer->addText("viewport 1", 10, 10, "v1 text", v1);
		viewer->addPointCloud(cloud1, "sample cloud1", v1);

		int v2(0);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
		viewer->addText("viewport 2", 10, 10, "v2 text", v2);
		viewer->addPointCloud(cloud2, "sample cloud2", v2);

		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
		viewer->addCoordinateSystem (1.0);

		return viewer;
	}

	void makeViewportBasicConfigs(string viewportText, int viewport = 0, float bgColor_r = 0.0, float bgColor_g = 0.0, float bgColor_b = 0.0) {
		viewer->setBackgroundColor(bgColor_r, bgColor_g, bgColor_b, viewport);
		viewer->addCoordinateSystem(.1, "axes", viewport);

		// texto e id do viewport
		std::ostringstream viewportTextId;
		viewportTextId << "viewport" << viewport << " text";
		viewer->addText(viewportText, 10, 10, viewportTextId.str(), viewport);

		// texto, posição, escala (tamanho do texto), r, g, b, textId, viewport
		viewer->addText3D("X", pcl::PointXYZ(0.1, 0.0, 0.0), 0.005, 1.0, 1.0, 1.0, "", viewport);
		viewer->addText3D("Y", pcl::PointXYZ(0.0, 0.1, 0.0), 0.005, 1.0, 1.0, 1.0, "", viewport);
		viewer->addText3D("Z", pcl::PointXYZ(0.0, 0.0, 0.1), 0.005, 1.0, 1.0, 1.0, "", viewport);

		// posição inicial, posição onde está olhando, vetor up
		viewer->setCameraPosition(
			0.0, 0.0, -0.5,
			0.0, 0.0, 1.0,
			0.0, -1.0, 0.0,
			viewport);
	}

	void splitScreen(string v1Text, string v2Text) {
		int v1(1);
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		makeViewportBasicConfigs(v1Text, v1, 0.0, 0.0, 0.0);

		int v2(2);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		makeViewportBasicConfigs(v2Text, v2, 0.1, 0.1, 0.1);
	}

	void splitScreen(string v1Text, string v2Text, string v3Text) {
		int v1(1);
		viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
		makeViewportBasicConfigs(v1Text, v1, 0.0, 0.0, 0.0);

		int v2(2);
		viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
		makeViewportBasicConfigs(v2Text, v2, 0.1, 0.1, 0.1);

		int v3(3);
		viewer->createViewPort(0.0, 0.0, 1.0, 0.5, v3);
		makeViewportBasicConfigs(v3Text, v3, 0.2, 0.2, 0.2);
	}

	void splitScreen(string v1Text, string v2Text, string v3Text, string v4Text) {
		int v1(1);
		viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
		makeViewportBasicConfigs(v1Text, v1, 0.0, 0.0, 0.0);

		int v2(2);
		viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);
		makeViewportBasicConfigs(v2Text, v2, 0.1, 0.1, 0.1);

		int v3(3);
		viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
		makeViewportBasicConfigs(v3Text, v3, 0.2, 0.2, 0.2);

		int v4(4);
		viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
		makeViewportBasicConfigs(v4Text, v4, 0.3, 0.3, 0.3);
	}

	// cria visualizer com keybooard callback
	/*
	ViewerPtr createViewer() {
		ViewerPtr viewer(new Viewer("Nuvem de Pontos"));
		viewer->setSize(1024, 768);
		viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
		return viewer;
	}
	*/

	void tryRegistration() {
		// precisa ter pelo menos uma face salva
		if (partialFaces.size() > 0)
		{
			Scan3D basis = partialFaces.front();
			fullCloudScan->cloud = CloudOp::copyCloud(basis.cloud);
			fullCloudScan->eyeLandmarks = basis.eyeLandmarks;
			fullCloudScan->noseLandmarks = basis.noseLandmarks;

			// se ainda nao chegou no numero máximo de nuvens parciais, salva a atual e já faz o registro da mesma
			if (partialFaces.size() < MAX_CLOUDS_NUMBER)
			{
				partialFaces.push_back(faceScan->clone());
				Scan3D partialFace = partialFaces.back();

				partialFace.rigidICPRegistration(basis);

				CloudOp::addClouds(fullCloudScan->cloud, partialFace.cloud);

				std::ostringstream ss;
				ss << "partialFace" << partialFaces.size();
				cout << "partial face (" << partialFaces.size() << "/" << MAX_CLOUDS_NUMBER << ") registered" << endl;
				showCloud(partialFace.cloud, ss.str(), 3);
			}
			// caso contrario, apaga nuvens parciais salvas
			else {
				// partialFaces.clear();
				// viewer->removeAllPointClouds(2);
			}
		} else {
			partialFaces.push_back(faceScan->clone());
			cout << "basis saved! (1/" << MAX_CLOUDS_NUMBER << ")" << endl;
			showCloud(partialFaces.front().cloud, "basis", 2);
			showCloud(partialFaces.front().cloud, "basis", 3);
		}
	}

	void registerCallbacks(pcl::Grabber* interface) {
		boost::function<void (const CloudType::ConstPtr&)> cloudFunc = boost::bind(&KinectScanner::cloudCallback, this, _1);
		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> rgbFunc = boost::bind(&KinectScanner::rgbCallback, this, _1);

		interface->registerCallback(cloudFunc);
		interface->registerCallback(rgbFunc);
	}

	// função de testes para trabalhar com nuvens salvas em arquivos
	void testFunc(string facesDir = "../MM_Restricted_PCA/automatic_tests/") {
		/* carrega nuvens */

		string cloud0FileName = facesDir + "point_cloud0.off";
		string cloud0Landmarks = facesDir + "point_cloud0_landmarks.txt";
		Scan3D cloud0(cloud0FileName, cloud0Landmarks);
		
		string cloud1FileName = facesDir + "point_cloud1.off";
		string cloud1Landmarks = facesDir + "point_cloud1_landmarks.txt";
		Scan3D cloud1(cloud1FileName, cloud1Landmarks);
		
		string cloud2FileName = facesDir + "point_cloud2.off";
		string cloud2Landmarks = facesDir + "point_cloud2_landmarks.txt";
		Scan3D cloud2(cloud2FileName, cloud2Landmarks);

		string cloud3FileName = facesDir + "point_cloud3.off";
		string cloud3Landmarks = facesDir + "point_cloud3_landmarks.txt";
		Scan3D cloud3(cloud3FileName, cloud3Landmarks);
		
		string cloud4FileName = facesDir + "point_cloud4.off";
		string cloud4Landmarks = facesDir + "point_cloud4_landmarks.txt";
		Scan3D cloud4(cloud4FileName, cloud4Landmarks);

		/* mostra antes de registrar */

		showCloud(cloud0.cloud, "cloud0_before", 1);
		showCloud(cloud1.cloud, "cloud1_before", 1);
		showCloud(cloud2.cloud, "cloud2_before", 1);
		showCloud(cloud3.cloud, "cloud3_before", 1);
		showCloud(cloud4.cloud, "cloud4_before", 1);

		/* registra */

		cloud1.rigidRegistration(cloud0);
		cloud2.rigidRegistration(cloud0);
		cloud3.rigidRegistration(cloud0);
		cloud4.rigidRegistration(cloud0);

		/* mostra depois de registrar */

		showCloud(cloud0.cloud, "cloud0_after1", 2);
		showCloud(cloud1.cloud, "cloud1_after1", 2);
		showCloud(cloud2.cloud, "cloud2_after1", 2);
		showCloud(cloud3.cloud, "cloud3_after1", 2);
		showCloud(cloud4.cloud, "cloud4_after1", 2);

		cloud1.ICPRegistration(cloud0);
		cloud2.ICPRegistration(cloud0);
		cloud3.ICPRegistration(cloud0);
		cloud4.ICPRegistration(cloud0);

		showCloud(cloud0.cloud, "cloud0_after2", 3);
		showCloud(cloud1.cloud, "cloud1_after2", 3);
		showCloud(cloud2.cloud, "cloud2_after2", 3);
		showCloud(cloud3.cloud, "cloud3_after2", 3);
		showCloud(cloud4.cloud, "cloud4_after2", 3);
	}

	void testFunc2() {
		Scan3D cloud, polygon;

		polygon.addPoint(CloudOp::createPoint(1,1,0, 255,0,0));
		polygon.addPoint(CloudOp::createPoint(1.5,0,0, 255,0,0));
		polygon.addPoint(CloudOp::createPoint(2.5,0,0, 255,0,0));
		polygon.addPoint(CloudOp::createPoint(3,1,0, 255,0,0));
		polygon.addPoint(CloudOp::createPoint(1,1,0, 255,0,0));

		cloud.addPoint(CloudOp::createPoint(1,   0.5, 0));
		cloud.addPoint(CloudOp::createPoint(1.9, 0.4, 0));
		cloud.addPoint(CloudOp::createPoint(1.9, 0.5, 0));
		cloud.addPoint(CloudOp::createPoint(1.9, 0.6, 0));
		cloud.addPoint(CloudOp::createPoint(2,   0.4, 0));
		cloud.addPoint(CloudOp::createPoint(2,   0.5, 0));
		cloud.addPoint(CloudOp::createPoint(2,   0.6, 0));
		cloud.addPoint(CloudOp::createPoint(2,  -0.5, 0));
		cloud.addPoint(CloudOp::createPoint(2,   1.5, 0));
		cloud.addPoint(CloudOp::createPoint(2.1, 0.4, 0));
		cloud.addPoint(CloudOp::createPoint(2.1, 0.5, 0));
		cloud.addPoint(CloudOp::createPoint(2.1, 0.6, 0));
		cloud.addPoint(CloudOp::createPoint(2.9, 0.5, 0));
		cloud.addPoint(CloudOp::createPoint(2.9, 0.4, 0));
		cloud.addPoint(CloudOp::createPoint(2.9, 0.6, 0));
		cloud.addPoint(CloudOp::createPoint(3,   0.5, 0));
		cloud.addPoint(CloudOp::createPoint(3,   0.4, 0));
		cloud.addPoint(CloudOp::createPoint(3,   0.6, 0));

		for (int i = 0; i < cloud.cloud->size(); ++i)
		{
			PointType& pt = cloud.cloud->at(i);

			cout << "ponto (" << pt.x << ", " << pt.y << ", " << pt.z << ") " << endl;
			if (pcl::isXYPointIn2DXYPolygon(pt, *polygon.cloud))
			{
				pt.r = 255;
				pt.g = 0;
				pt.b = 0;
			}
			else {
				cout << "~~NÃO~~" << endl; 
			}
			cout << " está dentro do polígono" << endl;
		}

		showCloud(cloud.cloud, "cloud", 3);
		showCloud(polygon.cloud, "polygonPoints", 3);
		viewer->addPolygon<PointType>(polygon.cloud, 255, 0, 0, "polygon", 3);
	}

	void testFunc3() {
		/* create a RGB colour image (set it to a black background) */

		cv::Mat img = cv::Mat::zeros(400, 400, CV_8UC3);

		/* define a polygon (as a vector of points) */

		vector<cv::Point> contour;
		contour.push_back(cv::Point(50,50));
		contour.push_back(cv::Point(300,50));
		contour.push_back(cv::Point(350,200));
		contour.push_back(cv::Point(300,150));
		contour.push_back(cv::Point(150,350));
		contour.push_back(cv::Point(100,100));

		/* create a pointer to the data as an array of points (via a conversion to a Mat() object) */

		const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
		int npts = cv::Mat(contour).rows;

		std::cout << "Number of polygon vertices: " << npts << std::endl;

		/* draw the polygon */
		cv::polylines(img, &pts,&npts, 1, true, cv::Scalar(0,255,0), 3, CV_AA, 0);


		/* do point in polygon test (by conversion/cast to a Mat() object) */

		cv::Point2f test_pt;
		test_pt.x = 150;
		test_pt.y = 75;

		cv::rectangle(img, test_pt, test_pt, cv::Scalar(0, 0, 255), 3, 8, 0); // RED point
		
		if (cv::pointPolygonTest(cv::Mat(contour), test_pt, true) > 0) {
			std::cout << "RED {" << test_pt.x << "," << test_pt.y << "} is in the polygon (dist. " << cv::pointPolygonTest(cv::Mat(contour), test_pt, 1) << ")" << std::endl;
		}

		// define and test point two (draw it in blue)

		test_pt.x = 50;
		test_pt.y = 350;

		cv::rectangle(img, test_pt, test_pt, cv::Scalar(255, 0, 0), 3, 8, 0); // BLUE point
		
		if (cv::pointPolygonTest(cv::Mat(contour), test_pt, true) < 0) {
			std::cout << "BLUE {" << test_pt.x << "," << test_pt.y << "} is NOT in the polygon (dist. " << cv::pointPolygonTest(cv::Mat(contour), test_pt, 1) << ")" << std::endl;
		}

		// pointPolygonTest :- 
		// "The function determines whether the point is inside a contour, outside, 
		// or lies on an edge (or coincides with a vertex). It returns positive 
		// (inside), negative (outside) or zero (on an edge) value, correspondingly. 
		// When measureDist=false , the return value is +1, -1 and 0, respectively. 
		// Otherwise, the return value it is a signed distance between the point 
		// and the nearest contour edge." - OpenCV Manual version 2.1

		// create an image and display the image

		cv::namedWindow("Polygon Test", 0);
		cv::imshow( "Polygon Test", img );
		cv::waitKey(0);
	}

	// run :D
	void run() {
		pcl::Grabber* interface = new pcl::OpenNIGrabber();
		registerCallbacks(interface);

		splitScreen("viewport 1", "viewport 2", "viewport 3", "viewport 4");

		// testFunc();

		interface->start();
		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
			cv::imshow(rgbWindowName, frameBGR);	// Show our image inside it.
			int c = cv::waitKey(waitTime);			// Wait for a keystroke in the window (0 = inf)
			if(char(c) == 'd') detector->reset();	// reseta o facetracker
			if(char(c) == 27) exit(0);				// ESC
		}
		interface->stop();
	}
};

int main(int argc, char const *argv[])
{
	KinectScanner ks;
	ks.run();
	return 0;
}