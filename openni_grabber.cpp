/* pcl includes */
#include <pcl/io/openni_grabber.h>				// kinect grabber
#include <pcl/visualization/cloud_viewer.h>		// cloud viewer
#include <pcl/visualization/pcl_visualizer.h>	// pcl visualizer

/* opencv includes */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/* standard includes */
#include <iostream>
#include <sstream>
#include <string>
#include <stdlib.h>		// atof
#include <locale.h>		// setlocale
using namespace std;

/* my project includes */
#include "Scan3D.h"			// nuvem de pontos e outras informações
#include "CloudOp.h"		// operações entre nuvens
#include "CloudIO.h"		// leitura e escrita de nuvens em .pcd e .off
#include "Detector.h"		// detecção de features em imagens com opencv
#include "Util.h"			// funções uteis em situações variadas

#define MAX_CLOUDS_NUMBER 5 // número máximo de nuvens salvas para fazer o rigid/icp

typedef pcl::visualization::CloudViewer				Viewer;
typedef boost::shared_ptr<Viewer>					ViewerPtr;
typedef pcl::visualization::PCLVisualizer			PCLVis;
typedef boost::shared_ptr<openni_wrapper::Image>	ImageType;

// globais
Scan3D* tmpCloud = new Scan3D();
Scan3D* faceScan = new Scan3D();		// nuvem corrente
Scan3D* baseCloudScan = new Scan3D();	// nuvem base
Scan3D* fullCloudScan = new Scan3D();	// nuvem cheia
Scan3D* faceMesh = new Scan3D();		// malha resultado
Scan3D* texFaceMesh = new Scan3D();			// malha resultado texturizada
ViewerPtr viewer(new Viewer("Point Clouds"));

string rgbWindowName = "RGB Stream";
cv::Mat frameBGR = cv::Mat::zeros(480, 640, CV_8UC3);	// frame RGB corrente (que no caso é BGR)
Detector* detector = new Detector();
cv::Rect face;
vector<cv::Point> facePolygon, landmarks;
int savedClouds = MAX_CLOUDS_NUMBER;
bool displayMesh = true;
bool createMesh = false;

// keyboard event handler
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
	if (event.keyDown()) {
		// tecla ENTER para salvar a nuvem base
		if (event.getKeyCode() == 13) {
			cout << "\tENTER [" << event.getKeySym() << "] was pressed! Saving base cloud..." << endl;

			*baseCloudScan = faceScan->clone();
			*fullCloudScan = baseCloudScan->clone();
			faceMesh = new Scan3D();
			texFaceMesh = new Scan3D();

			cout << "\tThis cloud is good as basis?" << endl;
			cout << "\tIf yes, hit \"Space\"! If not, adjust your face and hit \"Enter\" again!" << endl;
		}

		// tecla ESPAÇO para registrar a nuvem corrente à base e adicionar à nuvem cheia
		if (event.getKeyCode() == 32) {
			if (not baseCloudScan->cloud->size())
			{
				cerr << "\t(!!!) You need to save a cloud as basis first! To save a cloud as basis, hit \"Enter\"!" << endl;
				return;
			}
			// cout << "Full cloud process started!" << endl;
			savedClouds = MAX_CLOUDS_NUMBER-1;
		}

		// tecla Z para salvar as nuvens base e cheia em disco (Ctrl+Z para iniciar processo de criação do modelo)
		if (event.getKeySym() == "z") {
			createMesh = true;
			// cout << "\tS was pressed! Exporting point clouds..." << endl;

			// // sempre salva em uma nova pasta baseada no horário corrente
			// string currentDateTime = Util::currentDateTime();
			// string cloudsStdPath = "../automatic_tests/" + currentDateTime + "/";
			// mkdir(cloudsStdPath.c_str(), 0777);
			// string baseCloudName = cloudsStdPath + "base_cloud.off";
			// string fullCloudName = cloudsStdPath + "full_cloud.off";
			// string landmarksFileName = Util::changeSuffix(baseCloudName, "_landmarks.txt");

			// // salvar nuvens para chamar programa externo
			// baseCloudScan->save(baseCloudName, landmarksFileName);

			// // remover pontos com poucos vizinhos (provavelmente são partes pouco escaneadas)
			// fullCloudScan->removeOutliers();
			// fullCloudScan->save(fullCloudName, landmarksFileName);

			// cout << "\t-> Finished clouds saving!" << endl;

			// string modelName = cloudsStdPath + "result_model.off";

			// // chamar o programa que cria o modelo, passando a nuvem cheia salva como parâmetro
			// if (event.isCtrlPressed())
			// {
			// 	string cmdLine = "sh ../modelAlign.sh " + fullCloudName + " " + landmarksFileName + " " + modelName;
			// 	system(cmdLine.c_str());

			// 	// atualiza mesh (sem textura)
			// 	faceMesh = new Scan3D(modelName, landmarksFileName);
			// }
			// else if (event.isShiftPressed())
			// {
			// 	// carrega resultado para o programa, mas ainda está sem texturas
			// 	cout << "\tApplying texture to the mesh..." << endl;

			// 	faceMesh->updateTexData("../resources/meanFace.obj", "texture.png");
			// 	faceMesh->applyTextureFromImage(*fullCloudScan, "../resources/texture.png", cloudsStdPath);

			// 	cout << "\t...DONE!" << endl;

			// 	// salvar modelo agora com textura, em formato obj
			// 	string texModelName = Util::changeSuffix(modelName, "_with_texture.obj");
			// 	faceMesh->save(texModelName);

			// 	cout << "\t-> Finished 3D Face Reconstruction Succefully!" << endl;

			// 	// atualiza o modelo no programa
			// 	displayMesh = true;
			// 	faceMesh = new Scan3D(texModelName, landmarksFileName);
			// }
		}
	}
}

void showSphere(PCLVis& pclVis, PointType p, int r, int g, int b, string id, int viewport) {
	float radius = 0.001;
	if(not pclVis.updateSphere(p, radius, r, g, b, id))
		pclVis.addSphere(p, radius, id, viewport);
}

void showScan(PCLVis& pclVis, Scan3D scan, string cloudId, int viewport) {
	if (not pclVis.updatePointCloud(scan.cloud, cloudId))
		pclVis.addPointCloud(scan.cloud, cloudId, viewport);

	// mostrar landmarks no pclVis
	std::vector<PointType> scanLandmarks = scan.landmarkPoints();
	for (int i = 0; i < scanLandmarks.size(); ++i)
	{
		int r = 0, g = 0, b = 0;

		if ( i < 4 or (i > 7 and i < 12) )	r = 255;
		if (i > 3)							g = 255;
		if (i > 11)							b = 255;

		ostringstream ss;
		ss << "landmark" << i << "_vp" << viewport;
		showSphere(pclVis, scanLandmarks[i], r, g, b, ss.str(), viewport);
	}
}

void showTexMesh(PCLVis& pclVis, pcl::TextureMesh mesh, string meshId, int viewport) {
	if (mesh.cloud.data.size())
	{
		pclVis.removeShape(meshId, viewport);
		pclVis.addTextureMesh(mesh, meshId, viewport);
		displayMesh = false;
	}
}

void showMesh(PCLVis& pclVis, CloudType::Ptr points, vector<pcl::Vertices> faces, string meshId, int viewport) {
	if (points->size())
		if (not pclVis.updatePolygonMesh<PointType>(points, faces, meshId))
			pclVis.addPolygonMesh<PointType>(points, faces, meshId, viewport);
}

void makeViewportBasicConfigs(PCLVis& pclVis, string viewportText, int viewport = 0, float bgColor_r = 0.0, float bgColor_g = 0.0, float bgColor_b = 0.0) {
	// cor de fundo
	pclVis.setBackgroundColor(bgColor_r, bgColor_g, bgColor_b, viewport);

	// texto e id do viewport
	ostringstream viewportTextId;
	viewportTextId << "viewport" << viewport << " text";
	pclVis.addText(viewportText, 10, 10, 12, 1.0, 1.0, 1.0, viewportTextId.str(), viewport);

	// eixos cartesianos
	// viewer->addCoordinateSystem(.1, "axes", viewport);
	// viewer->addText3D("X", pcl::PointXYZ(0.1, 0.0, 0.0), 0.005, 1.0, 1.0, 1.0, "", viewport);
	// viewer->addText3D("Y", pcl::PointXYZ(0.0, 0.1, 0.0), 0.005, 1.0, 1.0, 1.0, "", viewport);
	// viewer->addText3D("Z", pcl::PointXYZ(0.0, 0.0, 0.1), 0.005, 1.0, 1.0, 1.0, "", viewport);

	// ajuste da câmera (posição inicial, posição onde está olhando, vetor up)
	pclVis.setCameraPosition(
		0.0, 0.0, -0.5,
		0.0, 0.0, 1.0,
		0.0, -1.0, 0.0,
		viewport);
}

void splitScreen(PCLVis& pclVis, string v1Text, string v2Text, string v3Text, string v4Text, string v5Text) {
	int v1(1);
	pclVis.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
	makeViewportBasicConfigs(pclVis, v1Text, v1, 0.3, 0.3, 0.3);

	int v2(2);
	pclVis.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
	makeViewportBasicConfigs(pclVis, v2Text, v2, 0.2, 0.2, 0.2);

	int v3(3);
	pclVis.createViewPort(0.0, 0.0, 0.4, 0.5, v3);
	makeViewportBasicConfigs(pclVis, v3Text, v3, 0.2, 0.2, 0.2);

	int v4(4);
	pclVis.createViewPort(0.4, 0.0, 0.7, 0.5, v4);
	makeViewportBasicConfigs(pclVis, v4Text, v4, 0.25, 0.25, 0.25);

	int v5(5);
	pclVis.createViewPort(0.7, 0.0, 1.0, 0.5, v5);
	makeViewportBasicConfigs(pclVis, v5Text, v5, 0.3, 0.3, 0.3);
}

void pclVisualizerInit(PCLVis& pclVis) {
	// tamanho da janela
	pclVis.setSize(1366-640, 480);

	// tela dividida em 4 viewports
	splitScreen(pclVis, "REAL TIME CLOUD", "BASE CLOUD", "FULL CLOUD", "3D RECONSTRUCTION", "TEXTURIZED FACE");
	// splitScreen(pclVis, "", "", "FULL CLOUD", "3D RECONSTRUCTION", "TEXTURIZED FACE");

	// texto de help
	string helpText = "Q - EXIT\nENTER - SET BASE CLOUD\nSPACE - REGISTER CURRENT CLOUD\nZ - SAVE CLOUDS AND CREATE FACE MESH";
	pclVis.addText(helpText, 10, 300, 12, 1.0, 1.0, 1.0, "helpText", 1);

// ////////////////////////////////////////////////////////////////////////////////////////////////
	// string facesDir = "../MM_Restricted_PCA/automatic_tests/";
	// string fullCloudName = facesDir + "full_cloud.off";
	// string landmarksFileName = facesDir + "base_cloud_landmarks.txt";
	// string modelName = facesDir + "result_model.off";

	// Scan3D fullCloud(fullCloudName, landmarksFileName);

	// string cmdLine = "sh ../modelAlign.sh " + fullCloudName + " " + landmarksFileName + " " + modelName;
	// system(cmdLine.c_str());

	// Scan3D mesh(modelName, landmarksFileName);
	// showMesh(pclVis, mesh.cloud, mesh.faces, "whiteMesh", 4);

	// sleep(2);

	// Scan3D mesh2(modelName, landmarksFileName);
	// mesh2.updateTexData("../resources/meanFace.obj", "texture.png");
	// mesh2.applyTextureFromImage(fullCloud, "../resources/texture.png", facesDir);
	// mesh2.save(facesDir + "mesh_result.obj");
	// pclVis.removePolygonMesh("whiteMesh", 4);
	// pclVis.addTextureMesh(mesh.texMesh, "resultMesh", 4);
// ////////////////////////////////////////////////////////////////////////////////////////////////
}

void pclVisualizerViewer(PCLVis& pclVis) {
	showScan(pclVis, *faceScan, "cloud", 1);
	showScan(pclVis, *baseCloudScan, "baseCloud", 2);
	showScan(pclVis, *fullCloudScan, "fullCloud", 3);
	showMesh(pclVis, faceMesh->cloud, faceMesh->faces, "whiteMesh", 4);
	if (displayMesh) {
		showTexMesh(pclVis, texFaceMesh->texMesh, "resultMesh", 5);
	}
}

class KinectScanner
{
public:
	// construtor
	KinectScanner() {
		cv::namedWindow(rgbWindowName);
		viewer->registerKeyboardCallback(keyboardEventOccurred);
		viewer->runOnVisualizationThreadOnce(pclVisualizerInit);
		viewer->runOnVisualizationThread(pclVisualizerViewer);
	}

	// realiza registro rigido e icp da nuvem corrente com a base, formando a nuvem cheia
	void tryRegistration() {
		if (savedClouds < MAX_CLOUDS_NUMBER)
		{
			Scan3D faceScan_tmp = faceScan->clone();
			Scan3D fullCloudScan_tmp = fullCloudScan->clone();

			faceScan_tmp.rigidRegistration(*baseCloudScan);
			CloudOp::addClouds(fullCloudScan_tmp.cloud, faceScan_tmp.cloud);
			*fullCloudScan = fullCloudScan_tmp.clone();
			savedClouds++;

			cout << "\t<[ Current face registered ]>" << endl;

			// if (savedClouds == MAX_CLOUDS_NUMBER)
			// 	cout << "\t<<< DATA COLLECTION COMPLETE! >>>" << endl;
		}
	}

	// thread para dados de nuvens de pontos
	void cloudCallback(const CloudType::ConstPtr& ambientCloud) {
		// se não identificar nenhuma face na imagem, não atualiza o viewer
		if (not face.area()) return;

		// pega nuvem do ambiente corta todos os pontos que não pertencem à face
		pcl::copyPointCloud<PointType, PointType>(*ambientCloud, *(tmpCloud->cloud));
		tmpCloud->getPointsInsidePolygon(facePolygon);

		// se houve algum problema no corte, a nuvem será apagada e devemos esperar o proximo scan
		if (not tmpCloud->cloud->size()) return;

		// ajustar indices dos landmarks
		tmpCloud->updateLandmarks(landmarks, face);

		// atualizar faceScan
		*faceScan = tmpCloud->clone();

		// se estiver com o ctrl+enter ativado, registrar nuvem corrente com a base e adicionar à nuvem cheia
		tryRegistration();
	}

	// converter imagem da pcl para imagem do opencv
	cv::Mat pclImgToOpencvImg(ImageType img) {
		cv::Mat tempImg(img->getHeight(), img->getWidth(), CV_8UC3);

		img->fillRGB(tempImg.cols, tempImg.rows, tempImg.data);
		cv::cvtColor(tempImg, tempImg, CV_RGB2BGR);

		return tempImg;
	}

	// retorna pontos que formam o poligono da face
	std::vector<cv::Point> createFacePolygon() {
		int arr[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16, 26,25,24, 19,18,17, 0};
		int arrLength = sizeof(arr) / sizeof(arr[0]);
		vector<int> faceContour(arr, arr + arrLength);

		// std::vector<cv::Point> result = detector->getFacePoints(frameBGR, faceContour);

		return detector->getFacePoints(frameBGR, faceContour);
	}

	// retorna pontos referentes aos landmarks do rosto usados no registro rigido
	std::vector<cv::Point> getFaceLandmarks() {
		// landmarks (olhos, nariz, boca, queixo)
		int arr1[] = {36,39, 42,45, 30,31,35,33, 61,48,54,64, 8};
		vector<int> landmarkIndices(arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );

		std::vector<cv::Point> faceLandmarks = detector->findLandmarks(landmarkIndices);
		faceLandmarks.back().y -= 3;

		return faceLandmarks;
	}

	// thread para dados de imagem
	void rgbCallback(const ImageType& img) {
		// converter imagem da pcl para imagem do opencv
		cv::Mat tempImg = pclImgToOpencvImg(img);

		// detectar face na imagem
		face = detector->trackFace(tempImg);

		// selecionar pontos dentro do poligono que envolve a face
		if (face.area())
		{
			// indices dos pontos que formam o polígono do rosto
			facePolygon = createFacePolygon();

			// indices dos pontos de interesse para o registro rigido
			landmarks = getFaceLandmarks();
		}

		// atualiza frame para ser mostrado na janela
		frameBGR = tempImg;

		cv::imshow(rgbWindowName, frameBGR);		// Show image inside the window
		cv::waitKey(100);							// Wait (miliseconds) for a keystroke in the window (0 = inf)
	}

	void registerCallbacks(pcl::Grabber* interface) {
		boost::function<void (const CloudType::ConstPtr&)> cloudFunc = boost::bind(&KinectScanner::cloudCallback, this, _1);
		boost::function<void (const ImageType&)> rgbFunc = boost::bind(&KinectScanner::rgbCallback, this, _1);

		interface->registerCallback(cloudFunc);
		interface->registerCallback(rgbFunc);
	}

	Scan3D testFunc3(string facesDir = "../MM_Restricted_PCA/automatic_tests/") {
		string fullCloudName = facesDir + "full_cloud.off";
		string landmarksFileName = facesDir + "base_cloud_landmarks.txt";
		string modelName = facesDir + "result_model.off";

		Scan3D fullCloud(fullCloudName, landmarksFileName);

		string cmdLine = "sh ../modelAlign.sh " + fullCloudName + " " + landmarksFileName + " " + modelName;
		system(cmdLine.c_str());

		Scan3D mesh(modelName, landmarksFileName);
		return mesh;

		// mesh.updateTexData("../resources/meanFace.obj", "texture.png");
		// mesh.applyTextureFromImage(fullCloud, "../resources/texture.png", facesDir);
		// mesh.save("mesh_result.obj");
	}

	// run :D
	void run() {
		pcl::Grabber* interface = new pcl::OpenNIGrabber();
		registerCallbacks(interface);

		interface->start();
		while (!viewer->wasStopped())
		{
			// viewer->spinOnce();						// Update 3D viewer

			if (createMesh)
			{
				cout << "\tS was pressed! Exporting point clouds..." << endl;

				// sempre salva em uma nova pasta baseada no horário corrente
				string currentDateTime = Util::currentDateTime();
				string cloudsStdPath = "../automatic_tests/" + currentDateTime + "/";
				mkdir(cloudsStdPath.c_str(), 0777);
				string baseCloudName = cloudsStdPath + "base_cloud.off";
				string fullCloudName = cloudsStdPath + "full_cloud.off";
				string landmarksFileName = Util::changeSuffix(baseCloudName, "_landmarks.txt");

				// salvar nuvens para chamar programa externo
				baseCloudScan->save(baseCloudName, landmarksFileName);

				// remover pontos com poucos vizinhos (provavelmente são partes pouco escaneadas)
				fullCloudScan->removeOutliers();
				fullCloudScan->save(fullCloudName, landmarksFileName);

				cout << "\t-> Finished clouds saving!" << endl;

				string modelName = cloudsStdPath + "result_model.off";

				// chamar o programa que cria o modelo, passando a nuvem cheia salva como parâmetro
				string cmdLine = "sh ../modelAlign.sh " + fullCloudName + " " + landmarksFileName + " " + modelName;
				system(cmdLine.c_str());

				// atualiza mesh (sem textura)
				faceMesh = new Scan3D(modelName, landmarksFileName);

				// carrega resultado para o programa, mas ainda está sem texturas
				cout << "\tApplying texture to the mesh..." << endl;

				faceMesh->updateTexData("../resources/meanFace.obj", "texture.png");
				faceMesh->applyTextureFromImage(*fullCloudScan, "../resources/texture.png", cloudsStdPath);

				cout << "\t...DONE!" << endl;

				// salvar modelo agora com textura, em formato obj
				string texModelName = Util::changeSuffix(modelName, "_with_texture.obj");
				faceMesh->save(texModelName);

				cout << "\t-> Finished 3D Face Reconstruction Succefully!" << endl;

				// atualiza o modelo no programa
				displayMesh = true;
				texFaceMesh = new Scan3D(texModelName, landmarksFileName);

				createMesh = false;
			}

			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}
		interface->stop();
	}
};

int main(int argc, char const *argv[])
{
	KinectScanner ks;
	ks.run();
	// ks.testFunc3();
	return 0;
}