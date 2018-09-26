#include "LDI.h"
#include "PointCloudUtils.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "pcl\filters\extract_indices.h"
#include "NvEncoder.h"
#include "FFencoder.h"
using namespace pcl;
using namespace PointCloudUtils;


string outputDir;
string calibFile;
string mainViewpoint;
map<string, string> id_path;
map<string, float*> id_calibration;
Matrix3f *intrinsics;
bool matrixCalib;
float thresholdSeg;
string method;
int cloudNumber = 0;
int cloudCount = 1;

void loadConfig(){
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("config.ini", pt);
	outputDir = pt.get<std::string>("outputDir"); 
	mainViewpoint = pt.get<std::string>("mainLDIViewpoint");
	calibFile = pt.get<std::string>("calibFile");
	method = pt.get<std::string>("method");
	thresholdSeg = pt.get<float>("thresholdSeg");
	matrixCalib = pt.get<bool>("MATRIXCALIB");
	int num = pt.get<int>("NUMBER");
	cloudNumber = pt.get<int>("cloudNumber");
	cloudCount = pt.get<int>("cloudCount");
	for (int i = 0; i < num; i++){
		stringstream s;
		s << i;
		string calibname = pt.get<string>(s.str());
		s << "P";
		string datapath = pt.get<string>(s.str());
		id_path[calibname] = datapath;
	}
}


void loadViewPointsInfo(){
	map<string, string>::iterator iter;
	for (iter = id_path.begin(); iter != id_path.end(); iter++) {
		id_calibration[iter->first] = PointCloudUtils::loadCalibration(calibFile, iter->first,matrixCalib);
	}
	*intrinsics << 351.00146191913234, 0, 255.5, 
					0, 351.00146191913234, 211.5, 
					0, 0, 1;
}


LDI *encodeGenerate(PointCloud<PointXYZRGB>::Ptr cloud, bool writeResults) {
	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals;
	cloud_with_normals = calculateNormals(cloud);

	std::vector<int> aux_indices;
	removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, aux_indices);
	LDI *frame = new LDI(thresholdSeg);
	frame->encodeGenerate(cloud_with_normals, intrinsics);
	if(writeResults){
		frame->writeToPly("Encoded.ply");
		frame->writeToDiskImg("Results");
	}
	return frame;

}

LDI *encodeToOriginal(PointCloud<PointXYZRGB>::Ptr cloud, bool writeResults) {
	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals;
	cloud_with_normals = calculateNormals(cloud);
	std::vector<int> aux_indices;
	removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, aux_indices);
	if(writeResults){
		PLYWriter wt;
		wt.write("thiscloud.ply", *cloud_with_normals);
	}
	LDI *frame = new LDI(thresholdSeg);
	frame->encodeOriginal(cloud_with_normals, id_calibration, intrinsics,0);
	if(writeResults){
		
		frame->writeToDiskImg("Results");
		frame->writeToPly("Encoded.ply");
	}
	return frame;

}

LDI *encodeToLDI(PointCloud<PointXYZRGB>::Ptr cloud,float distance, bool writeResults) {
	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals;
	cloud_with_normals = calculateNormals(cloud);
	
	std::vector<int> aux_indices;
	removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, aux_indices);
	LDI *frame = new LDI(thresholdSeg);
	frame->encodeLDI(cloud_with_normals, id_calibration[mainViewpoint], intrinsics,distance);
	if(writeResults){
		frame->writeToDiskImg("Results");
		PLYWriter wt;
		wt.write("thiscloud.ply", *cloud_with_normals);
		frame->writeToPly("Encoded.ply");
	}
	return frame;

}

//int mainSingle(int argc, char *argv[]){
//	intrinsics = new Matrix3f();
//
//	loadConfig();
//	loadViewPointsInfo();
//	stringstream ss;
//
//	ss << "outputCloud" << cloudNumber;
//	PointCloud<PointXYZRGB>::Ptr cloud = combinePointClouds(id_path, calibFile, ss.str(),matrixCalib);
//	if (method == "generate")
//		encodeGenerate(cloud, false);
//	else if (method == "original")
//		encodeToOriginal(cloud, false);
//	else if (method == "LDI")
//		encodeToLDI(cloud, false, 1);
//
//	delete intrinsics;
//	return 0;
//}

void writeOutputConfig(string cname, string dname, string nname, int vidw, int vidh, int nl) {
	boost::property_tree::ptree pt;
	pt.put("colorStreamName", cname);
	pt.put("depthStreamName", dname);
	pt.put("normalStreamName", nname);
	pt.put("vidWidth", vidw);
	pt.put("vidHeight", vidh);
	pt.put("numLayers", nl);

	stringstream ss;
	map<string, float*>::iterator iter;
	int i = 0;
	for (iter = id_calibration.begin(); iter != id_calibration.end(); iter++) {
		Eigen::Quaternionf q = Eigen::Quaternionf(iter->second[6], iter->second[3], iter->second[4], iter->second[5]);
		Eigen::Translation<float, 3> translate(iter->second[0], iter->second[1], iter->second[2]);
		Eigen::Transform<float, 3, Affine> *transf = new Eigen::Transform<float, 3, Affine>();
		*transf = translate *q;
		ss.swap(stringstream());
		ss << (*transf)(0, 0) << ";" << (*transf)(0, 1) << ";" << (*transf)(0, 2) << ";" << (*transf)(0, 3) << ";"
			<< (*transf)(1, 0) << ";" << (*transf)(1, 1) << ";" << (*transf)(1, 2) << ";" << (*transf)(1, 3) << ";"
			<< (*transf)(2, 0) << ";" << (*transf)(2, 1) << ";" << (*transf)(2, 2) << ";" << (*transf)(2, 3) << ";"
			<< (*transf)(3, 0) << ";" << (*transf)(3, 1) << ";" << (*transf)(3, 2) << ";" << (*transf)(3, 3);
		string calib = ss.str();
		ss.swap(stringstream());
		ss << i;
		pt.put(ss.str(), calib);
		i++;
		if (i < nl && i < id_calibration.size()) {
			iter = id_calibration.begin();
		}
		if (i == nl) break;
		//delete transf;
	}

	boost::property_tree::write_ini("output.ini", pt);
}

int mainVideoNVCodec(int argc, char *argv[]) {
	intrinsics = new Matrix3f();

	loadConfig();
	loadViewPointsInfo();
	stringstream ss;

	int width = 512;
	int height = 424;
	int datasize = width * height * 4;
	uint8_t* data =(uint8_t*) malloc(datasize);
	int usedLayers = 0;
	int deviceID = 0;
	CNvEncoder::InitCuda(deviceID);
	vector<CNvEncoder*> colorEncoders;
	vector<CNvEncoder*> depthEncoders;
	vector<CNvEncoder*> normalEncoders;
	for (int i = 0; i < id_path.size() * 2; i++) {
		ss.swap(std::stringstream());
		CNvEncoder *nvEncoderColor = new CNvEncoder();
		ss << i << "layerColor" << ".h264";
		std::string outPath = ss.str();
		char* pathPointer = &outPath[0];
		nvEncoderColor->InitEncoder(width, height, pathPointer);
		colorEncoders.push_back(nvEncoderColor);
		CNvEncoder *nvEncoderDepth = new CNvEncoder();
		ss.swap(std::stringstream());
		ss << i << "layerDepth" << ".h264";
		outPath = ss.str();
		pathPointer = &outPath[0];
		nvEncoderDepth->InitEncoder(width, height, pathPointer);
		depthEncoders.push_back(nvEncoderDepth);
		CNvEncoder *nvEncoderNormal = new CNvEncoder();
		ss.swap(std::stringstream());
		ss << i << "layerNormal" << ".h264";
		outPath = ss.str();
		pathPointer = &outPath[0];
		nvEncoderNormal->InitEncoder(width, height, pathPointer);
		normalEncoders.push_back(nvEncoderNormal);
	}


	int i = cloudNumber;
	while(i < cloudNumber + cloudCount){
		ss.swap(std::stringstream());
		ss << "outputCloud" << i;
		PointCloud<PointXYZRGB>::Ptr cloud(combinePointClouds(id_path, calibFile, ss.str(), matrixCalib));
		LDI *frame;
		if (method == "generate")
			frame = encodeGenerate(cloud,false);
		else if (method == "original")
			frame = encodeToOriginal(cloud, false);
		else if (method == "LDI")
			frame = encodeToLDI(cloud, false, 1);

		i++;
		int thisSize = frame->data.size();
		usedLayers = max(usedLayers, thisSize);
		int j = 0;
		for (int j = 0; j < colorEncoders.size(); j++) {
			frame->getFrameARGBData(j, data,datasize);
			colorEncoders[j]->processFrame(data);
			frame->getFrameDepthARGBData(j, data,datasize);
			depthEncoders[j]->processFrame(data);
			frame->getFrameNormalARGBData(j, data, datasize);
			normalEncoders[j]->processFrame(data);
		}
		cloud.reset();
		delete frame;
	}
	for (int j = 0; j <colorEncoders.size(); j++) {
		colorEncoders[j]->FinalizeEncoder();
		depthEncoders[j]->FinalizeEncoder();
		normalEncoders[j]->FinalizeEncoder();
	}
	writeOutputConfig("layerColor.h264","layerDepth.h264","layerNormal.h264",width,height,usedLayers);
	CNvEncoder::DeInitCuda();
	delete intrinsics;
	return 0;
}

int main(int argc, char *argv[]) {
	
	intrinsics = new Matrix3f();

	loadConfig();
	loadViewPointsInfo();
	stringstream ss;

	int width = 512;
	int height = 424;
	int datasize = width * height * 4;
	uint8_t* data = (uint8_t*)malloc(datasize);
	int usedLayers = 0;
	int deviceID = 0;
	vector<FFencoder*> colorEncoders;
	vector<FFencoder*> depthEncoders;
	vector<FFencoder*> normalEncoders;
	for (int i = 0; i < id_path.size() * 2; i++) {
		ss.swap(std::stringstream());
		ss << i << "layerColor" << ".h264";
		std::string outPath = ss.str();
		FFencoder *ffEncoderColor = new FFencoder(outPath,width, height);
		colorEncoders.push_back(ffEncoderColor);
		
	/*	ss.swap(std::stringstream());
		ss << i << "layerDepth" << ".h264";
		outPath = ss.str();
		FFencoder *ffEncoderDepth = new FFencoder(outPath, width, height);
		depthEncoders.push_back(ffEncoderDepth);
		
		ss.swap(std::stringstream());
		ss << i << "layerNormal" << ".h264";
		outPath = ss.str();
		FFencoder *ffEncoderNormal = new FFencoder(outPath, width, height);
		normalEncoders.push_back(ffEncoderNormal);*/
	}


	int i = cloudNumber;
	int k = 0;
	while (i < cloudNumber + cloudCount){
		ss.swap(std::stringstream());
		ss << "outputCloud" << i;
		PointCloud<PointXYZRGB>::Ptr cloud(combinePointClouds(id_path, calibFile, ss.str(), matrixCalib));
		LDI *frame;
		if (method == "generate")
			frame = encodeGenerate(cloud, false);
		else if (method == "original")
			frame = encodeToOriginal(cloud, false);
		else if (method == "LDI")
			frame = encodeToLDI(cloud, false, 1);

		i++;
		int thisSize = frame->data.size();
		usedLayers = max(usedLayers, thisSize);
		int j = 0;
		for (int j = 0; j < 1; j++) {
			frame->getFrameARGBData(j, data, datasize);
			colorEncoders[j]->processFrame(k,data);
			/*frame->getFrameDepthARGBData(j, data, datasize);
			depthEncoders[j]->processFrame(k,data);
			frame->getFrameNormalARGBData(j, data, datasize);
			normalEncoders[j]->processFrame(k,data);*/
		}
		k++;
		cloud.reset();
		delete frame;
	}
	for (int j = 0; j <colorEncoders.size(); j++) {
		colorEncoders[j]->finalizeEncoder();
		depthEncoders[j]->finalizeEncoder();
		normalEncoders[j]->finalizeEncoder();
	}
	writeOutputConfig("layerColor.h264", "layerDepth.h264", "layerNormal.h264", width, height, usedLayers);
	CNvEncoder::DeInitCuda();
	delete intrinsics;
	return 0;
}

int mainSingle(int argc, char *argv[]) {
	intrinsics = new Matrix3f();

	loadConfig();
	loadViewPointsInfo();
	stringstream ss;

	int width = 512;
	int height = 424;
	int datasize = width * height * 4;
	uint8_t* data = (uint8_t*)malloc(datasize);
	int usedLayers = 0;
	int deviceID = 0;

	int i = cloudNumber;
	ss.swap(std::stringstream());
	ss << "outputCloud" << i;
	PointCloud<PointXYZRGB>::Ptr cloud(combinePointClouds(id_path, calibFile, ss.str(), matrixCalib));
	//PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	//loadPly(cloud, "C:\\Users\\Blackbox\\Downloads\\Rubiks Cube\\rubrisk.ply");
	//loadPly(cloud, "C:\\Users\\rafae\\Downloads\\cubee.ply");

	LDI *frame;
	if (method == "generate")
		frame = encodeGenerate(cloud, true);
	else if (method == "original")
		frame = encodeToOriginal(cloud, true);
	else if (method == "LDI")
		frame = encodeToLDI(cloud, false, 1);
	
	delete intrinsics;
	return 0;
}
//PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
//loadPly(cloud, "C:\\Users\\rafae\\Downloads\\Rubiks Cube\\Rubiks Cube\\Rubiks Cube.ply");
//loadPly(cloud, "C:\\Users\\Blackbox\\Downloads\\Rubiks Cube\\rubik.ply");
//loadPly(cloud, "D:\\GoogleDrive\\VSProjects\\KinectCPP\\LDIEncoder\\miyu.ply"); 
//PointCloudUtils::segmentRegionGrowing(cloud);