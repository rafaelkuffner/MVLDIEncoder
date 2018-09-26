#pragma once
#define inline __inline

//#include "FFencoder.h"
#include "LDI.h"
#include "PointCloudUtils.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "pcl\filters\extract_indices.h"
#include "NvEncoder.h"
#include "RVLEncoder.h"

using namespace pcl;
using namespace PointCloudUtils;


string outputDir;
string calibFile;
string mainViewpoint;
map<string, stream> id_stream;
map<string, int> id_lastCloud;
map<string, float*> id_calibration;
Matrix3f *intrinsics;
bool matrixCalib;
float thresholdSeg;
string method;
int cloudNumber = 0;
int cloudCount = 1;
int maxLayers = 0;
int depthLimit = 10000;
int deviceID = 0;
string configFile = "config.ini";
string colorprefix;
string depthprefix;
bool filter;

void loadConfig(){
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(configFile, pt);
	outputDir = pt.get<std::string>("outputDir"); 
	colorprefix = pt.get<std::string>("colorprefix");
	depthprefix = pt.get<std::string>("depthprefix");
	mainViewpoint = pt.get<std::string>("mainLDIViewpoint");
	calibFile = pt.get<std::string>("calibFile");
	method = pt.get<std::string>("method");
	thresholdSeg = pt.get<float>("thresholdSeg");
	matrixCalib = pt.get<bool>("MATRIXCALIB");
	filter = pt.get<bool>("filter");
	int num = pt.get<int>("NUMBER");
	cloudNumber = pt.get<int>("cloudNumber");
	cloudCount = pt.get<int>("cloudCount");
	maxLayers = pt.get<int>("maxLayers");
	depthLimit = pt.get<int>("depthLimit");
	deviceID = pt.get<int>("deviceID");
	for (int i = 0; i < num; i++){
		stringstream n, f, s;
		n << i;
		string calibname = pt.get<string>(n.str());
		n << "P";
		string datapath = pt.get<string>(n.str());
		f << i << "frame";
		int fsync = pt.get<int>(f.str());
		stream a;
		a.framesahead = fsync;
		a.path = datapath;
		id_stream[calibname] = a;
		id_lastCloud[calibname] = 0;
	}
}


void loadViewPointsInfo(){
	map<string, stream>::iterator iter;
	for (iter = id_stream.begin(); iter != id_stream.end(); iter++) {
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
		frame->writeToDiskImg(outputDir);
	}
	return frame;

}

LDI *encodeToOriginal(PointCloud<PointXYZRGB>::Ptr cloud, bool writeResults, int layerLimit = 0) {
	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals;
	cloud_with_normals = calculateNormals(cloud);
	std::vector<int> aux_indices;
	removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, aux_indices);
	if(writeResults){
		PLYWriter wt;
		wt.write("thiscloud.ply", *cloud_with_normals);
	}
	LDI *frame = new LDI(thresholdSeg);
	frame->encodeOriginal(cloud_with_normals, id_calibration, intrinsics,0,layerLimit);
	if(writeResults){
		
		frame->writeToDiskImg(outputDir);
		frame->writeToPly("Encoded.ply");
	}
	return frame;

}


void printTree(NumbersTree t, std::string tabs) {
	std::cout << tabs << " " << t.index << " " << t.points << "{" << std::endl;
	stringstream tt;
	tt << tabs << "\t";

	for (int i = 0; i < t.childNodes.size(); i++) {
		printTree(t.childNodes[i], tt.str());
	}

	std::cout<< "}" << std::endl;
	return;
}
int getTotalPoints(NumbersTree t) {
	if (t.childNodes.size() == 0) {
		return t.points;
	}
	else {
		int res = t.points;
		for (int i = 0; i < t.childNodes.size(); i++){
			res += getTotalPoints(t.childNodes[i]);
		}
		return res;
	}
}
NumbersTree findBest(NumbersTree t, int &level) {
	
	if (t.childNodes.size() == 0) {
		return t;
	}

	int minLevel = INT_MAX;
	NumbersTree thebest;
	thebest.points = 0;
	
	for (int i = 0; i < t.childNodes.size(); i++) {
		int thislevel =level + 1;
	
		if (t.childNodes[i].points > 0) {
			NumbersTree kid = findBest(t.childNodes[i], thislevel);
			int kidPoints = getTotalPoints(kid);
			int bestPoints = getTotalPoints(thebest);
			if (thislevel < minLevel) {
				thebest = kid;
				minLevel = thislevel;
			}
			else if (thislevel == minLevel) {
				if (kidPoints > bestPoints) {
					thebest = kid;
				}
				else if (kidPoints == bestPoints && kid.points > thebest.points) {
					thebest = kid;
				}
			}
		}
	}
	level = minLevel;
	NumbersTree res;
	res.index = t.index;
	res.points = t.points;
	res.childNodes.push_back(thebest);
	return res;
}

NumbersTree findBestDistanceMetric(NumbersTree t, int &level) {

	if (t.childNodes.size() == 0) {
		return t;
	}

	int minLevel = INT_MAX;
	NumbersTree thebest;
	thebest.points = 0;

	for (int i = 0; i < t.childNodes.size(); i++) {
		int thislevel = level + 1;

		if (t.childNodes[i].points > 0) {
			NumbersTree kid = findBestDistanceMetric(t.childNodes[i], thislevel);
			int kidPoints = getTotalPoints(kid);
			int bestPoints = getTotalPoints(thebest);
			if (thislevel < minLevel) {
				thebest = kid;
				minLevel = thislevel;
			}
			else if (thislevel == minLevel) {
				if(kid.medianDepth > 0 && kid.medianDepth < thebest.medianDepth) {
					thebest = kid;
				}
			}
		}
	}
	level = minLevel;
	NumbersTree res;
	res.index = t.index;
	res.points = t.points;
	res.childNodes.push_back(thebest);
	return res;
}

LDI *encodeOptimal(PointCloud<PointXYZRGB>::Ptr cloud, bool writeResults,int maxLayers) {
	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals;
	PointCloud<PointXYZRGBNormal>::Ptr cloud_original(new PointCloud<PointXYZRGBNormal>);

	cloud_with_normals = calculateNormals(cloud);
	std::vector<int> aux_indices;
	removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, aux_indices);
	if (writeResults) {
		PLYWriter wt;
		wt.write("thiscloud.ply", *cloud_with_normals);
	}	
	copyPointCloud(*cloud_with_normals, *cloud_original);
	LDI *frame = new LDI(thresholdSeg);
	NumbersTree t;
	t.points = cloud_with_normals->size();
	t.index = "";	
	
	std::queue<string> ids;
	for (map<string, float*>::iterator it = id_calibration.begin(); it != id_calibration.end(); ++it) {
		ids.push(it->first);
	}
	t = frame->encodeOptimal(cloud_with_normals,ids, id_calibration, intrinsics,t,-1,maxLayers);
	string s;
	std::cout << "best tree number " << endl;
	int level = 0;
	NumbersTree best = findBest(t, level);
	printTree(best,s);
	
	std::cout << "creating layers" << endl;
	frame->createEmptyLayersFromTree(intrinsics,id_calibration,best);
	std::cout << "encode from layers" << endl;
	frame->encodeCreatedLayers(cloud_original, intrinsics, 0);
	
	std::cout << "--------------------------------" << endl;
	std::cout << "best tree distance " << endl;
	level = 0;
	NumbersTree bestd = findBestDistanceMetric(t, level);
	printTree(bestd, s);
	std::cout << "--------------------------------"<<endl;

	printTree(t, "");
	
	if (writeResults) {
		frame->writeToDiskImg(outputDir);
		frame->writeToPly("Encoded.ply");
	}
	return frame;

}


LDI *encodeGreedyOptimal(PointCloud<PointXYZRGB>::Ptr cloud, bool writeResults) {
	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals;
	PointCloud<PointXYZRGBNormal>::Ptr cloud_original(new PointCloud<PointXYZRGBNormal>);

	cloud_with_normals = calculateNormals(cloud);
	std::vector<int> aux_indices;
	removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, aux_indices);
	if (writeResults) {
		PLYWriter wt;
		wt.write("thiscloud.ply", *cloud_with_normals);
	}
	copyPointCloud(*cloud_with_normals, *cloud_original);
	LDI *frame = new LDI(thresholdSeg);

	frame->encodeGreedyOptimal(cloud_with_normals,  id_calibration, intrinsics);
	
	if (writeResults) {
		frame->writeToDiskImg(outputDir);
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
		frame->writeToDiskImg(outputDir);
		PLYWriter wt;
		wt.write("thiscloud.ply", *cloud_with_normals);
		frame->writeToPly("Encoded.ply");
	}
	return frame;

}

LDI *encodeFindBest(PointCloud<PointXYZRGB>::Ptr cloud, bool writeResults,int mode) {
	PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals;
	cloud_with_normals = calculateNormals(cloud);

	std::vector<int> aux_indices;
	removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, aux_indices);
	LDI *frame = new LDI(thresholdSeg);
	frame->encodeFindBest(cloud_with_normals, intrinsics, id_calibration, mode);
	if (writeResults) {
		frame->writeToDiskImg(outputDir);
		PLYWriter wt;
		wt.write("thiscloud.ply", *cloud_with_normals);
		frame->writeToPly("Encoded.ply");
	}
	return frame;

}

void writeOutputConfig(string cname, string dname, string nname, int vidw, int vidh, int nl) {
	boost::property_tree::ptree pt;
	pt.put("colorStreamName", cname);
	pt.put("depthStreamName", dname);
	pt.put("normalStreamName", nname);
	pt.put("vidWidth", vidw);
	pt.put("vidHeight", vidh);
	pt.put("numLayers", nl);

	stringstream ss;
	map<string, float*>::iterator iter = id_calibration.begin();
	int i = 0;
	bool reset = false;
	while (i < nl) {
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
		iter++;
		if (i < nl && i >= id_calibration.size() && !reset ) {
			iter = id_calibration.begin();
			reset = true;
		}
		//delete transf;
	}

	boost::property_tree::write_ini("output.ini", pt);
}


//NVENC
int mainVideoComplete(int argc, char *argv[]) {
	intrinsics = new Matrix3f();

	loadConfig();
	loadViewPointsInfo();
	stringstream ss;

	int width = 512;
	int height = 424;
	int datasize = width * height * 4;
	int depthSize = width*height * sizeof(short);
	uint8_t* data =(uint8_t*) malloc(datasize);
	short* depthData = (short*)malloc(depthSize);
	int usedLayers = 0;
	CNvEncoder::InitCuda(deviceID);
	vector<CNvEncoder*> colorEncoders;
	vector<CNvEncoder*> normalEncoders;
	vector<RVLEncoder*> depthEncoders;

	for (int i = 0; i < id_stream.size() *3; i++) {
		ss.swap(std::stringstream());
		CNvEncoder *nvEncoderColor = new CNvEncoder();
		ss <<outputDir <<"\\"<< i << "layerColor" << ".h264";
		std::string outPath = ss.str();
		char* pathPointer = &outPath[0];
		nvEncoderColor->InitEncoder(width, height, pathPointer);
		colorEncoders.push_back(nvEncoderColor);
		
		CNvEncoder *nvEncoderNormal = new CNvEncoder();
		ss.swap(std::stringstream());
		ss <<outputDir<< "\\" << i << "layerNormal" << ".h264";
		outPath = ss.str();
		pathPointer = &outPath[0];
		nvEncoderNormal->InitEncoder(width, height, pathPointer);
		normalEncoders.push_back(nvEncoderNormal);

		RVLEncoder *rvlEncoderDepth = new RVLEncoder();
		ss.swap(std::stringstream());
		ss << outputDir << "\\" << i << "layerDepth" << ".rvl";
		outPath = ss.str();
		pathPointer = &outPath[0];
		rvlEncoderDepth->InitEncoder(width, height, pathPointer);
		depthEncoders.push_back(rvlEncoderDepth);
	}


	int i = cloudNumber;
	while(i < cloudNumber + cloudCount){
		ss.swap(std::stringstream());
		ss << "outputCloud" << i;
		std::cout << ss.str() << endl;
		PointCloud<PointXYZRGB>::Ptr cloud(combinePointClouds(id_stream, calibFile, ss.str(), matrixCalib));
		LDI *frame = NULL;
		if (method == "generate")
			frame = encodeGenerate(cloud, false);
		else if (method == "original")
			frame = encodeToOriginal(cloud, false);
		else if (method == "LDI")
			frame = encodeToLDI(cloud, 1,false);
		else if (method == "optimal")
			frame = encodeOptimal(cloud, false, maxLayers);
		else if (method == "optimal_greedy")
			frame = encodeGreedyOptimal(cloud, false);
		else if (method == "best_dir")
			frame = encodeFindBest(cloud, false, EFB_MODE_DIRECTION);
		else if (method == "best_pos")
			frame = encodeFindBest(cloud, false, EFB_MODE_POSITION);

		i++;
		int thisSize = frame->data.size();
		usedLayers = max(usedLayers, thisSize);
		int j = 0;
		for (int j = 0; j < colorEncoders.size(); j++) {
			frame->getFrameARGBData(j, data,datasize);
			colorEncoders[j]->processFrame(data);
			frame->getFrameNormalARGBData(j, data, datasize);
			normalEncoders[j]->processFrame(data);
			frame->getFrameDepthData(j, depthData, depthSize);
			depthEncoders[j]->CompressRVL(depthData, width*height);
		}
		cloud.reset();
		delete frame;
	}
	std::cout <<"Total layers " << usedLayers << endl;
	for (int j = 0; j <colorEncoders.size(); j++) {
		colorEncoders[j]->FinalizeEncoder();
		normalEncoders[j]->FinalizeEncoder();
	}
	writeOutputConfig("layerColor.h264","layerDepth.rvl","layerNormal.h264",width,height,usedLayers);
	CNvEncoder::DeInitCuda();
	delete intrinsics;
	return 0;
}


int mainVideoDepth(int argc, char *argv[]) {
	intrinsics = new Matrix3f();

	loadConfig();
	loadViewPointsInfo();
	stringstream ss;

	int width = 512;
	int height = 424;
	int datasize = width * height * 4;
	int depthSize = width*height * sizeof(short);
	uint8_t* data = (uint8_t*)malloc(datasize);
	short* depthData = (short*)malloc(depthSize);
	int usedLayers = 0;
	CNvEncoder::InitCuda(deviceID);
	vector<CNvEncoder*> colorEncoders;
	vector<CNvEncoder*> normalEncoders;
	vector<RVLEncoder*> depthEncoders;

	for (int i = 0; i < id_stream.size() * 3; i++) {
		ss.swap(std::stringstream());
		CNvEncoder *nvEncoderColor = new CNvEncoder();
		ss << outputDir << "\\" << i << "layerColor" << ".h264";
		std::string outPath = ss.str();
		char* pathPointer = &outPath[0];
		nvEncoderColor->InitEncoder(width, height, pathPointer);
		colorEncoders.push_back(nvEncoderColor);

		CNvEncoder *nvEncoderNormal = new CNvEncoder();
		ss.swap(std::stringstream());
		ss << outputDir << "\\" << i << "layerNormal" << ".h264";
		outPath = ss.str();
		pathPointer = &outPath[0];
		nvEncoderNormal->InitEncoder(width, height, pathPointer);
		normalEncoders.push_back(nvEncoderNormal);

		RVLEncoder *rvlEncoderDepth = new RVLEncoder();
		ss.swap(std::stringstream());
		ss << outputDir << "\\" << i << "layerDepth" << ".rvl";
		outPath = ss.str();
		pathPointer = &outPath[0];
		rvlEncoderDepth->InitEncoder(width, height, pathPointer);
		depthEncoders.push_back(rvlEncoderDepth);
	}


	int i = cloudNumber;
	while (i < cloudNumber + cloudCount) {
	

		PointCloud<PointXYZRGB>::Ptr cloud(combinePointClouds(id_stream, calibFile, i,depthprefix,depthData, colorprefix, data, matrixCalib,width,height,intrinsics, depthLimit,filter,id_lastCloud));

		LDI *frame = NULL;

		if (method == "generate")
			frame = encodeGenerate(cloud, false);
		else if (method == "original")
			frame = encodeToOriginal(cloud, false);
		else if (method == "LDI")
			frame = encodeToLDI(cloud, 1, false);
		else if (method == "optimal")
			frame = encodeOptimal(cloud, false, maxLayers);
		else if (method == "optimal_greedy")
			frame = encodeGreedyOptimal(cloud, false);
		else if (method == "best_dir")
			frame = encodeFindBest(cloud, false, EFB_MODE_DIRECTION);
		else if (method == "best_pos")
			frame = encodeFindBest(cloud, false, EFB_MODE_POSITION);

		i++;
		int thisSize = frame->data.size();
		usedLayers = max(usedLayers, thisSize);
		int j = 0;
		for (int j = 0; j < colorEncoders.size(); j++) {
			frame->getFrameARGBData(j, data, datasize);
			colorEncoders[j]->processFrame(data);
			frame->getFrameNormalARGBData(j, data, datasize);
			normalEncoders[j]->processFrame(data);
			frame->getFrameDepthData(j, depthData, depthSize);
			depthEncoders[j]->CompressRVL(depthData, width*height);
		}
		cloud.reset();
		delete frame;
	}
	std::cout << "Total layers " << usedLayers << endl;
	for (int j = 0; j <colorEncoders.size(); j++) {
		colorEncoders[j]->FinalizeEncoder();
		normalEncoders[j]->FinalizeEncoder();
	}
	writeOutputConfig("layerColor.h264", "layerDepth.rvl", "layerNormal.h264", width, height, usedLayers);
	CNvEncoder::DeInitCuda();
	delete intrinsics;
	return 0;
}

int mainOptimal(int argc, char* argv[]) {
	intrinsics = new Matrix3f();

	loadConfig();
	loadViewPointsInfo();
	stringstream ss;

	int width = 512;
	int height = 424;
	int datasize = width * height * 4;
	uint8_t* data = (uint8_t*)malloc(datasize);
	int usedLayers = 0;

	int i = cloudNumber;
	ss.swap(std::stringstream());
	ss << "outputCloud" << i;
	//PointCloud<PointXYZRGB>::Ptr cloud(combinePointClouds(id_path, calibFile, ss.str(), matrixCalib));
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	loadPly(cloud, "C:\\Users\\rafae\\Documents\\OpenGL3Repository\\source\\PCLShader\\resources\\models\\entrada.ply");
	//loadPly(cloud, "C:\\Users\\rafae\\Downloads\\cubee.ply");

	LDI *frame;
	if (method == "generate")
		frame = encodeGenerate(cloud, true);
	else if (method == "original")
		frame = encodeToOriginal(cloud, true);
	else if (method == "LDI")
		frame = encodeToLDI(cloud, false, 1);
	else if (method == "optimal")
		frame = encodeOptimal(cloud, true, maxLayers);
	else if (method == "optimal_greedy")
		frame = encodeGreedyOptimal(cloud, true);
	else if (method == "best_dir")
		frame = encodeFindBest(cloud, true,EFB_MODE_DIRECTION);
	else if (method == "best_pos")
		frame = encodeFindBest(cloud, true, EFB_MODE_POSITION);
	delete intrinsics;
	return 0;
}

//NVENC
int mainDepthColor(int argc, char *argv[]) {
	intrinsics = new Matrix3f();

	loadConfig();
	loadViewPointsInfo();
	stringstream ss;

	int width = 512;
	int height = 424;
	int datasize = width * height * 4;
	int depthSize = width*height * sizeof(short);
	uint8_t* data = (uint8_t*)malloc(datasize);
	short* depthData = (short*)malloc(depthSize);
	memset(depthData, 0, width*height * sizeof(short));
	int usedLayers = 0;
	CNvEncoder::InitCuda(deviceID);
	CNvEncoder* colorEncoder;
	RVLEncoder* depthEncoder;

	
	ss.swap(std::stringstream());
	colorEncoder = new CNvEncoder();
	ss << mainViewpoint << "layerColor" << ".h264";
	std::string outPath = ss.str();
	char* pathPointer = &outPath[0];
	colorEncoder->InitEncoder(width, height, pathPointer);

	depthEncoder = new RVLEncoder();
	ss.swap(std::stringstream());
	ss << mainViewpoint << "layerDepth" << ".rvl";
	outPath = ss.str();
	pathPointer = &outPath[0];
	depthEncoder->InitEncoder(width, height, pathPointer);
	

	int i = cloudNumber;
	while (i < cloudNumber + cloudCount) {
		int s = floor(i / 30.0);
		int f = i % 30;

		ss.swap(std::stringstream());
		ss << id_stream[mainViewpoint].path <<"\\"<<depthprefix <<"\\depthData" << s << "," << f;
		std::cout << ss.str() << endl;
		readDepthFile(ss.str(), depthData, width, height);

		ss.swap(std::stringstream());
		ss << id_stream[mainViewpoint].path << "\\"<<colorprefix <<"\\color" << s << "," << f <<".bmp";
		std::cout << ss.str() << endl;
		readBitmap(ss.str(), data, width, height);

		i++;

		colorEncoder->processFrame(data);
		depthEncoder->CompressRVL(depthData, width*height);
		
	}
	usedLayers++;
	std::cout << "Total layers " << usedLayers << endl;
	writeOutputConfig("layerColor.h264", "layerDepth.rvl", "layerNormal.h264", width, height, usedLayers);

	colorEncoder->FinalizeEncoder();
	
	CNvEncoder::DeInitCuda();
	delete intrinsics;
	return 0;
}

int main(int argc, char *argv[]) {
	std::cout << "test" << endl;
	string s(argv[1]);
	std::cout << s << endl;
	configFile = argv[2];
	std::cout << configFile << endl;
	if (s == "mvldi")
		return mainVideoDepth(argc, argv);
	else if (s == "mvd")
		return mainDepthColor(argc, argv);
}