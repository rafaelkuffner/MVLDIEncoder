#pragma once

#include "Layer.h"
#include "pcl\common\common.h"
#include <vector>
#include <map>

#define EFB_MODE_DIRECTION 1
#define EFB_MODE_POSITION 2

using namespace std;
using namespace pcl;
class LDI
{
public:
	vector<Layer*> data;
	float redundancyTreshold;
	LDI(float dt);
	~LDI();
	void createEmptyLayersFromTree(Matrix3f *intrinsics,map<string, float*> id_calibration, NumbersTree t);
	NumbersTree encodeOptimal(PointCloud<PointXYZRGBNormal>::Ptr cloud, std::queue<string> ids, map<string, float*> id_calibration, Matrix3f *intrinsics, NumbersTree nt, int level, int &maxlevel);
	void encodeGreedyOptimal(PointCloud<PointXYZRGBNormal>::Ptr cloud, map<string, float*> id_calibration, Matrix3f *intrinsics);
	void encodeCreatedLayers(PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, int layerLimit);
	void encodeOriginal(PointCloud<PointXYZRGBNormal>::Ptr cloud, map<string, float*> id_calibration,Matrix3f *intrinsics,int recursion, int layerLimit);
	void encodeGenerate(PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics);
	void encodeFindBest(PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, std::map<string, float*> id_calibration,int mode);
	void encodeLDI(PointCloud<PointXYZRGBNormal>::Ptr cloud,float* mainViewpoint, Matrix3f *intrinsics, float distance);
	void writeToDisk(string path);
	void writeToDiskImg(string path);
	void writeToPly(string path);
	void getFrameARGBData(int layer, uint8_t* &data,int size);
	void getFrameDepthARGBData(int layer, uint8_t* &data,int size);
	void getFrameDepthData(int layer, short* &data, int size);
	void getFrameNormalARGBData(int layer, uint8_t* &data, int size);

private:
	void encodeOrignalPass(PointCloud<PointXYZRGBNormal>::Ptr cloud,
		PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints,
		PointCloud<PointXYZRGBNormal>::Ptr outOfLayers,
		pcl::PointIndices::Ptr ToRetry,
		Matrix3f *intrinsics,
		int layerLimit);

	void encodeGeneratePass(PointCloud<PointXYZRGBNormal>::Ptr cloud,
		PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints,
		pcl::PointIndices::Ptr ToRetry, Layer *l);

	void encodeLDIPass(PointCloud<PointXYZRGBNormal>::Ptr cloud,
		PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints,
		PointCloud<PointXYZRGBNormal>::Ptr outOfSight,
		pcl::PointIndices::Ptr ToRetry,
		Matrix3f *intrinsics, Layer *l);
};

//Construtor 
//Save to file
//Load from file
//Render