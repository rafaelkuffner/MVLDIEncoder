#pragma once

#include "DepthPixel.h"
#include "PointCloudUtils.h"
#include <vector>
#include <Eigen\Dense>
#include<Eigen/StdVector>

using namespace std;
using namespace Eigen;

struct NumbersTree {
	int points;
	float medianDepth;
	string index;
	std::vector<NumbersTree> childNodes;
};

class Layer{
	Matrix3f *intrinsicsMatrix;
	Transform<float, 3, Affine> *extrinsics;
	DepthPixel *pixs;
	int width;
	int height;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
	Layer();
	Layer(Matrix3f *im, Transform<float, 3, Affine> *e, int w, int h);
	~Layer();

	DepthPixel getPixel(int u, int v);
	void addPixel(DepthPixel pix);
	void combineDepths(DepthPixel pix);
	bool pixelInside(DepthPixel pix);
	bool isRepeatedImageSpace(DepthPixel pix);
	bool isRepeatedImageSpaceThresh(DepthPixel pix, float threshold);
	bool isRepeatedWorldSpace(DepthPixel dp, PointXYZRGBNormal &pt, float redundancyTreshold);
	DepthPixel projectPoint(const PointXYZRGBNormal &point);
	PointXYZRGBNormal deprojectPoint(DepthPixel dp);
	PointXYZRGBNormal removePoint(DepthPixel dp); 
	void writeImg(string path,int id);
	void write(string path,int id);
	void deprojectAll(PointCloud<PointXYZRGBNormal>::Ptr cloud);
	void getFrameARGBData(uint8_t* &data);
	void getFrameDepthARGBData(uint8_t* &data);
	void getFrameDepthData(short* &data);
	void getFrameNormalARGBData(uint8_t* &data);
	float medianDepth();
	int getCount();
};

//construtores a partir de valores
//Construtor a partir de ficheiro 
//Desprojetar ponto
//Projetar depthPixel