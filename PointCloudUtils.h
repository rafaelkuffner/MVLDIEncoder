#pragma once

#include <string>
#include <vector>
#include <sstream>

#undef max
#undef min

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <Windows.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

namespace PointCloudUtils{
	
	struct stream {
		string path;
		int framesahead;
	};


	vector<string> split(const string &s, char delim, vector<string> &elems);
	vector<string> split(const string &s, char delim);
	void ApplyCalibrationToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string calibrationName, string calibrationFile, bool matrixCalib);
	void loadPointCloudNoFormat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string pointCloudPath);
	void loadPointCloudDepthColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string colorpath, uint8_t *colorData, string depthpath, short *depthData, int width, int height, Matrix3f *intrinsics,int depthLimit);
	bool readDepthFile(string path, short *depthData, int width, int height);
	void readBitmap(string path, uint8_t *colorData, int width, int height);

	PointCloud<PointXYZRGB>::Ptr combinePointClouds(map<string, stream> filenameByCalibrationpath,string calibrationFile, string cloudID, bool matrixCalib);
	PointCloud<PointXYZRGB>::Ptr combinePointClouds(map<string, stream> filenameByCalibrationpath, string calibrationFile, int index, string depthprefix, short *depthData, string colorprefix,
		uint8_t *colorData, bool matrixCalib, int width, int height, Matrix3f *intrinsics, int depthLimit, bool filter,map<string,int> &id_lastCloud);

	float* loadCalibration(string calibrationFile, string calibrationName,bool matrix);
	PointCloud<PointXYZRGBNormal>::Ptr calculateNormals(PointCloud<PointXYZRGB>::Ptr cloud);
	void loadPly(PointCloud<PointXYZRGB>::Ptr cloud, std::string path);
	double computeCloudResolution(const pcl::PointCloud<PointXYZRGB>::ConstPtr &cloud);
	double computeCloudResolution(const pcl::PointCloud<PointXYZRGBNormal>::ConstPtr &cloud);
	HRESULT SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR path);
	string ExePath();

	std::vector <pcl::PointIndices> segmentCloudByNormals(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, float resolution, int minClusterSize,int angle);
	std::vector <pcl::PointIndices> segmentCloudByDoNormals(pcl::PointCloud<PointXYZRGB>::Ptr cloud);
	std::vector <pcl::PointIndices> segmentRegionGrowing(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, float resolution, int minClusterSize, float degreesThreshold);
	Eigen::Transform<float, 3, Eigen::Affine> *obtainViewpointClusterCentroid(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, int minCluster, double res, int width, int height);
	Eigen::Transform<float, 3, Eigen::Affine> *obtainViewpointCentroid(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, double res);
	Eigen::Transform<float, 3, Eigen::Affine> *obtainViewpointCalibrationDirection(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, map<string, float*> id_calibration, int minCluster, string &viewRes, double res, int &lastidx);
	Eigen::Transform<float, 3, Eigen::Affine> *obtainViewpointCalibrationPosition(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, map<string, float*> id_calibration, int minCluster, string &viewRes, double res, int &lastidx);
	float calculateRadius(PointCloud<PointXYZRGBNormal>::Ptr cloud, Vector3f &centroid, Vector3f &normal);
	void centroidDirVector(pcl::PointCloud<PointXYZRGBNormal>::Ptr bigCluster, double resolution, Matrix3f *intrinsics, Vector3f &center, Vector3f &normal, Vector3f &dir, Vector3f &position, bool cluster);
	void optimizeDistance(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Eigen::Transform<float, 3, Affine> *mat, Matrix3f *intrinsics, Vector3f &center, Vector3f &normal, Vector3f &dir, Vector3f &position, int width, int height);
	void OBBDirVector(pcl::PointCloud<PointXYZRGBNormal>::Ptr bigCluster, double resolution, Matrix3f *intrinsics, Vector3f &dir, Vector3f &position, Vector3f &centroid, bool cluster);
	Matrix3f makeRotationDir(const Vector3f& direction, const Vector3f& up);
	void writeClustersDisk(std::vector <pcl::PointIndices> clusters, pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, bool colorize);
};