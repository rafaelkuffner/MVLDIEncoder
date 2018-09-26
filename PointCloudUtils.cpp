#include "PointCloudUtils.h"



vector<string> PointCloudUtils::split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

vector<string> PointCloudUtils::split(const string &s, char delim) {
	vector<string> elems;
	split(s, delim, elems);
	return elems;
}

float* PointCloudUtils::loadCalibration(string calibrationFile, string calibrationName,bool matrix){
	// Get calibration data
	std::ifstream f;
	f.open(calibrationFile, fstream::in);
	std::string line;
	string delimiter = ";";
	float *translationRotation = new float[7]; // posx, posy, posz, rotx, roty, rotz
	int i = 0;

	while (std::getline(f, line))
	{
		std::vector<string> tokens = PointCloudUtils::split(line, ';');
		if (tokens[0] == calibrationName){
			if(!matrix){
				translationRotation[0] = stof(tokens[1]);
				translationRotation[1] = stof(tokens[2]);
				translationRotation[2] = stof(tokens[3]);
				translationRotation[3] = stof(tokens[4]);
				translationRotation[4] = stof(tokens[5]);
				translationRotation[5] = stof(tokens[6]);
				translationRotation[6] = stof(tokens[7]);
			}
			else {
				Matrix3f mat;
				mat(0,0) = stof(tokens[1]);
				mat(0,1) = stof(tokens[2]);
				mat(0,2) = stof(tokens[3]);
				mat(1,0) = stof(tokens[4]);
				mat(1,1) = stof(tokens[5]);
				mat(1,2) = stof(tokens[6]);
				mat(2,0) = stof(tokens[7]);
				mat(2,1) = stof(tokens[8]);
				mat(2,2) = stof(tokens[9]);
				Quaternionf q(mat);
				translationRotation[0] = 0;
				translationRotation[1] = 0;
				translationRotation[2] = 0;
				translationRotation[3] = 0;
				translationRotation[4] =0;
				translationRotation[5] = 0;
				translationRotation[6] = 1;
			}
			break;
		}
	}
	return translationRotation;
}

void PointCloudUtils::ApplyCalibrationToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string calibrationName, string calibrationFile,bool matrixCalib){

	float* translationRotation = loadCalibration(calibrationFile, calibrationName,matrixCalib);

	//cout << "translation x = " << translationRotation[0] << " y = " << translationRotation[1] << " z = " << translationRotation[2] << endl;
	//cout << "rotation x = " << translationRotation[3] << " y = " << translationRotation[4] << " z = " << translationRotation[5] << " w =" << translationRotation[6] << endl;

	Eigen::Quaterniond q = Eigen::Quaterniond(translationRotation[6], translationRotation[3], translationRotation[4], translationRotation[5]);
	Eigen::Vector3d translate(translationRotation[0], translationRotation[1], translationRotation[2]);

	pcl::transformPointCloud(*cloud, *cloud, translate, q);

	delete translationRotation;
	return;
}

void PointCloudUtils::loadPointCloudNoFormat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string pointCloudPath){
	std::ifstream f;
	f.open(pointCloudPath, fstream::in);
	std::string line;
	string delimiter1 = " ";

	string pos[6];
	while (std::getline(f, line))
	{

		size_t p;
		int i = 0;
		string substring = "";
		while ((p = line.find(delimiter1)) != std::string::npos) {
			substring = pos[i] = line.substr(0, p);
			line.erase(0, p + delimiter1.length());
			i++;
		}
		pos[5] = line;

		pcl::PointXYZRGB point;
		point.x = -stof(pos[0]);
		point.y = stof(pos[1]);
		point.z = stof(pos[2]);
		point.r = (uint8_t)stoi(pos[5]);
		point.g = (uint8_t)stoi(pos[4]);
		point.b = (uint8_t)stoi(pos[3]);

		if (point.x != 0 && point.y != 0 && point.z != 0)
			cloud->points.push_back(point);
	}
	//delete pos;
}

void PointCloudUtils::loadPointCloudDepthColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string colorpath,  uint8_t *colorData, string depthpath,short *depthData,int width, int height, Matrix3f *intrinsics,int depthLimit) {
	
	if (!readDepthFile(depthpath, depthData, width, height))
		return;
	std::cout << depthpath << endl;
	readBitmap(colorpath, colorData, width, height);
	std::cout << colorpath << endl;

	int depthi = 0;
	int colori = 0;

	float SxF = (*intrinsics)(0, 0);
	float SyF = (*intrinsics)(1, 1);
	float ox = (*intrinsics)(0, 2);
	float oy = (*intrinsics)(1, 2);

	for (int m = height - 1; m >= 0; m--)
	{
		for (int n = 0; n < width; n++) {
			PointXYZRGB ret;
			short d = depthData[depthi++];
			if (d> depthLimit) d = 0;
			ret.z = d / 1000.0;
			ret.x = -ret.z * (n - ox) / SxF;
			ret.y = ret.z  * (m - oy) / SyF;

			ret.b =	colorData[colori++];
			ret.g = colorData[colori++];
			ret.r = colorData[colori++];
			colori++; // a

			if (ret.x != 0 && ret.y != 0 && ret.z != 0)
				cloud->points.push_back(ret);
		}
		
	}
}


bool PointCloudUtils::readDepthFile(string path, short *depthData, int width, int height) {
	std::wstring stempb = std::wstring(path.begin(), path.end());
	LPCWSTR swb = stempb.c_str();
	if (INVALID_FILE_ATTRIBUTES == GetFileAttributesW(swb) && GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		return false;
	}
	else {
		HANDLE _inFile = CreateFileW(swb, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		DWORD bytesRead;
		ReadFile(_inFile, depthData, width*height * sizeof(short), &bytesRead, NULL);
		CloseHandle(_inFile);
		return true;
	}
}

void PointCloudUtils::readBitmap(string path, uint8_t *colorData, int width, int height) {
	int i;
	FILE* f = fopen(path.c_str(), "rb");
	if (f != NULL) {
		unsigned char info[54];
		fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

												   // extract image height and width from header

		int size = 4 * width * height;
		unsigned char* data = new unsigned char[size]; // allocate 3 bytes per pixel
		fread(data, sizeof(unsigned char), size, f); // read the rest of the data at once
		fclose(f);

		int j = 0;
		for (i = 0; i < size; i++)
		{
			colorData[j++] = data[i];
		}
		delete data;
	}
	return;
}


PointCloud<PointXYZRGB>::Ptr PointCloudUtils::combinePointClouds(map<string, stream> filenameByCalibrationpath, string calibrationFile, string cloudId,bool matrixCalib) {

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new PointCloud<PointXYZRGB>);
	map<string, stream>::iterator iter;
	for (iter = filenameByCalibrationpath.begin(); iter != filenameByCalibrationpath.end(); iter++) {
		stringstream cloudn;
		cloudn <<  iter->second.path << "\\" << cloudId;
		PointCloudUtils::loadPointCloudNoFormat(cloud, cloudn.str());
		PointCloudUtils::ApplyCalibrationToPointCloud(cloud, iter->first, calibrationFile, matrixCalib);
		//std::cout << "size " << cloud->size() << endl;
		*tempCloud += *cloud;
		cloud->clear();
	}
	cloud.reset();
	return tempCloud;
}

bool fexists(const char *filename)
{
	ifstream ifile(filename);
	return ifile.good();
}

PointCloud<PointXYZRGB>::Ptr PointCloudUtils::combinePointClouds(map<string, stream> filenameByCalibrationpath, string calibrationFile,
	int index, string depthprefix, short *depthData, string colorprefix, uint8_t *colorData, bool matrixCalib, int width, int height, Matrix3f *intrinsics, int depthLimit,
	bool filter, map<string,int> &id_lastCloud) {
	std::cout << "CombinePointClouds ";
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new PointCloud<PointXYZRGB>);
	map<string, stream>::iterator iter;
	for (iter = filenameByCalibrationpath.begin(); iter != filenameByCalibrationpath.end(); iter++) {
		stringstream ss;
		int i = index + iter->second.framesahead;
		int s = floor(i / 30.0);
		int f = i % 30;
	
		ss.swap(std::stringstream());
		ss << "\\" << depthprefix << "\\depthData" << s << "," << f;
		string depthID = ss.str();
		stringstream depthn;
		depthn << iter->second.path << "\\" << depthID;
		
		if (!fexists(depthn.str().c_str())&& filter ) {
			std::cout << "filter!" << i << " is " << id_lastCloud[iter->first] << endl;
			i = id_lastCloud[iter->first];
			s = floor(i / 30.0);
			f = i % 30;

			ss.swap(std::stringstream());
			ss << "\\" << depthprefix << "\\depthData" << s << "," << f;
			string depthID = ss.str();
			depthn.swap(std::stringstream());
			depthn << iter->second.path << "\\" << depthID;
		}
		else {
			id_lastCloud[iter->first] = i;
		}

		ss.swap(std::stringstream());
		ss << "\\" << colorprefix << "\\color" << s << "," << f << ".bmp";
		string colorID = ss.str();
		stringstream colorn;
		colorn << iter->second.path << "\\" << colorID;

		PointCloudUtils::loadPointCloudDepthColor(cloud, colorn.str(),colorData, depthn.str(),depthData, width, height, intrinsics, depthLimit);

		PointCloudUtils::ApplyCalibrationToPointCloud(cloud, iter->first, calibrationFile, matrixCalib);
		std::cout << "size " << cloud->size() << endl;
		*tempCloud += *cloud;
		cloud->clear();
	}
	cloud.reset();
	return tempCloud;
}

PointCloud<PointXYZRGBNormal>::Ptr PointCloudUtils::calculateNormals(PointCloud<PointXYZRGB>::Ptr cloud){

	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	PointCloud<Normal>::Ptr invertednormals(new PointCloud<Normal>);
	PointCloud<PointXYZRGBNormal>::Ptr res(new PointCloud<PointXYZRGBNormal>);
	NormalEstimation<PointXYZRGB, Normal> ne;

	search::KdTree<pcl::PointXYZRGB>::Ptr tree(new search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	//ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	ne.setViewPoint(0, 0, 0);
	//ne.setKSearch(2);
	double resolution = PointCloudUtils::computeCloudResolution(cloud);
	ne.setRadiusSearch(5 * resolution);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
	for each (Normal n in *normals)
	{
		Normal a;
		a.normal_x = -n.normal_x;
		a.normal_y = -n.normal_y;
		a.normal_z = -n.normal_z;
		invertednormals->push_back(a);
	}
	pcl::concatenateFields(*cloud, *invertednormals, *res);
	normals.reset();
	invertednormals.reset();
	tree.reset();
	return res;
}

double PointCloudUtils::computeCloudResolution(const pcl::PointCloud<PointXYZRGB>::ConstPtr &cloud)
{
	double res = 0.0;
	//int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	std::vector<float>* distances = new std::vector<float>();
	pcl::search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>);
	tree->setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree->nearestKSearch(i, 2, indices, (sqr_distances));
		if (nres == 2)
		{
			distances->push_back(sqrt(sqr_distances[1]));
		//	res += sqrt(sqr_distances[1]);
		//	++n_points;
		}
	}/*
	if (n_points != 0)
	{
		res /= n_points;
	}*/
	std::sort(distances->begin(), distances->end());
	if (distances->size() != 0){
		int idx = (int)std::floor(distances->size() / 2.0);
		res =(*distances)[idx];
	}
	delete distances;
	return res;
}

double PointCloudUtils::computeCloudResolution(const pcl::PointCloud<PointXYZRGBNormal>::ConstPtr &cloud)
{
	PointCloud<PointXYZRGB>::Ptr cloudrgb(new PointCloud<PointXYZRGB>);
	copyPointCloud(*cloud, *cloudrgb);
	return PointCloudUtils::computeCloudResolution(cloudrgb);

}
void PointCloudUtils::loadPly(PointCloud<PointXYZRGB>::Ptr cloud, std::string path){
	PolygonMesh mesh;
	string str2 = "D:";
	io::loadPLYFile(path, mesh);
	fromPCLPointCloud2(mesh.cloud, *cloud);
}

HRESULT PointCloudUtils::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR path)
{
	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

	BITMAPINFOHEADER bmpInfoHeader = { 0 };

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);  // Size of the header
	bmpInfoHeader.biBitCount = wBitsPerPixel;             // Bit count
	bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
	bmpInfoHeader.biWidth = lWidth;                    // Width in pixels
	bmpInfoHeader.biHeight = lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
	bmpInfoHeader.biPlanes = 1;                         // Default
	bmpInfoHeader.biSizeImage = dwByteCount;               // Image size in bytes

	BITMAPFILEHEADER bfh = { 0 };

	bfh.bfType = 0x4D42;                                           // 'M''B', indicates bitmap
	bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
	bfh.bfSize = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers


	// Create the file on disk to write to
	HANDLE hFile = CreateFileW(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	// Return if error opening file
	if (NULL == hFile)
	{
		return E_ACCESSDENIED;
	}

	DWORD dwBytesWritten = 0;

	// Write the bitmap file header
	if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the bitmap info header
	if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the RGB Data
	if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Close the file
	CloseHandle(hFile);
	return S_OK;
}

string PointCloudUtils::ExePath() {
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	string::size_type pos = string(buffer).find_last_of("\\/");
	return string(buffer).substr(0, pos);
}

int _angle = 0;

bool enforceCurvature(const PointXYZRGBNormal& point_a, const PointXYZRGBNormal& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
	/*if (fabs(point_a_normal.dot(point_b_normal)) < 0.5)
		return (true);
	return (false);
	if (abs(point_a.curvature - point_b.curvature < 0.05)) {
		return true;
	}*/
	float theta = acos(point_a_normal.dot(point_b_normal) / (point_a_normal.norm()*point_b_normal.norm()));
	if (theta < _angle*M_PI / 180 && squared_distance) return true;
	
	else return false;
}

float donThreshold = 0.02;

bool enforceDon(const PointNormal& point_a, const PointNormal& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
	//if (fabs(point_a_normal.dot(point_b_normal)) < 0.5)
	//return (true);
	//return (false);
	//if (abs(point_a.curvature - point_b.curvature < 0.05)) {
	//return true;
	//}
	if (abs(point_a_normal.norm() - point_b_normal.norm()) < donThreshold) return true;

	else return false;
}

std::vector <pcl::PointIndices>  PointCloudUtils::segmentCloudByNormals(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, float resolution,int minClusterSize,int angle) {

	double segradius =resolution*10;

	cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
	
	pcl::ConditionalEuclideanClustering<PointXYZRGBNormal> cec(false);
	cec.setInputCloud(cloud);
	_angle = angle;
	cec.setConditionFunction(&enforceCurvature);
	cec.setClusterTolerance(segradius);
	cec.setMinClusterSize(minClusterSize);
	cec.setMaxClusterSize(minClusterSize*100);
	cec.segment(*clusters);
	//cec.getRemovedClusters(small_clusters, large_clusters);

	//PointCloudUtils::writeClustersDisk(clusters, res, true);

	return *clusters;
}

std::vector <pcl::PointIndices>  PointCloudUtils::segmentCloudByDoNormals(pcl::PointCloud<PointXYZRGB>::Ptr cloud) {
	double resolution = PointCloudUtils::computeCloudResolution(cloud);
	///The smallest scale to use in the DoN filter.
	double scale1 = resolution*1.5;
	double scale2 = resolution * 5;
	///The minimum DoN magnitude to threshold by
	double threshold = 0.05;
	donThreshold = scale1;
	///segment scene into clusters with given distance tolerance using euclidean clustering
	double segradius = resolution * 3.0;


	// Create a search tree, use KDTreee for non-organized data.
	pcl::search::Search<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>(false));

	// Set the input pointcloud for the search tree
	tree->setInputCloud(cloud);

	// Compute normals using both small and large scales at each point
	pcl::NormalEstimation<PointXYZRGB, Normal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);

	/**
	* NOTE: setting viewpoint is very important, so that we can ensure
	* normals are all pointed in the same direction!
	*/
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

	// calculate normals with the small scale
	cout << "Calculating normals for scale..." << scale1 << endl;
	pcl::PointCloud<Normal>::Ptr normals_small_scale(new pcl::PointCloud<Normal>);
	pcl::PointCloud<Normal>::Ptr normals_large_scale(new pcl::PointCloud<Normal>);
	pcl::PointCloud<PointNormal>::Ptr res(new pcl::PointCloud<PointNormal>);

	ne.setRadiusSearch(scale1);
	ne.compute(*normals_small_scale);

	ne.setRadiusSearch(scale2);
	ne.compute(*normals_large_scale);

	copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *res);

	cout << "Calculating DoN... " << endl;
	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<PointXYZRGB, Normal, PointNormal> don;
	don.setInputCloud(cloud);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);

	if (!don.initCompute())
	{
		std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Compute DoN
	don.computeFeature(*res);

	// Save DoN features



	pcl::PLYWriter writer;
	writer.write<pcl::PointNormal>("don.ply", *res, false);

	cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);

	pcl::ConditionalEuclideanClustering<PointNormal> cec(false);
	cec.setInputCloud(res);
	cec.setConditionFunction(&enforceDon);
	cec.setClusterTolerance(segradius);
	cec.setMinClusterSize(500);
	cec.setMaxClusterSize(500000);
	cec.segment(*clusters);
	//cec.getRemovedClusters(small_clusters, large_clusters);

	//PointCloudUtils::writeClustersDisk(*clusters, res, true);
	return *clusters;
	
}

std::vector <pcl::PointIndices>  PointCloudUtils::segmentRegionGrowing(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, float resolution, int minClusterSize, float degreesThreshold) {

	///The smallest scale to use in the DoN filter.
	double scale1 = resolution * 5;
	///The minimum DoN magnitude to threshold by
	double threshold = 0.2;
	///segment scene into clusters with given distance tolerance using euclidean clustering
	double segradius = resolution * 3;

	//// Create a search tree, use KDTreee for non-organized data.
	//pcl::search::Search<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>(false));

	//// Set the input pointcloud for the search tree
	//tree->setInputCloud(cloud);

	//// Compute normals using both small and large scales at each point
	//pcl::NormalEstimation<PointXYZRGB, Normal> ne;
	//ne.setInputCloud(cloud);
	//ne.setSearchMethod(tree);

	///**
	//* NOTE: setting viewpoint is very important, so that we can ensure
	//* normals are all pointed in the same direction!
	//*/
	//ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

	//// calculate normals with the small scale
	//cout << "Calculating normals for scale..." << scale1 << endl;
	//pcl::PointCloud<Normal>::Ptr normals_small_scale(new pcl::PointCloud<Normal>);
	//pcl::PointCloud<PointXYZRGBNormal>::Ptr res(new pcl::PointCloud<PointXYZRGBNormal>);

	//ne.setRadiusSearch(scale1);
	//ne.compute(*normals_small_scale);
	//pcl::concatenateFields(*cloud, *normals_small_scale, *res);

	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	for each (PointXYZRGBNormal pt in cloud->points)
	{
		Normal n;
		n.normal_x = pt.normal_x;
		n.normal_y = pt.normal_y;
		n.normal_z = pt.normal_z;
		normals->push_back(n);
	}

	//writer.write("norm.ply", *res);
	cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "angle " << degreesThreshold << "..." << endl;

	pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree2 = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> >(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

	pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
	reg.setMinClusterSize(minClusterSize);
	reg.setMaxClusterSize(100000);
	reg.setSearchMethod(tree2);
	reg.setNumberOfNeighbours(50);
	reg.setInputCloud(cloud);

	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(degreesThreshold / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	//PointCloudUtils::writeClustersDisk(clusters, res, true);
	return clusters;

}

#define trimax(a,b,c) max(max(a,b),c)

float PointCloudUtils::calculateRadius(PointCloud<PointXYZRGBNormal>::Ptr cloud, Vector3f &centroid, Vector3f &normal){
	float m = 0;
	Vector3f maxp = Vector3f(0, 0, 0);
	PointXYZRGBNormal pmin, pmax;

	getMinMax3D(*cloud, pmin, pmax);
	Vector3f center((pmax.x + pmin.x) / 2.0, (pmax.y + pmin.y) / 2.0, (pmax.z + pmin.z) / 2.0);
	Vector3f minimum(pmin.x, pmin.y, pmin.z);
	Vector3f maximum(pmax.x, pmax.y, pmax.z);
	m =(maximum - minimum).norm()/2.0;
	
	//for (PointCloud<PointXYZRGBNormal>::iterator it = cloud->begin(); it != cloud->end(); it++){

	//	PointXYZRGBNormal p = *it;
	//	Vector3f pt(p.x, p.y, p.z);
	//	float dist = sqrt((pow(pt.x() - centroid.x(), 2)) + (pow(pt.y() - centroid.y(), 2)) + (pow(pt.z() - centroid.z(), 2)));
	//	Vector3f pc = pt - centroid;
	//	pc.normalize();
	//	float theta = acos(centroid.dot(pc) / (centroid.norm()*pc.norm()));
	//	
	//	if (dist > max ){//&&  theta < 135 * M_PI / 180 && theta > 45*M_PI/180) {
	//		max = dist;
	//		maxp = pt;
	//	}
	//}
	return m;
}

//camera fov 84.1(1.4678219 radians) x 53.8(0.938987138 radians)
//http://stackoverflow.com/questions/2866350/move-camera-to-fit-3d-scene

void PointCloudUtils::centroidDirVector(pcl::PointCloud<PointXYZRGBNormal>::Ptr bigCluster, double resolution, Matrix3f *intrinsics, Vector3f &center,Vector3f &normal,Vector3f &dir, Vector3f &position,bool cluster){
	CentroidPoint<PointXYZRGBNormal> cp;

	for (PointCloud<PointXYZRGBNormal>::iterator it = bigCluster->begin(); it != bigCluster->end(); it++)
	{
		cp.add(*it);
	}
	PointXYZRGBNormal centroid;
	cp.get(centroid);

	normal = Vector3f(centroid.normal_x, centroid.normal_y, centroid.normal_z);
	//Vector3f center(centroid.x, centroid.y, centroid.z);
	
	PointXYZRGBNormal pmin, pmax;
	getMinMax3D(*bigCluster, pmin, pmax);
	center = Vector3f((pmax.x + pmin.x) / 2.0, (pmax.y + pmin.y) / 2.0, (pmax.z + pmin.z) / 2.0);
	
	float radius = calculateRadius(bigCluster,center,normal);
	if (radius < 0.05) 
		radius = 0.05;
	std::cout << "radius " << radius << endl;
	float distance = (radius) / tan(1.04719755/2);
	float SxF = (*intrinsics)(0, 0);
	float bbratio = SxF*radius*2 / 424;
	float pixRatio = SxF*resolution;
	float ratio = pixRatio;// cluster ? min(bbratio, pixRatio) : bbratio;
	ratio = ratio < 0.01 ? 0.01 : ratio;
	PLYWriter wt;
	wt.write("cluster.ply", *bigCluster);
	position = center + (normal*distance);
	dir = position - center;
	dir.normalize();
}


void  PointCloudUtils::OBBDirVector(pcl::PointCloud<PointXYZRGBNormal>::Ptr bigCluster, double resolution, Matrix3f *intrinsics, Vector3f &dir, Vector3f &position,Vector3f &centroid, bool cluster){

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZRGBNormal, pcl::PointXYZ>(*bigCluster, *cloud);

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();


	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	
	float sizeX = abs(max_point_OBB.x - min_point_OBB.x);
	float sizeY = abs(max_point_OBB.y - min_point_OBB.y);
	float sizeZ = abs(max_point_OBB.z - min_point_OBB.z);
	float size = 0;
	if (sizeX == trimax(sizeX, sizeY, sizeZ)) {
		size = sizeX;
		if (sizeY == max(sizeY, sizeZ)) {
			//XY
			dir = Vector3f(0, 0, 1);
		}
		else {
			//XZ
			dir = Vector3f(0, 1, 0);
		}
	}
	else if (sizeY == trimax(sizeX, sizeY, sizeZ)) {
		size = sizeY;
		if (sizeX == max(sizeX, sizeZ)) {
			//YX
			dir = Vector3f(0, 0, 1);
		}
		else {
			//YZ
			dir = Vector3f(1, 0, 0);
		}
	}
	else if (sizeZ == trimax(sizeX, sizeY, sizeZ)) {
		size = sizeZ;
		if (sizeY == max(sizeY, sizeX)) {
			//ZY
			dir = Vector3f(1, 0, 0);
		}
		else {
			//ZX
			dir = Vector3f(0, 1, 0);
		}
	}
	dir = rotational_matrix_OBB * dir;
	Vector3f center = Vector3f(position_OBB.x, position_OBB.y, position_OBB.z);
	dir.normalize();

	float n1 = ((center + (dir*0.1)) - centroid).squaredNorm();
	float n2 = (center - centroid).squaredNorm();
	std::cout << "dist " << n1 << " " << n2 << endl;
	if (n1 < n2){
		dir = Vector3f(-dir.x(), -dir.y(), -dir.z());
	}

	float SxF = (*intrinsics)(0, 0);
	float oneToOneratio = resolution*SxF;
	float bbratio = SxF*size / 424;
	//float ratio = cluster? min(oneToOneratio, bbratio): bbratio;
	float ratio = bbratio;
	ratio = ratio < 0.001 ? 0.001 : ratio;
	position = center + (dir*(ratio));
	
}

Eigen::Transform<float, 3, Eigen::Affine> *PointCloudUtils::obtainViewpointClusterCentroid(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, int minCluster, double res, int width, int height) {
	int angle = 2;
	pcl::PointCloud<PointXYZRGBNormal>::Ptr bigCluster(new pcl::PointCloud<PointXYZRGBNormal>);
	double resolution = res;
	*bigCluster = *cloud;
	bool cluster = false;
	if (cloud->size() > 3 * minCluster) {
		while (angle <= 360) {
			//std::vector <pcl::PointIndices> clusters = segmentRegionGrowing(cloud, res, minCluster, angle);
			std::vector <pcl::PointIndices> clusters = segmentCloudByNormals(cloud, res, minCluster,angle);
			pcl::PointIndices::Ptr bigClusterIndices(new pcl::PointIndices);
			int maxCount = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
			{
				if (it->indices.size() > maxCount) {
					bigClusterIndices->indices = it->indices;
					maxCount = it->indices.size();
				}
			}
			if (maxCount > minCluster) {
				pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
				extract.setInputCloud(cloud);
				extract.setIndices(bigClusterIndices);
				extract.filter(*bigCluster);
				//resolution = computeCloudResolution(bigCluster);
				std::cout << "cluster\n";
				cluster = true;
				break;
			}
			std::cout << "found: " << maxCount << endl;
			angle += 8;
		}
	}

	Vector3f dir;
	Vector3f position;
	Vector3f centroid;
	Vector3f normal;
	centroidDirVector(bigCluster, resolution, intrinsics,  centroid, normal, dir, position,cluster);


	//OBBDirVector(bigCluster, resolution, intrinsics, dir, position, Vector3f(centroid.x(), centroid.y(), centroid.z()), cluster);
	float nx = dir.x();
	float ny = dir.y();
	float nz = dir.z();
	float n = sqrt(pow(nx, 2) + pow(ny, 2) + pow(nz, 2));
	float h1 = max(dir.x() - n, nx + n);
	float h2 = ny;
	float h3 = nz;
	float h = sqrt(pow(h1, 2) + pow(h2, 2) + pow(h3, 2));

	Vector3f right(-2 * h1*h3 / pow(h, 2), -2 * h2*h3 / pow(h, 2), 1 - 2 * pow(h3, 2) / pow(h, 2));
	Vector3f up(-2 * h1*h2 / pow(h, 2), 1 - 2 * pow(h2, 2) / pow(h, 2), -2 * h2*h3 / pow(h, 2));

	Matrix3f rotationMat = makeRotationDir(dir, up);
	Eigen::Translation<float, 3> translate(position.x(), position.y(), position.z());
	Eigen::Transform<float, 3, Affine> *mat = new Eigen::Transform<float, 3, Affine>();
	*mat = translate*rotationMat;

	//if (!cluster){
	optimizeDistance(bigCluster, mat, intrinsics, centroid, normal, dir, position, width, height);
	rotationMat = makeRotationDir(dir, up);
	Eigen::Translation<float, 3> translateFix(position.x(), position.y(), position.z());
	delete mat;
	mat = new Eigen::Transform<float, 3, Affine>();
	*mat = translateFix*rotationMat;
	//}
	
	
	return mat;
}


Eigen::Transform<float, 3, Eigen::Affine> *PointCloudUtils::obtainViewpointCalibrationDirection(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, map<string, float*> id_calibration, int minCluster, string &viewRes,double res,int &lastidx) {
	int angle = 2;

	pcl::PointCloud<PointXYZRGBNormal>::Ptr bigCluster(new pcl::PointCloud<PointXYZRGBNormal>);
	double resolution = res;
	*bigCluster = *cloud;
	bool cluster = false;
	if (cloud->size() > 3 * minCluster) {
		while (angle <= 360) {
			//std::vector <pcl::PointIndices> clusters = segmentRegionGrowing(cloud, res, minCluster, angle);
			std::vector <pcl::PointIndices> clusters = segmentCloudByNormals(cloud, res, minCluster,angle);
			pcl::PointIndices::Ptr bigClusterIndices(new pcl::PointIndices);
			int maxCount = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
			{
				if (it->indices.size() > maxCount) {
					bigClusterIndices->indices = it->indices;
					maxCount = it->indices.size();
				}
			}
			if (maxCount > minCluster) {
				pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
				extract.setInputCloud(cloud);
				extract.setIndices(bigClusterIndices);
				extract.filter(*bigCluster);
				//resolution = computeCloudResolution(bigCluster);
				std::cout << "cluster\n";
				cluster = true;
				break;
			}
			std::cout << "found: " << maxCount << endl;
			angle += 8;
		}
	}

	Vector3f dir;
	Vector3f position;
	Vector3f centroid;
	Vector3f normal;
	centroidDirVector(bigCluster, resolution, intrinsics, centroid, normal, dir, position, cluster);

	Vector4f norm(dir.x(), dir.y(), dir.z(), 1);
	Vector4f negnorm(-dir.x(), -dir.y(), -dir.z(), 1);
	float maxDot = std::numeric_limits<float>::min();
	map<string, float*>::iterator iter;
	string minName;
	int minid = -1;
	Eigen::Transform<float, 3, Affine> *minTransf = NULL;
	int i = 0;
	for (iter = id_calibration.begin(); iter != id_calibration.end(); iter++) {
		if (i == lastidx) {
			i++;
			continue;
		}
		Eigen::Quaternionf q = Eigen::Quaternionf(iter->second[6], iter->second[3], iter->second[4], iter->second[5]);
		Eigen::Translation<float, 3> translate(iter->second[0], iter->second[1], iter->second[2]);
		Eigen::Transform<float, 3, Affine> *transf = new Eigen::Transform<float, 3, Affine>();
		*transf = translate *q;
		Vector4f front = transf->matrix().col(2);
		front.normalize();
		norm.normalize();
		float dotProduct =abs( front.dot(norm));
		/*float d1 = (front - norm).norm();
		float d2 = (front - negnorm).norm();*/
		std::cout << iter->first << " " << dotProduct<< " " << " min " << maxDot << endl;
		if (dotProduct > maxDot) {
			maxDot= dotProduct;
			minTransf = transf;
			minName = iter->first;
			minid = i;
		}
		else {
			delete transf;
		}
		i++;
		//delete transf;
	}
	std::cout << "best viewpoint is " << minName << endl;;
	viewRes = minName;
	lastidx = minid;
	return minTransf;
}


Eigen::Transform<float, 3, Eigen::Affine> *PointCloudUtils::obtainViewpointCalibrationPosition(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, map<string, float*> id_calibration, int minCluster, string &viewRes, double res, int &lastidx) {
	int angle = 2;

	pcl::PointCloud<PointXYZRGBNormal>::Ptr bigCluster(new pcl::PointCloud<PointXYZRGBNormal>);
	double resolution = res;
	*bigCluster = *cloud;
	bool cluster = false;
	if (cloud->size() > 3 * minCluster) {
		while (angle <= 360) {
			//std::vector <pcl::PointIndices> clusters = segmentRegionGrowing(cloud, res, minCluster, angle);
			std::vector <pcl::PointIndices> clusters = segmentCloudByNormals(cloud, res, minCluster,angle);
			pcl::PointIndices::Ptr bigClusterIndices(new pcl::PointIndices);
			int maxCount = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
			{
				if (it->indices.size() > maxCount) {
					bigClusterIndices->indices = it->indices;
					maxCount = it->indices.size();
				}
			}
			if (maxCount > minCluster) {
				pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
				extract.setInputCloud(cloud);
				extract.setIndices(bigClusterIndices);
				extract.filter(*bigCluster);
				//resolution = computeCloudResolution(bigCluster);
				std::cout << "cluster\n";
				cluster = true;
				break;
			}
			std::cout << "found: " << maxCount << endl;
			angle += 8;
		}
	}

	Vector3f dir;
	Vector3f position;
	Vector3f centroid;
	Vector3f normal;
	centroidDirVector(bigCluster, resolution, intrinsics, centroid, normal, dir, position, cluster);

	
	float minDist = std::numeric_limits<float>::max();
	map<string, float*>::iterator iter;
	string minName;
	int minid = -1;
	Eigen::Transform<float, 3, Affine> *minTransf = NULL;
	int i = 0;
	for (iter = id_calibration.begin(); iter != id_calibration.end(); iter++) {
		if (i == lastidx) {
			i++;
			continue;
		}
		Eigen::Quaternionf q = Eigen::Quaternionf(iter->second[6], iter->second[3], iter->second[4], iter->second[5]);
		Eigen::Translation<float, 3> translate(iter->second[0], iter->second[1], iter->second[2]);
		Eigen::Transform<float, 3, Affine> *transf = new Eigen::Transform<float, 3, Affine>();
		*transf = translate *q;

		Vector3f vppos(iter->second[0], iter->second[1], iter->second[2]);
		float d1 = (vppos - position).norm();
		std::cout << iter->first << " " << d1 << " " << " min " << minDist << endl;
		if (d1 < minDist) {
			minDist = d1;
			minTransf = transf;
			minName = iter->first;
			minid = i;
		}
		else {
			delete transf;
		}
		i++;
		//delete transf;
	}
	std::cout << "best viewpoint is " << minName << endl;;
	viewRes = minName;
	lastidx = minid;
	return minTransf;
}

void PointCloudUtils::optimizeDistance(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Eigen::Transform<float, 3, Affine> *mat, Matrix3f *intrinsics, Vector3f &center, Vector3f &normal, Vector3f &dir, Vector3f &position, int width, int height){
	//trying to center and maximze space
	Eigen::Transform<float, 3, Affine> revert = mat->inverse();
	float SxF = (*intrinsics)(0, 0);
	float SyF = (*intrinsics)(1, 1);
	float ox = (*intrinsics)(0, 2);
	float oy = (*intrinsics)(1, 2);
	int minu = std::numeric_limits<int>::max(); int depthminu;
	int minv = std::numeric_limits<int>::max(); int depthminv;
	int maxu = std::numeric_limits<int>::min(); int depthmaxu;
	int maxv = std::numeric_limits<int>::min(); int depthmaxv;
	int minGlobalDepth = std::numeric_limits<int>::max();

	for (PointCloud<PointXYZRGBNormal>::iterator it = cloud->begin(); it != cloud->end(); it++)
	{
		PointXYZRGBNormal prev = pcl::transformPointWithNormal(*it, revert);
		int u = floorf((((SxF * prev.x) / prev.z) + ox) + 0.5);
		int v = floorf((((SyF * prev.y) / prev.z) + oy) + 0.5);
		int d = (int)floorf((prev.z * 1000) + 0.5);

		if (u < minu){ minu = u; depthminu = d; }
		if (v < minu){ minv = v; depthminv = d; }
		if (u > maxu){ maxu = u; depthmaxu = d; }
		if (v > maxv){ maxv = v; depthmaxv = d; }
		minGlobalDepth = min(minGlobalDepth, d);
	}

	int sizeu = maxu - minu;
	int sizev = maxv - minv;
	int halfdist, mindepth;
	float z;
	float worldSize;
	float fov;
	float o, sF;
	int dim;
	if (sizeu > sizev){
		fov = 1.23220245;
		halfdist = (width - sizeu) / 2;
		mindepth = min(depthminu, depthmaxu);
		o = ox; 
		sF = SxF;
		dim = width;
	}
	else {
		fov = 1.04719755;
		halfdist = (height - sizev) / 2;
		mindepth = min(depthminv, depthmaxv);
		o = oy;
		sF = SyF;
		dim = height;
	}
	
	PointXYZRGBNormal p;
	p.z = mindepth / 1000.0;;
	p.x = p.z * (width / 2 - ox) / SxF;
	p.y = p.z  * (height / 2 - oy) / SyF;
	PointXYZRGBNormal pcent = pcl::transformPointWithNormal(p, *mat);

	PointXYZRGBNormal p2;
	p2.z = minGlobalDepth / 1000.0;
	p2.x = p2.z * (width / 2 - ox) / SxF;
	p2.y = p2.z  * (height / 2 - oy) / SyF;
	PointXYZRGBNormal p2cent = pcl::transformPointWithNormal(p2, *mat);

	worldSize = sqrt(pow((p2.z * (halfdist - o) / sF) - (p2.z * (dim - halfdist - o) / sF), 2));
	center = Vector3f(p2cent.x,p2cent.y,p2cent.z);
	worldSize = worldSize < 0.05 ? 0.05 : worldSize;
	float distance = (worldSize/2) / tan(fov / 2);
	distance = distance > 7000 ? 7000 : distance;
	position = center + (normal*distance);
	dir = position - center;
	dir.normalize();
}

Eigen::Transform<float, 3, Eigen::Affine> *PointCloudUtils::obtainViewpointCentroid(pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, double res) {

	
	CentroidPoint<PointXYZRGBNormal> cp;

	for (PointCloud<PointXYZRGBNormal>::iterator it = cloud->begin(); it != cloud->end(); it++)
	{
		cp.add(*it);
	}
	PointXYZRGBNormal centroid;
	cp.get(centroid);
	float SxF = (*intrinsics)(0, 0);
	Vector3f normal(centroid.normal_x, centroid.normal_y, centroid.normal_z);
	Vector3f center(centroid.x, centroid.y, centroid.z);
	Vector3f position = center + (normal*SxF*res);
	Vector3f dir = position - center;
	dir.normalize();
	Matrix3f rotationMat = makeRotationDir(dir, Vector3f(0, 1, 0));
	Eigen::Translation<float, 3> translate(position.x(), position.y(), position.z());
	Eigen::Transform<float, 3, Affine> *mat = new Eigen::Transform<float, 3, Affine>();
	*mat = translate*rotationMat;


	return mat;
}

Matrix3f PointCloudUtils::makeRotationDir(const Vector3f& direction, const Vector3f& up )
{
	
	Vector3f  f = direction;
	Vector3f  u = up.normalized();
	Vector3f  s = f.cross(u);
	if (s.isZero()) {
		u = Vector3f(1, 0, 0);
		s = f.cross(u);
	}
	s.normalize();
	u = s.cross(f);
	u.normalize();

	Matrix3f rot;
	rot(0,0) = s.x();
	rot(1, 0) = s.y();
	rot(2, 0) = s.z();
	rot(0, 1) = u.x();
	rot(1, 1) = u.y();
	rot(2, 1) = u.z();
	rot(0, 2) = -f.x();
	rot(1, 2) = -f.y();
	rot(2, 2) = -f.z();
	return rot;
	
}

void PointCloudUtils::writeClustersDisk(std::vector <pcl::PointIndices> clusters, pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud, bool colorize){

	pcl::PLYWriter writer;
	int j = 0;
	srand(time(NULL));
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it, j++)
	{
		int r = rand() % 255;
		int g = rand() % 255;
		int b = rand() % 255;
		pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_cluster_don(new pcl::PointCloud<PointXYZRGBNormal>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			PointXYZRGBNormal pn = cloud->points[*pit];
			//check  if its NAN
			if(pn.normal_x == pn.normal_x){
				if (colorize) 
				{
					pn.r = r; pn.g = g; pn.b = b;
				}
				cloud_cluster_don->points.push_back(pn);
			}
		}

		cloud_cluster_don->width = int(cloud_cluster_don->points.size());
		cloud_cluster_don->height = 1;
		cloud_cluster_don->is_dense = true;

		//Save cluster
		cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size() << " data points." << std::endl;
		stringstream ss;
		ss << "don_cluster_" << j << ".ply";

		std::vector<int> i;
		pcl::removeNaNFromPointCloud(*cloud_cluster_don, *cloud_cluster_don, i);
		writer.write<pcl::PointXYZRGBNormal>(ss.str(), *cloud_cluster_don, false);
	}
}