#include "LDI.h"
#include "PointCloudUtils.h"
#include "pcl\filters\extract_indices.h"
LDI::LDI(float t)
{
	redundancyTreshold = t;
}


LDI::~LDI()
{
	for each(Layer *l in data){
		delete l;
	}
}
void LDI::encodeGenerate(PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics) {

	double res = PointCloudUtils::computeCloudResolution(cloud);
	int count = (int) 3*cloud->size() /100;
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints(new PointCloud<PointXYZRGBNormal>);
	pcl::PointIndices::Ptr ToRetry(new pcl::PointIndices);
	while(true){
		//Eigen::Affine3f extrinsics = PointCloudUtils::obtainViewpointCentroid(cloud, intrinsics,res);
		Eigen::Affine3f *extrinsics = PointCloudUtils::obtainViewpointClusterCentroid(cloud, intrinsics,count,res,512,424);
		Layer *l = new Layer(intrinsics, extrinsics, 512, 424);
		encodeGeneratePass(cloud, repeatedPoints, ToRetry, l);
		std::cout <<"retry" << ToRetry->indices.size() << "\n";
		if (ToRetry->indices.size() == 0){
			data.push_back(l);
			break;
		}
		pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ToRetry);
		extract.filter(*cloud);
		ToRetry->indices.clear();
		data.push_back(l);
	}
	if (repeatedPoints->size() != 0){
		pcl::PLYWriter pl;
		pl.write("repeated.ply",*repeatedPoints);
	}
}

void LDI::encodeGeneratePass(PointCloud<PointXYZRGBNormal>::Ptr cloud,
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints,
	pcl::PointIndices::Ptr ToRetry,Layer *l) {

	pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = cloud->begin();
	int i = 0;
	while (it != cloud->end()) {
		PointXYZRGBNormal p = *it;
		DepthPixel dp = l->projectPoint(p);
		//check and add point
		
		if (l->pixelInside(dp)) {
			if (l->isRepeatedWorldSpace(dp, p, redundancyTreshold)) {
				/*l.combineDepths(dp);*/
				repeatedPoints->push_back(p);
			}
			else {
				if (l->getPixel(dp.u, dp.v).depth == 0) {
					l->addPixel(dp);
				}
				else if (dp.depth < l->getPixel(dp.u, dp.v).depth) {
					PointXYZRGBNormal oldPoint = l->removePoint(l->getPixel(dp.u, dp.v));
					l->addPixel(dp);	
					*it = oldPoint;
					ToRetry->indices.push_back(i);
				}
				else
				{
					ToRetry->indices.push_back(i);
				}
			}
		}
		else
		{
			ToRetry->indices.push_back(i);
		}

		it++;
		i++;
	}
}

NumbersTree LDI::encodeOptimal(PointCloud<PointXYZRGBNormal>::Ptr cloud, std::queue<string> ids, map<string, float*> id_calibration, Matrix3f *intrinsics, NumbersTree nt,int level, int &maxlevel) {
	
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints(new PointCloud<PointXYZRGBNormal>);
	pcl::PointIndices::Ptr ToRetry(new pcl::PointIndices);
	//Create layers
	int i = 0;
	int mylevel = level + 1;
	if (mylevel > maxlevel) {
		NumbersTree res;
		res.points = -1;
		res.index = nt.index;
		res.medianDepth = -1;
		return res;
	}
	int sizeQueue = ids.size();
	for (int j = 0; j < sizeQueue; j++) {
		string id = ids.front();
		ids.pop();

		Eigen::Quaternionf q = Eigen::Quaternionf(id_calibration[id][6], id_calibration[id][3], id_calibration[id][4], id_calibration[id][5]);
		Eigen::Translation<float, 3> translate(id_calibration[id][0], id_calibration[id][1], id_calibration[id][2]);
		Eigen::Transform<float, 3, Affine> *transf = new Eigen::Transform<float, 3, Affine>();
		*transf = translate *q;
		Layer *l = new Layer(intrinsics, transf, 512, 424);
		encodeGeneratePass(cloud, repeatedPoints, ToRetry, l);

		NumbersTree t;
		t.points = l->getCount();// take into account repeated as encoded 
		t.index = id;
		t.medianDepth = l->medianDepth();

		if(t.points > 0 && ToRetry->indices.size() > 0){

			pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
			PointCloud<PointXYZRGBNormal>::Ptr recursionPoints(new PointCloud<PointXYZRGBNormal>);
			extract.setInputCloud(cloud);
			extract.setIndices(ToRetry);
			extract.filter(*recursionPoints);
			ids.push(id);
			t = encodeOptimal(recursionPoints, ids,id_calibration, intrinsics, t,mylevel,maxlevel);

		}else if (t.points >0 && ToRetry->indices.size() == 0) {
			maxlevel = mylevel;
		}
		ToRetry->indices.clear();
		repeatedPoints->clear();
		nt.childNodes.push_back(t);
		i++;
		delete l;
	}
	return nt;
}

void LDI::encodeGreedyOptimal(PointCloud<PointXYZRGBNormal>::Ptr cloud,  map<string, float*> id_calibration, Matrix3f *intrinsics) {

	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints(new PointCloud<PointXYZRGBNormal>);
	pcl::PointIndices::Ptr ToRetry(new pcl::PointIndices);
	pcl::PointIndices::Ptr ToRetryBest;
	//Create layers
	int i = 0;
	int maxPoints = INT_MIN;
	string maxid = "";
	Layer *bestLayer = NULL;

	map<string, float*>::iterator iter;
	for (iter = id_calibration.begin(); iter != id_calibration.end(); iter++) {
		Eigen::Quaternionf q = Eigen::Quaternionf(iter->second[6], iter->second[3], iter->second[4], iter->second[5]);
		Eigen::Translation<float, 3> translate(iter->second[0], iter->second[1], iter->second[2]);
		Eigen::Transform<float, 3, Affine> *transf = new Eigen::Transform<float, 3, Affine>();
		*transf = translate *q;
		Layer *l = new Layer(intrinsics, transf, 512, 424);
		encodeGeneratePass(cloud, repeatedPoints, ToRetry, l);
		std::cout << "Layer " << iter->first << l->getCount() << endl;
		if (l->getCount() > maxPoints) {
			maxid = iter->first;
			maxPoints = l->getCount();
			if(bestLayer != NULL)
				delete bestLayer;
			bestLayer = l;
			ToRetryBest = boost::make_shared<PointIndices>(*ToRetry);
		}
		else {
			delete l;
		}	
		ToRetry->indices.clear();
		repeatedPoints->clear();
		i++;
		
	}
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	PointCloud<PointXYZRGBNormal>::Ptr recursionPoints(new PointCloud<PointXYZRGBNormal>);
	extract.setInputCloud(cloud);
	extract.setIndices(ToRetryBest);
	extract.filter(*recursionPoints);
	data.push_back(bestLayer);
	std::cout << "best " << maxid << endl;
	if(recursionPoints->size() > 0){
		encodeGreedyOptimal(recursionPoints, id_calibration, intrinsics);
	}
	return;
}

void LDI::encodeCreatedLayers(PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, int layerLimit) {

	map<string, float*>::iterator iter;
	PointCloud<PointXYZRGBNormal>::Ptr encoded(new PointCloud<PointXYZRGBNormal>);
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints(new PointCloud<PointXYZRGBNormal>);
	PointCloud<PointXYZRGBNormal>::Ptr outOfLayers(new PointCloud<PointXYZRGBNormal>);
	pcl::PointIndices::Ptr ToRetry(new pcl::PointIndices);
	int i = 0;
	while (i < data.size()) {

		encodeGeneratePass(cloud, repeatedPoints, ToRetry, data[i]);
		
		pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ToRetry);
		extract.filter(*cloud);
		ToRetry->indices.clear();
		//repeatedPoints->clear();
		i++;
	}
	std::cout << "Repeated " << repeatedPoints->size() << endl;
}

void LDI::encodeFindBest(PointCloud<PointXYZRGBNormal>::Ptr cloud, Matrix3f *intrinsics, map<string, float*> id_calibration, int mode) {

	double res = PointCloudUtils::computeCloudResolution(cloud);
	int count = (int)3 * cloud->size() / 100;
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints(new PointCloud<PointXYZRGBNormal>);
	pcl::PointIndices::Ptr ToRetry(new pcl::PointIndices);
	int lastidx = -1;
	while (true) {
		//Eigen::Affine3f extrinsics = PointCloudUtils::obtainViewpointCentroid(cloud, intrinsics,res);
		string viewpoint;
		Eigen::Affine3f *extrinsics = NULL;
		if (id_calibration.size() == 0) {
			break;
		}
		if(mode == EFB_MODE_DIRECTION){
			extrinsics = PointCloudUtils::obtainViewpointCalibrationDirection(cloud, intrinsics, id_calibration,count,viewpoint,res,lastidx);
		}
		else  {
			extrinsics = PointCloudUtils::obtainViewpointCalibrationPosition(cloud, intrinsics, id_calibration, count, viewpoint, res, lastidx);
		}
		Layer *l = new Layer(intrinsics, extrinsics, 512, 424);
		encodeGeneratePass(cloud, repeatedPoints, ToRetry, l);
		if (ToRetry->indices.size() == 0) {
			data.push_back(l);
			break;
		}
		if (ToRetry->indices.size() == cloud->size()) {
			std::cout << "deleting " << viewpoint;
			id_calibration.erase(viewpoint);
		}
		else {
			data.push_back(l);
		}
		pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ToRetry);
		extract.filter(*cloud);
		ToRetry->indices.clear();
	}
	if (repeatedPoints->size() != 0) {
		pcl::PLYWriter pl;
		pl.write("repeated.ply", *repeatedPoints);
	}
}

void LDI::createEmptyLayersFromTree(Matrix3f *intrinsics,map<string, float*> id_calibration,NumbersTree t ) {
	if (t.index == "") {
		createEmptyLayersFromTree(intrinsics, id_calibration, t.childNodes[0]);
		return;
	}
	float *calibration = id_calibration[t.index];

	Eigen::Quaternionf q = Eigen::Quaternionf(calibration[6], calibration[3], calibration[4], calibration[5]);
	Eigen::Translation<float, 3> translate(calibration[0], calibration[1], calibration[2]);

	Eigen::Transform<float, 3, Affine> *transf = new Eigen::Transform<float, 3, Affine>();
	*transf = translate *q;
	Layer *l = new Layer(intrinsics, transf, 512, 424);
	data.push_back(l);
	if (t.childNodes.size() != 0 && t.childNodes[0].index != "") {
		createEmptyLayersFromTree(intrinsics, id_calibration, t.childNodes[0]);
	}
	return;
}

void LDI::encodeOriginal(PointCloud<PointXYZRGBNormal>::Ptr cloud, map<string, float*> id_calibration,Matrix3f *intrinsics,int recursion,int layerLimit){

	map<string, float*>::iterator iter;
	PointCloud<PointXYZRGBNormal>::Ptr encoded(new PointCloud<PointXYZRGBNormal>);
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints(new PointCloud<PointXYZRGBNormal>);
	PointCloud<PointXYZRGBNormal>::Ptr outOfLayers(new PointCloud<PointXYZRGBNormal>);
	pcl::PointIndices::Ptr ToRetry(new pcl::PointIndices);
	std::cout << "Total " << cloud->size()<<endl;
	//Create layers
	int i = 0;
	for (iter = id_calibration.begin(); iter != id_calibration.end() ; iter++) {
		Eigen::Quaternionf q = Eigen::Quaternionf(iter->second[6], iter->second[3], iter->second[4], iter->second[5]);
		Eigen::Translation<float,3> translate(iter->second[0], iter->second[1], iter->second[2]);
		Eigen::Transform<float, 3, Affine> *transf = new Eigen::Transform<float, 3, Affine>();
		*transf =  translate *q;
		Layer *l = new Layer(intrinsics, transf, 512, 424);
		data.push_back(l);
		//delete transf;
	}

	while (true) {
		encodeOrignalPass(cloud, repeatedPoints, outOfLayers, ToRetry, intrinsics,layerLimit);
		//std::cout << ToRetry->indices.size() << "\n";
		if (ToRetry->indices.size() == 0)
			break;
		pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ToRetry);
		extract.filter(*cloud);
		ToRetry->indices.clear();
	}
	
	//pcl::PLYWriter wt;
	std::vector<int> aux_indices;
	
	if (outOfLayers->size() > 0) {
		encodeOriginal(outOfLayers, id_calibration, intrinsics, recursion + 1, layerLimit);
	}
	std::cout << "Repeated " << repeatedPoints->size() << endl;
	removeNaNNormalsFromPointCloud(*cloud, *cloud,aux_indices);
	stringstream ss;
	//ss << "repeated" << recursion << ".ply";
	//wt.write(ss.str(), *repeatedPoints);
	encoded.reset();
	repeatedPoints.reset();
	outOfLayers.reset();
	ToRetry.reset();
}

void LDI::encodeOrignalPass(PointCloud<PointXYZRGBNormal>::Ptr cloud,
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints,
	PointCloud<PointXYZRGBNormal>::Ptr outOfLayers,
	pcl::PointIndices::Ptr ToRetry,
	Matrix3f *intrinsics,
	int layerLimit) {
	int i = 0;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = cloud->begin();
	while (it != cloud->end()) {
		bool ok = false;
		PointXYZRGBNormal p = *it;
		int k = 0;
		for each(Layer *l in data) {
			if (layerLimit != 0 && k >= layerLimit)
					return;
			k++;
			DepthPixel dp = l->projectPoint(p);
			//check and add point
			if (l->pixelInside(dp)) {
				if (l->isRepeatedWorldSpace(dp,p,redundancyTreshold)) {
					/*l.combineDepths(dp);*/
					repeatedPoints->push_back(p);
					ok = true;
					break;
				}
				else {
					if (l->getPixel(dp.u, dp.v).depth == 0) {
						l->addPixel(dp);
						ok = true;
						break;
					}else if (dp.depth < l->getPixel(dp.u, dp.v).depth){
						PointXYZRGBNormal oldPoint = l->removePoint(l->getPixel(dp.u, dp.v));
						l->addPixel(dp);
						*it = oldPoint;
						ToRetry->indices.push_back(i);
						ok = true;
						break;
					}
				}
			}
		}
		if (!ok) outOfLayers->push_back(p);
		it++;
		i++;
	}
}

void LDI::encodeLDI(PointCloud<PointXYZRGBNormal>::Ptr cloud, float* mainViewpoint, Matrix3f *intrinsics,float distance) {
	PointCloud<PointXYZRGBNormal>::Ptr encoded(new PointCloud<PointXYZRGBNormal>);
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints(new PointCloud<PointXYZRGBNormal>);
	PointCloud<PointXYZRGBNormal>::Ptr outOfSight(new PointCloud<PointXYZRGBNormal>);
	pcl::PointIndices::Ptr ToRetry(new pcl::PointIndices);

	
	Eigen::Quaternionf q = Eigen::Quaternionf(mainViewpoint[6], mainViewpoint[3], mainViewpoint[4], mainViewpoint[5]);
	Eigen::Translation<float, 3> translate(mainViewpoint[0], mainViewpoint[1], mainViewpoint[2]);

	Eigen::Transform<float, 3, Affine> *transf = new Eigen::Transform<float, 3, Affine>();
	*transf = translate *q;

	while (true) {
		Layer *l = new Layer(intrinsics, transf, 512, 424);
		data.push_back(l);
		encodeLDIPass(cloud, repeatedPoints,outOfSight, ToRetry,intrinsics,l);
		//std::cout << ToRetry->indices.size() << "\n";
		if (ToRetry->indices.size() == 0)
			break;
		pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(ToRetry);
		extract.filter(*cloud);
		ToRetry->indices.clear();
	}

	std::vector<int> aux_indices;
	std::cout << "Repeated " << repeatedPoints->size() << endl;
	std::cout << "Out of sight " << outOfSight->size() << endl;
}

void LDI::encodeLDIPass(PointCloud<PointXYZRGBNormal>::Ptr cloud,
	PointCloud<PointXYZRGBNormal>::Ptr repeatedPoints,
	PointCloud<PointXYZRGBNormal>::Ptr outOfSight,
	pcl::PointIndices::Ptr ToRetry,
	Matrix3f *intrinsics, Layer *l) {
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it = cloud->begin();
	int i = 0;
	while (it != cloud->end()) {
		PointXYZRGBNormal p = *it;

		DepthPixel dp = l->projectPoint(p);
		//check and add point
		if (l->pixelInside(dp)) {
			if (l->isRepeatedImageSpaceThresh(dp, redundancyTreshold)) {
				/*l.combineDepths(dp);*/
				repeatedPoints->push_back(p);
			}
			else {
				if (l->getPixel(dp.u, dp.v).depth == 0) {
					l->addPixel(dp);
				}
				else if (dp.depth < l->getPixel(dp.u, dp.v).depth){
					PointXYZRGBNormal oldPoint = l->removePoint(l->getPixel(dp.u, dp.v));
					l->addPixel(dp);				
					*it = oldPoint;
					ToRetry->indices.push_back(i);
				}
				else{
					ToRetry->indices.push_back(i);
				}
			}
		}
		else {
			outOfSight->push_back(p);
		}
		it++;
		i++;
	}
}

void LDI::writeToDisk(string path){

	int i = 0;
	for each (Layer *l in data)
	{
		l->write(path,i++);
	}

	/*std::ostringstream path;
	path << ExePath() << "\\Output\\depthbg\\depthbgdata" << secCount << "," << frameCount;
	std::string s = path.str();
	std::wstring stemp = std::wstring(s.begin(), s.end());
	LPCWSTR swb = stemp.c_str();
	hFile = CreateFileW(swb, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	DWORD dwBytesWritten;
	WriteFile(hFile, pBuffer, dwidth*dheight * sizeof(UINT16), &dwBytesWritten, NULL);
	CloseHandle(hFile);*/
}

void LDI::writeToDiskImg(string path){
	int i = 0;
	std::cout << "----------------" << endl;
	for each (Layer *l in data)
	{
		l->writeImg(path, i++);
	}
}

void LDI::writeToPly(string path) {
	PointCloud<PointXYZRGBNormal>::Ptr cloud(new PointCloud<PointXYZRGBNormal>);
	for each(Layer *l in data) {
		l->deprojectAll(cloud);
	}
	PLYWriter wt;
	wt.write(path, *cloud);
}

void LDI::getFrameARGBData(int layer, uint8_t* &data,int size)
{
	if (layer < this->data.size())
		this->data[layer]->getFrameARGBData(data);
	else
		memset(data, 0,size);
	return;
}

void LDI::getFrameDepthARGBData(int layer, uint8_t* &data,int size)
{
	if (layer < this->data.size())
		this->data[layer]->getFrameDepthARGBData(data);
	else
		memset(data, 0, size);
	return;
}

void LDI::getFrameDepthData(int layer, short* &data, int size)
{
	if (layer < this->data.size())
		this->data[layer]->getFrameDepthData(data);
	else
		memset(data, 0, size);
	return;
}

void LDI::getFrameNormalARGBData(int layer, uint8_t* &data, int size) {
	if (layer < this->data.size())
		this->data[layer]->getFrameNormalARGBData(data);
	else
		memset(data, 0, size);
	return;
}

