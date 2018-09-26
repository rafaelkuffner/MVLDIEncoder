#include "Layer.h"
#include <boost/math/special_functions/round.hpp>

Layer::Layer(){
	*intrinsicsMatrix = Matrix3f::Zero();
	*extrinsics = Transform<float, 3, Affine>::Identity();
	width = 0;
	height = 0;
}

Layer::Layer(Matrix3f *im, Transform<float, 3, Affine> *e, int w, int h){
	intrinsicsMatrix = im;
	extrinsics = e;
	width = w;
	height = h;
	pixs = new DepthPixel[w*h];
};

Layer::~Layer() {
	delete [] pixs;
}

DepthPixel Layer::getPixel(int u, int v){
	int index = v*width + u;
	return pixs[index];
}

int Layer::getCount() {
	int count = 0;
	int i = 0;
	while (i < width*height) {
		DepthPixel dp = pixs[i];
		if (dp.depth != 0) count++;
		i++;
	}
	return count;
}

void Layer::combineDepths(DepthPixel pix){
	int index = pix.v*width + pix.u;
	pixs[index].depth = (int)(pixs[index].depth + pix.depth) / 2;
}
bool Layer::pixelInside(DepthPixel pix){
	return pix.u >= 0 && pix.u < width && pix.v >= 0 && pix.v < height && pix.depth > 0  && pix.depth && pix.depth;
}
void Layer::addPixel( DepthPixel pix){
	int index = pix.v*width + pix.u;
	pixs[index] = pix;
}

PointXYZRGBNormal Layer::removePoint(DepthPixel dp){
	PointXYZRGBNormal ret = deprojectPoint(dp);
	addPixel(DepthPixel(dp.u,dp.v,0,Color(),Vector3()));
	return ret;
}

bool Layer::isRepeatedImageSpaceThresh(DepthPixel pix,float threshold){
	//float SxF = (*intrinsicsMatrix)(0, 0);
	//float threshold = pix.depth / SxF;
	return abs(int(pix.depth - getPixel(pix.u, pix.v).depth)) < threshold;
}

bool Layer::isRepeatedImageSpace(DepthPixel pix){
	float SxF = (*intrinsicsMatrix)(0, 0);
	float threshold = pix.depth / SxF;
	return abs(int(pix.depth - getPixel(pix.u, pix.v).depth)) < threshold;
}

bool Layer::isRepeatedWorldSpace(DepthPixel dp, PointXYZRGBNormal &pt, float redundancyTreshold){
	float SxF = (*intrinsicsMatrix)(0, 0);
	float threshold = dp.depth / SxF;
	int npix = (int)floor((redundancyTreshold/threshold) + 0.5);
	int umin = dp.u - npix < 0 ? 0 : dp.u - npix;
	int vmin = dp.v - npix < 0 ? 0 : dp.v - npix;
	int umax = dp.u + npix >= width ? width-1 : dp.u + npix;
	int vmax = dp.v + npix >= height ? height-1 : dp.v + npix;
	for (int i = umin; i <= umax; i++){
		for (int j = vmin; j <= vmax; j++){
			PointXYZRGBNormal temp = deprojectPoint(getPixel(i, j));
			if (squaredEuclideanDistance(pt, temp) < (redundancyTreshold/1000))
				return true;
		}
	}
	return false;
}

PointXYZRGBNormal Layer::deprojectPoint(DepthPixel dp){
	float SxF = (*intrinsicsMatrix)(0, 0);
	float SyF = (*intrinsicsMatrix)(1, 1);
	float ox = (*intrinsicsMatrix)(0, 2);
	float oy = (*intrinsicsMatrix)(1, 2);
	PointXYZRGBNormal ret;
	ret.z = dp.depth / 1000.0;
	ret.x = ret.z * (dp.u - ox) / SxF;
	ret.y = ret.z  * (dp.v - oy) / SyF;

	
	ret.r = dp.c.r();
	ret.g = dp.c.g();
	ret.b = dp.c.b();
	ret.normal_x = dp.normal.getX();
	ret.normal_y = dp.normal.getY();
	ret.normal_z = dp.normal.getZ();
	
	ret = pcl::transformPointWithNormal(ret, *extrinsics);
	
	return ret;
}

DepthPixel Layer::projectPoint(const PointXYZRGBNormal &point) {	
	Eigen::Transform<float, 3, Affine> revert = extrinsics->inverse();

	PointXYZRGBNormal prev = pcl::transformPointWithNormal(point, revert);
	
	float SxF = (*intrinsicsMatrix)(0, 0);
	float SyF = (*intrinsicsMatrix)(1, 1);
	float ox = (*intrinsicsMatrix)(0, 2);
	float oy = (*intrinsicsMatrix)(1, 2);
	int u = floorf((((SxF * prev.x) / prev.z) + ox)+ 0.5);
	int v = floorf((((SyF * prev.y) / prev.z) + oy)+ 0.5);
	int d = (int)floorf((prev.z * 1000)+0.5);	
	return DepthPixel(u, v, d, Color(prev.r, prev.g, prev.b), Vector3(prev.normal_x, prev.normal_y, prev.normal_z));
}

void Layer::write(string path,int id) {

};

void Layer::writeImg(string path, int id){
	unsigned char *pix = new unsigned char[width*height * 4];
	unsigned char *pixDepths = new unsigned char[width*height * 4];
	unsigned char *pixNormals = new unsigned char[width*height * 4];

	int i = 0;
	int j = 0;
	int count = 0;
	while (i < width*height) {

		DepthPixel dp = pixs[i];
		if (dp.depth != 0) count++;
		BYTE intensity = static_cast<BYTE>((dp.depth % 256));
		
		double dm = floor(dp.depth / 256.0);
		int g = ((int)dm) % 256;
		int r = (int)floor(dm / 256.0);
	
		pix[j] = dp.c.b();
		int nb = int((abs(dp.normal.getZ()) * 255.0) + 0.5);
		pixNormals[j] = (unsigned char)nb;
		pixDepths[j++] =(BYTE)dp.depth % 256;;
		pix[j] = dp.c.g();
		int ng = int((abs(dp.normal.getY()) * 255.0) + 0.5);
		pixNormals[j] = (unsigned char)ng;
		pixDepths[j++] = (BYTE)g;
		pix[j] = dp.c.r();
		int nr = int((abs(dp.normal.getX()) * 255.0) + 0.5);
		pixNormals[j] = nr;
		pixDepths[j++] = (BYTE)r;
		pix[j] = 0xff;;
		pixNormals[j] = 0xff;
		if (dp.depth != 0)
			pixDepths[j++] = 0xff;
		else
			pixDepths[j++] = 0;
		i++;
	}
	std::cout <<count << endl;
	std::ostringstream pathc;
	pathc << PointCloudUtils::ExePath() <<"\\"<< path << "\\color" << id << ".bmp";
	std::string sc = pathc.str();
	std::wstring stempc = std::wstring(sc.begin(), sc.end());
	LPCWSTR sw = stempc.c_str();

	std::ostringstream pathd;
	pathd << PointCloudUtils::ExePath() << "\\"<<path << "\\depth" << id << ".bmp";
	std::string s = pathd.str();
	std::wstring stemp = std::wstring(s.begin(), s.end());
	LPCWSTR swb = stemp.c_str();

	std::ostringstream pathn;
	pathn << PointCloudUtils::ExePath() << "\\" << path << "\\normals" << id << ".bmp";
	std::string sn = pathn.str();
	std::wstring stempn = std::wstring(sn.begin(), sn.end());
	LPCWSTR swn = stempn.c_str();

	PointCloudUtils::SaveBitmapToFile(static_cast<BYTE *>(pix), 512, 424, 32, sw);
	PointCloudUtils::SaveBitmapToFile(static_cast<BYTE *>(pixDepths), 512, 424, 32, swb);
	PointCloudUtils::SaveBitmapToFile(static_cast<BYTE *>(pixNormals), 512, 424, 32, swn);

	delete pix;
	delete pixDepths;
};


void Layer::deprojectAll(PointCloud<PointXYZRGBNormal>::Ptr cloud) {
	for(int i = 0; i < width*height; i++)
	{	
		DepthPixel dp = pixs[i];
		if (dp.depth > 0)
			cloud->push_back(deprojectPoint(dp));
	}
}

void Layer::getFrameARGBData(uint8_t* &data)
{
	int i = width*height-1;
	int j = 0;
	while (i >= 0)
	{
		data[j++] = (uint8_t)pixs[i].c.b(); 
		data[j++] = (uint8_t)pixs[i].c.g();
		data[j++] = (uint8_t)pixs[i].c.r();
		data[j++] = 0xff;
		i--;
	}
	return;
}


//  r = (f / 256) / 256  high byte
//  g = (f / 256) % 256  middle byte
//  b = f % 256          low byte
void Layer::getFrameDepthARGBData(uint8_t* &data)
{
	int i = width*height - 1;
	int j = 0;
	while (i >= 0)
	{
		double dm = floor(pixs[i].depth / 256.0);
		int g = ((int)dm) % 256;
		int r = (int)floor(dm / 256.0);
		data[j++] = (uint8_t)pixs[i].depth % 256;
		data[j++] = (uint8_t)g;
		data[j++] = (uint8_t)r;

		if (pixs[i].depth != 0)
			data[j++] = 0xff;
		else
			data[j++] = 0;
		i--;
	}
	return;
}

float Layer::medianDepth() {
	int size = width*height;
	std::vector<unsigned long> sorted;
	for (int i = 0; i < size; i++) {
		if (pixs[i].depth != 0) {
			sorted.push_back(pixs[i].depth);
		}
	}
	if (sorted.size() == 0) {
		return -1;
	}
	std::sort(sorted.begin(), sorted.end());
	float dMedian = 0.0;
	int iSize = sorted.size();
	if ((iSize % 2) == 0) {
		dMedian = (sorted[iSize / 2] + sorted[(iSize / 2) - 1]) / 2.0;
	}
	else {
		dMedian = sorted[iSize / 2];
	}
	return dMedian;
}

void Layer::getFrameDepthData(short* &data)
{
	int i = width*height - 1;
	int j = 0;
	while (i >= 0)
	{
		short depth = (short) pixs[i].depth;
		data[j++] = depth;
		i--;
	}
	return;
}


void Layer::getFrameNormalARGBData(uint8_t* &data) 
{
	int i = width*height - 1;
	int j = 0;
	while (i >= 0)
	{
		if(pcl_isfinite(pixs[i].normal.getX()) &&
			pcl_isfinite(pixs[i].normal.getY()) &&
			pcl_isfinite(pixs[i].normal.getZ()))
		{
			pixs[i].normal.normalize();
			int x = int((pixs[i].normal.getX() * 255.0)+0.5);
			int y = int((pixs[i].normal.getY() * 255.0)+0.5);
			int z = int((pixs[i].normal.getZ() * 255.0)+0.5);
			uint8_t maskx = x < 0 ? 1 : 0;
			uint8_t masky = y < 0 ? 2 : 0;
			uint8_t maskz = z < 0 ? 4 : 0;
			uint8_t mask = maskx | masky | maskz;
			data[j++] = (uint8_t)abs(z);
			data[j++] = (uint8_t)abs(y);
			data[j++] = (uint8_t)abs(x);
			data[j++] = mask;
		}
		else {
			data[j++] = 0;
			data[j++] = 0;
			data[j++] = 0;
			data[j++] = 0;
		}
		i--;
	}
	return;

}
