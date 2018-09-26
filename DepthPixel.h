
#include "Color.h"
#include "Vector3.h"

class DepthPixel{

public:
	unsigned long depth;
	Color c;
	Vector3 normal;
	int u;
	int v;

	 DepthPixel(){
		depth = 0;
		normal = Vector3(0, 0, 0);
	}
	DepthPixel(int x, int y, long dep, Color col, Vector3 norm){
		u = x; v = y; depth = dep; c = col; normal = norm;
	};
	~DepthPixel(){
	};


};

