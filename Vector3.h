#pragma once
#include <math.h>
class Vector3
{
protected:
	double _x;
	double _y;
	double _z;

public:
	Vector3(){};
	Vector3(double x, double y, double z){
		_x = x; _y = y; _z = z;
	}

	inline double getX() const { return _x; }
	inline double getY() const { return _y; }
	inline double getZ() const { return _z; }
	inline void set(double x, double y, double z){
		_x = x; _y = y; _z = z;
	}
	inline void normalize(){
		double length = sqrt((_x * _x) + (_y * _y) + (_z * _z));
		_x = _x / length;
		_y = _y / length;
		_z = _z / length;
	}
	void operator=(const Vector3& b){
		set(b.getX(), b.getY(), b.getZ());
	}
	inline Vector3 operator*(double num){
		Vector3 res(getX() *num, getY() *num, getZ() *num);
		return res;
	}
	inline Vector3 operator+(const Vector3& b){
		Vector3 res(getX() + b.getX(), getY() + b.getY(), getZ() + b.getZ());
		return res;
	}
	inline Vector3 operator-(const Vector3& b){
		getX();
		Vector3 res(getX() - b.getX(), getY() - b.getY(), getZ() - b.getZ());
		return res;
	}
	


	~Vector3(){};
};

