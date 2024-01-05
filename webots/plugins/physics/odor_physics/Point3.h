// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classPoint3
#define classPoint3

class Point3;
class Point3Int;

#include <cmath>
#include "DataFileReader.h"
#include "DataFileWriter.h"

/*!
		3d Point3 (double).
*/
class Point3 {

public:
	double x, y, z;

	Point3(): x(0), y(0), z(0) {}
	Point3(double px, double py, double pz): x(px), y(py), z(pz) {}
	Point3(const Point3Int &p);

	double Length() const {
		return sqrt(x*x + y*y + z*z);
	}
	double Length2() const {
		return x*x + y*y + z*z;
	}
	double Volume() const {
		return x*y*z;
	}
	double Distance(const Point3 &p) const {
		return sqrt((x -p.x)*(x - p.x) + (y - p.y)*(y - p.y) + (z - p.z)*(z - p.z));
	}
	double Distance2(const Point3 &p) const {
		return (x -p.x)*(x - p.x) + (y - p.y)*(y - p.y) + (z - p.z)*(z - p.z);
	}

	Point3 Floor() const {
		return Point3(floor(x), floor(y), floor(z));
	}
	Point3 Ceil() const {
		return Point3(ceil(x), ceil(y), ceil(z));
	}
	Point3 Round() const {
		return Point3(floor(x + 0.5), floor(y + 0.5), floor(z + 0.5));
	}
	Point3 Abs() const {
		return Point3(sqrt(x * x), sqrt(y * y), sqrt(z * z));
	}
	Point3 Move(const Point3 &p) const {
		return Point3(x + p.x, y + p.y, z + p.z);
	}
	Point3 Move(double sx, double sy, double sz) const {
		return Point3(x + sx, y + sy, z + sz);
	}
	Point3 DotMultiply(const Point3 &p) const {
		return Point3(x*p.x, y*p.y, z*p.z);
	}
	Point3 DotMultiply(double sx, double sy, double sz) const {
		return Point3(x*sx, y*sy, z*sz);
	}
	Point3 DotDivide(const Point3 &p) const {
		return Point3(x / p.x, y / p.y, z / p.z);
	}
	Point3 DotDivide(double sx, double sy, double sz) const {
		return Point3(x / sx, y / sy, z / sz);
	}
	Point3 DotModulo(const Point3 &p) const {
		return Point3(fmod(x, p.x), fmod(y, p.y), fmod(z, p.z));
	}
	Point3 DotInv() const {
		return Point3(1 / x, 1 / y, 1 / z);
	}

	Point3 operator - () const {
		return Point3(-x, -y, -z);
	}
	Point3 operator + (const Point3 &p) const {
		return Point3(x + p.x, y + p.y, z + p.z);
	}
	Point3 operator - (const Point3 &p) const {
		return Point3(x -p.x, y - p.y, z - p.z);
	}
	Point3 operator * (double a) const {
		return Point3(x*a, y*a, z*a);
	}
	Point3 operator / (double a) const {
		return Point3(x / a, y / a, z / a);
	}
	double operator * (const Point3 &p) const {
		return x*p.x + y*p.y + z*p.z;
	}

	void operator += (const Point3 &p) {
		x += p.x;
		y += p.y;
		z += p.z;
	}
	void operator -= (const Point3 &p) {
		x -= p.x;
		y -= p.y;
		z -= p.z;
	}
	void operator *= (double a) {
		x *= a;
		y *= a;
		z *= a;
	}
	void operator /= (double a) {
		x /= a;
		y /= a;
		z /= a;
	}

	bool operator == (const Point3 &p) const {
		return (x == p.x) && (y == p.y) && (z == p.z);
	}

	void Read(DataFileReader &f);
	void Write(DataFileWriter &f);

	friend std::ostream &operator<<(std::ostream &stream, const Point3 &point);
};

inline Point3 Floor(const Point3 &p) {
	return p.Floor();
}
inline Point3 Ceil(const Point3 &p) {
	return p.Ceil();
}
inline Point3 Round(const Point3 &p) {
	return p.Round();
}

#endif
