// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classPoint3Int
#define classPoint3Int

class Point3Int;
class Point3;

#include <cmath>
#include "DataFileReader.h"
#include "DataFileWriter.h"

/*!
		3d Point3 (int).
*/
class Point3Int {

public:
	int x, y, z;

	Point3Int(): x(0), y(0), z(0) {}
	Point3Int(int px, int py, int pz): x(px), y(py), z(pz) {}
	Point3Int(const Point3 &p);

	double Length() const {
		return sqrt((double)(x*x + y*y + z*z));
	}
	int Length2() const {
		return x*x + y*y + z*z;
	}
	int Volume() const {
		return x*y*z;
	}
	double Distance(const Point3Int &p) const {
		return sqrt((x -p.x)*(x - p.x) + (y - p.y)*(y - p.y) + (z - p.z)*(z - p.z));
	}
	int Distance2(const Point3Int &p) const {
		return (x -p.x)*(x - p.x) + (y - p.y)*(y - p.y) + (z - p.z)*(z - p.z);
	}

	Point3Int Move(const Point3Int &p) const {
		return Point3Int(x + p.x, y + p.y, z + p.z);
	}
	Point3Int Move(int sx, int sy, int sz) const {
		return Point3Int(x + sx, y + sy, z + sz);
	}
	Point3Int DotMultiply(const Point3Int &p) const {
		return Point3Int(x*p.x, y*p.y, z*p.z);
	}
	Point3Int DotMultiply(int sx, int sy, int sz) const {
		return Point3Int(x*sx, y*sy, z*sz);
	}
	Point3Int DotDivide(const Point3Int &p) const {
		return Point3Int(x / p.x, y / p.y, z / p.z);
	}
	Point3Int DotDivide(int sx, int sy, int sz) const {
		return Point3Int(x / sx, y / sy, z / sz);
	}

	Point3Int operator - () const {
		return Point3Int(-x, -y, -z);
	}
	Point3Int operator + (const Point3Int &p) const {
		return Point3Int(x + p.x, y + p.y, z + p.z);
	}
	Point3Int operator - (const Point3Int &p) const {
		return Point3Int(x -p.x, y - p.y, z - p.z);
	}
	Point3Int operator * (int a) const {
		return Point3Int(x*a, y*a, z*a);
	}
	Point3Int operator / (int a) const {
		return Point3Int(x / a, y / a, z / a);
	}
	double operator * (const Point3Int &p) const {
		return x*p.x + y*p.y + z*p.z;
	}

	void operator += (const Point3Int &p) {
		x += p.x;
		y += p.y;
		z += p.z;
	}
	void operator -= (const Point3Int &p) {
		x -= p.x;
		y -= p.y;
		z -= p.z;
	}
	void operator *= (int a) {
		x *= a;
		y *= a;
		z *= a;
	}
	void operator /= (int a) {
		x /= a;
		y /= a;
		z /= a;
	}

	bool operator == (const Point3Int &p) const {
		return (x == p.x) && (y == p.y) && (z == p.z);
	}

	void Read(DataFileReader &f);
	void Write(DataFileWriter &f);

	friend std::ostream &operator<<(std::ostream &stream, const Point3Int &point);
};

#endif
