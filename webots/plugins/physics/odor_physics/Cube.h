// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classCube
#define classCube

class Cube;
class CubeInt;

#include <iostream>
#include "Point3.h"
#include "DataFileReader.h"
#include "DataFileWriter.h"

/*!
	3d Cube (double).
*/
class Cube {
public:
	Point3 a;
	Point3 b;

	Cube();
	Cube(double x1, double y1, double z1, double x2, double y2, double z2);
	Cube(const Point3 &p);
	Cube(const Point3 &p1, const Point3 &p2);
	Cube(const CubeInt &c);

	double MiddleX(double scale = 0.5) {
		return a.x + (b.x - a.x)*scale;
	}
	double MiddleY(double scale = 0.5) {
		return a.y + (b.y - a.y)*scale;
	}
	double MiddleZ(double scale = 0.5) {
		return a.z + (b.z - a.z)*scale;
	}
	Point3 Middle(double scalex = 0.5, double scaley = 0.5, double scalez = 0.5) {
		return Point3(a.x + (b.x - a.x)*scalex, a.y + (b.y - a.y)*scaley, a.z + (b.z - a.z)*scalez);
	}

	Point3 Size() const {
		return b -a;
	}
	bool IsEmpty() const {
		return (a.x >= b.x) || (a.y >= b.y) || (a.z >= b.z);
	}
	double Volume() const {
		return (b.x -a.x)*(b.y - a.y)*(b.z - a.z);
	}
	double Area() const {
		return 2*((b.x - a.x)*(b.y - a.y) + (b.x - a.x)*(b.z - a.z) + (b.y - a.y)*(b.z - a.z));
	}
	double AreaXY() const {
		return (b.x -a.x)*(b.y - a.y);
	}
	double AreaXZ() const {
		return (b.x -a.x)*(b.z - a.z);
	}
	double AreaYZ() const {
		return (b.y -a.y)*(b.z - a.z);
	}

	void Include(const Cube &c);
	void Include(const Point3 &c);
	void Intersect(const Cube &c);

	bool Inside(const Point3 &c) const {
		return ((c.x >= a.x) && (c.y >= a.y) && (c.z >= a.z) && (c.x < b.x) && (c.y < b.y) && (c.z < b.z));
	}
	bool InsideXY(const Point3 &c) const {
		return ((c.x >= a.x) && (c.y >= a.y) && (c.x < b.x) && (c.y < b.y));
	}
	bool InsideXZ(const Point3 &c) const {
		return ((c.x >= a.x) && (c.z >= a.z) && (c.x < b.x) && (c.z < b.z));
	}
	bool InsideYZ(const Point3 &c) const {
		return ((c.y >= a.y) && (c.z >= a.z) && (c.y < b.y) && (c.z < b.z));
	}

	Cube Move(const Point3 &p) const {
		return Cube(a.Move(p), b.Move(p));
	}
	Cube Move(double sx, double sy = 0, double sz = 0) const {
		return Cube(a.Move(sx, sy, sz), b.Move(sx, sy, sz));
	}
	Cube Scale(const Point3 &p) const {
		return Cube(a.DotMultiply(p), b.DotMultiply(p));
	}
	Cube Scale(double sx, double sy = 1, double sz = 1) const {
		return Cube(a.DotMultiply(sx, sy, sz), b.DotMultiply(sx, sy, sz));
	}

	Cube operator + (const Point3 &p) const {
		return Cube(a + p, b + p);
	}
	Cube operator - (const Point3 &p) const {
		return Cube(a -p, b - p);
	}
	Cube operator * (double s) const {
		return Cube(a*s, b*s);
	}
	Cube operator / (double s) const {
		return Cube(a / s, b / s);
	}

	void Read(DataFileReader &f);
	void Write(DataFileWriter &f);

	friend std::ostream &operator<<(std::ostream &stream, const Cube &cube);
};

#endif
