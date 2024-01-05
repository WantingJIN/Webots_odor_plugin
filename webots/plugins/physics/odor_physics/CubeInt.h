// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#ifndef classCubeInt
#define classCubeInt

class CubeInt;
class Cube;

#include "Point3Int.h"
#include "DataFileReader.h"
#include "DataFileWriter.h"

/*!
		3d Cube (int).
*/
class CubeInt {
public:
	Point3Int a;
	Point3Int b;

	CubeInt();
	CubeInt(int x1, int y1, int x2, int y2);
	CubeInt(int x1, int y1, int z1, int x2, int y2, int z2);
	CubeInt(const Point3Int &p);
	CubeInt(const Point3Int &p1, const Point3Int &p2);
	CubeInt(const Cube &c);

	Point3Int Size() const {
		return b -a;
	}
	bool IsEmpty() const {
		return (a.x >= b.x) || (a.y >= b.y) || (a.z >= b.z);
	}
	int Volume() const {
		return (b.x -a.x)*(b.y - a.y)*(b.z - a.z);
	}
	int Area() const {
		return 2*((b.x - a.x)*(b.y - a.y) + (b.x - a.x)*(b.z - a.z) + (b.y - a.y)*(b.z - a.z));
	}
	int AreaXY() const {
		return (b.x -a.x)*(b.y - a.y);
	}
	int AreaXZ() const {
		return (b.x -a.x)*(b.z - a.z);
	}
	int AreaYZ() const {
		return (b.y -a.y)*(b.z - a.z);
	}

	void Include(const CubeInt &c);
	void Include(const Point3Int &c);
	void Intersect(const CubeInt &c);

	bool Inside(const Point3Int &c) const {
		return ((c.x >= a.x) && (c.y >= a.y) && (c.z >= a.z) && (c.x < b.x) && (c.y < b.y) && (c.z < b.z));
	}
	bool InsideXY(const Point3Int &c) const {
		return ((c.x >= a.x) && (c.y >= a.y) && (c.x < b.x) && (c.y < b.y));
	}
	bool InsideXZ(const Point3Int &c) const {
		return ((c.x >= a.x) && (c.z >= a.z) && (c.x < b.x) && (c.z < b.z));
	}
	bool InsideYZ(const Point3Int &c) const {
		return ((c.y >= a.y) && (c.z >= a.z) && (c.y < b.y) && (c.z < b.z));
	}

	CubeInt Move(const Point3Int &p) const {
		return CubeInt(a.Move(p), b.Move(p));
	}
	CubeInt Move(int sx, int sy = 0, int sz = 0) const {
		return CubeInt(a.Move(sx, sy, sz), b.Move(sx, sy, sz));
	}

	CubeInt operator + (const Point3Int &p) const {
		return CubeInt(a + p, b + p);
	}
	CubeInt operator - (const Point3Int &p) const {
		return CubeInt(a -p, b - p);
	}

	void Read(DataFileReader &f);
	void Write(DataFileWriter &f);

	friend std::ostream &operator<<(std::ostream &stream, const CubeInt &cube);
};

#endif
