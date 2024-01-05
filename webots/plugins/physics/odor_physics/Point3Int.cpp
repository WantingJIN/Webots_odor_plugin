// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "Point3Int.h"
#include "Point3.h"
#define THIS Point3Int

THIS::Point3Int(const Point3 &p) {
	x = (int)floor(p.x + 0.5);
	y = (int)floor(p.y + 0.5);
	z = (int)floor(p.z + 0.5);
}

void THIS::Read(DataFileReader &f) {
	x = f.Int();
	y = f.Int();
	z = f.Int();
}

void THIS::Write(DataFileWriter &f) {
	f.Int(x);
	f.Int(y);
	f.Int(z);
}

std::ostream &operator<<(std::ostream &stream, const Point3Int &point) {
	return stream << "(" << point.x << ", " << point.y << ", " << point.z << ")";
}
