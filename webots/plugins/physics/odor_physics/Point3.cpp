// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "Point3.h"
#include "Point3Int.h"
#define THIS Point3

THIS::Point3(const Point3Int &p) {
	x = (double)p.x;
	y = (double)p.y;
	z = (double)p.z;
}

void THIS::Read(DataFileReader &f) {
	x = f.Double();
	y = f.Double();
	z = f.Double();
}

void THIS::Write(DataFileWriter &f) {
	f.Double(x);
	f.Double(y);
	f.Double(z);
}

std::ostream &operator<<(std::ostream &stream, const Point3 &point) {
	return stream << "(" << point.x << ", " << point.y << ", " << point.z << ")";
}
