// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "Cube.h"
#include "CubeInt.h"
#define THIS Cube

THIS::Cube() {
	a = Point3(0, 0, 0);
	b = Point3(0, 0, 0);
}

THIS::Cube(double x1, double y1, double z1, double x2, double y2, double z2) {
	if (x1 < x2) {
		a.x = x1;
		b.x = x2;
	} else {
		a.x = x2;
		b.x = x1;
	}
	if (y1 < y2) {
		a.y = y1;
		b.y = y2;
	} else {
		a.y = y2;
		b.y = y1;
	}
	if (z1 < z2) {
		a.z = z1;
		b.z = z2;
	} else {
		a.z = z2;
		b.z = z1;
	}
}

THIS::Cube(const Point3 &p) {
	a = p;
	b = p;
}

THIS::Cube(const Point3 &p1, const Point3 &p2) {
	if (p1.x < p2.x) {
		a.x = p1.x;
		b.x = p2.x;
	} else {
		a.x = p2.x;
		b.x = p1.x;
	}
	if (p1.y < p2.y) {
		a.y = p1.y;
		b.y = p2.y;
	} else {
		a.y = p2.y;
		b.y = p1.y;
	}
	if (p1.z < p2.z) {
		a.z = p1.z;
		b.z = p2.z;
	} else {
		a.z = p2.z;
		b.z = p1.z;
	}
}

THIS::Cube(const CubeInt &c) {
	a = Point3(c.a);
	b = Point3(c.b);
}

void THIS::Include(const THIS &c) {
	if (c.IsEmpty()) {
		return;
	}
	if (this->IsEmpty()) {
		a = c.a;
		b = c.b;
	} else {
		if (c.a.x < a.x) {
			a.x = c.a.x;
		}
		if (c.a.y < a.y) {
			a.y = c.a.y;
		}
		if (c.a.z < a.z) {
			a.z = c.a.z;
		}
		if (c.b.x > b.x) {
			b.x = c.b.x;
		}
		if (c.b.y > b.y) {
			b.y = c.b.y;
		}
		if (c.b.z > b.z) {
			b.z = c.b.z;
		}
	}
}

void THIS::Include(const Point3 &p) {
	if (p.x < a.x) {
		a.x = p.x;
	}
	if (p.y < a.y) {
		a.y = p.y;
	}
	if (p.z < a.z) {
		a.z = p.z;
	}
	if (p.x > b.x) {
		b.x = p.x;
	}
	if (p.y > b.y) {
		b.y = p.y;
	}
	if (p.z > b.z) {
		b.z = p.z;
	}
}

void THIS::Intersect(const THIS &c) {
	if (this->IsEmpty()) {
		return;
	}
	if (c.IsEmpty()) {
		a = b;
	} else {
		if (c.a.x > a.x) {
			a.x = c.a.x;
		}
		if (c.a.y > a.y) {
			a.y = c.a.y;
		}
		if (c.a.z > a.z) {
			a.z = c.a.z;
		}
		if (c.b.x < b.x) {
			b.x = c.b.x;
		}
		if (c.b.y < b.y) {
			b.y = c.b.y;
		}
		if (c.b.z < b.z) {
			b.z = c.b.z;
		}
		if (this->IsEmpty()) {
			a = b;
		}
	}
}

void THIS::Read(DataFileReader &f) {
	a.Read(f);
	b.Read(f);
}

void THIS::Write(DataFileWriter &f) {
	a.Write(f);
	b.Write(f);
}

std::ostream &operator<<(std::ostream &stream, const Cube &cube) {
	return stream << cube.a << " - " << cube.b;
}
