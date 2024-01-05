// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "CubeInt.h"
#include "Cube.h"
#define THIS CubeInt

THIS::CubeInt() {
	a = Point3Int(0, 0, 0);
	b = Point3Int(0, 0, 0);
}

THIS::CubeInt(int x1, int y1, int z1, int x2, int y2, int z2) {
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

THIS::CubeInt(const Point3Int &p) {
	a = p;
	b = p;
}

THIS::CubeInt(const Point3Int &p1, const Point3Int &p2) {
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

THIS::CubeInt(const Cube &c) {
	a.x = (int)floor(c.a.x);
	a.y = (int)floor(c.a.y);
	a.z = (int)floor(c.a.z);
	b.x = (int)ceil(c.b.x);
	b.y = (int)ceil(c.b.y);
	b.z = (int)ceil(c.b.z);
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

void THIS::Include(const Point3Int &p) {
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

std::ostream &operator<<(std::ostream &stream, const CubeInt &cube) {
	return stream << cube.a << " - " << cube.b;
}
