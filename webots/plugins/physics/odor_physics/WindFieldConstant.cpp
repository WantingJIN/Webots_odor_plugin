// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <iostream>
#include "WindFieldConstant.h"
#define	THISCLASS WindFieldConstant

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<WindFieldConstant>" << std::endl;
	out << "\t<WindSpeed>" << mWindSpeed << "</WindSpeed>" << std::endl;
	out << "</WindFieldConstant>" << std::endl;
}
