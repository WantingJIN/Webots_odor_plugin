// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "TextFileReaderOpenFOAMSamples.h"
#define THISCLASS TextFileReaderOpenFOAMSamples

#include <sstream>
#include <fstream>
#include <string>

THISCLASS::TextFileReaderOpenFOAMSamples(const std::string &filename):
		TextFileReader(filename) {
}


void THISCLASS::Read(WindFieldSnapshot *wfs) {
//std::cout << "TextFileReaderOpenFOAMSamples Read" << std::endl;
//Fa
	Start();
	int counter = 0;

	if(mFile.is_open()){
		// read the file line by line
		
		std::string line;	
		while (std::getline(mFile, line)){
			//std::cout << "line " << line << std::endl;
		
			// the ')' at the begining of the line means the end of the file
			if (line[0]==')')
				break;
			
			// the '(' at the begining of the line doesnt let to extract the double form string
			if (line[0]=='(')
				line.erase(line.begin()); 
			
			// seperate x, y and z
			std::istringstream iss(line);
			//std::cout << iss.str() << std::endl;
			double a, b ,c;
    		if (!(iss >> a >> b >> c)) 
    			continue; // in case of useless lines
    		
    		// x
    		wfs->mWind[counter].x = a; 
    		// y
    		wfs->mWind[counter].y = b;
    		// z
    		wfs->mWind[counter].z = c;
    		
    		/*std::cout << "x=" << wfs->mWind[counter].x 
    					<< " y=" << wfs->mWind[counter].y 
    					<< " z=" << wfs->mWind[counter].z << std::endl;*/
    		counter++;
		}
    } else
    	std::cout << "unable to open the file " << std::endl;
    
//std::cout << "TextFileReaderOpenFOAMSamples Read end" << std::endl;
}
