// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "WindFieldStatic.h"
#include "TextFileReaderOpenFOAMSamples.h"
#define	THISCLASS WindFieldStatic

bool THISCLASS::SetFile(const std::string &file) {
	/* //Thomas
	TextFileReaderOpenFOAMSamples tfr(file);
	if (tfr.Error()) {
		return false;
	}
	tfr.Read(&mWindFieldSnapshot);
	return true;
	*/

	//Fa
	int size = 0;
	int counter = 0;

	//open the file
	//FILE * pFile = fopen (file.c_str(),"r");
	std::ifstream mFile;
	mFile.open(file.c_str(), std::ios::in);
	// if the file exists
	if(mFile.is_open()){
		//extract the size and allocate memory
		std::string line;	
		std::getline(mFile, line);
		std::istringstream iss(line);
		iss >> size;
		mWindFieldSnapshot.AllocateArray(size);

		// read the file line by line
		while (std::getline(mFile, line)){
			//std::cout << "line " << line << std::endl;
			
			// read one line
			std::istringstream iss(line);
			//std::cout << iss.str() << std::endl;

			// seperate x, y and z
			double a, b ,c;
    		if (!(iss >> a >> b >> c)) 
    			continue; // in case of useless lines
    		
    		// x
    		mWindFieldSnapshot.mWind[counter].x = a; 
    		// y
    		mWindFieldSnapshot.mWind[counter].y = b;
    		// z
    		mWindFieldSnapshot.mWind[counter].z = c;
    		
    		// std::cout << "x=" << mWindFieldSnapshot.mWind[counter].x 
    		// 			<< " y=" << mWindFieldSnapshot.mWind[counter].y 
    		// 			<< " z=" << mWindFieldSnapshot.mWind[counter].z << std::endl;
    		counter++;
		}
    } else{
    	std::cout << "unable to open the file " << std::endl;
    	return false;
    }

    // make sure all of the elements are read
    if(counter == size)
    	return true;
    else{
    	std::cout << "Number of elements in file "<< file <<" incorrect" << std::endl;
    	return false;
    }
    //Fa
}

Point3 THISCLASS::GetWindSpeed(const Point3 &preal) {
	//return mWindFieldSnapshot.GetWindSpeed(preal);
	//printf("Inside WindFieldStatic::GetWindSpeed\n");

	//Fa
	int index;
	int nbins[3] = {100,64,19};						//HARD CODED /!\ //
	float resolution[3] = {0.1586,0.0632,0.095};		//HARD CODED /!\ //
	
	//convert position to indices.
	int indices[3];
	indices[0] = (int)(preal.x / resolution[0]); 
	indices[1] = (int)(preal.y / resolution[1]); 
	indices[2] = (int)(preal.z / resolution[2]); 
	
	//convert indices to single index
	index = indices[1] + indices[0]*nbins[1] + indices[2]*nbins[1]*nbins[0];
	//if the index is not incorrect, return the corresponding wind value
	if(index >= nbins[0] * nbins[1] * nbins[2] || index < 0){
		// std::cout << "Dangerous ERROR in WindFieldStatic::GetWindSpeed. index >= " <<  mWindFieldSnapshot.mArraySize.x * mWindFieldSnapshot.mArraySize.y * mWindFieldSnapshot.mArraySize.z << std::endl;
		// std::cout << "position (" << preal << ") -> indices " << indices[0] << " " <<indices[1] << " " << indices[2] << " index " << index << std::endl;
		return Point3(-100, -100, -100);
	}else{
		//printf("position %f %f %f, indices %d %d %d, index %d, wind %f %f %f\n", preal.x, preal.y, preal.z, indices[0], indices[1], indices[2], index, mWindFieldSnapshot.mWind[index].x, mWindFieldSnapshot.mWind[index].y, mWindFieldSnapshot.mWind[index].z);
		return mWindFieldSnapshot.mWind[index];
	}
	//Fa
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	mWindFieldSnapshot.WriteConfiguration(out);
}
