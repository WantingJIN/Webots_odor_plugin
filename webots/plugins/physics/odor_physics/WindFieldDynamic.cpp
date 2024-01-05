// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <sys/types.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include <math.h> 
#include "WindFieldDynamic.h"
#include "TextFileReaderOpenFOAMSamples.h"
#define	THISCLASS WindFieldDynamic

THISCLASS::WindFieldDynamic(Simulation *sim): WindField(sim) {
	mWindFieldSnapshot[0] = new WindFieldSnapshot();
	mWindFieldSnapshot[1] = new WindFieldSnapshot();
	
}

// mini function for sort function used in OnSimulationStart()
bool wayToSort(double i, double j) { return i > j; }

void THISCLASS::OnSimulationStart() {
	mFileHeapCount = 0;

	// Open folder
	DIR *d = opendir(mSamplesFolder.c_str());
	if (! d) {
		AddError("Unable to access samples folder! Is the path correct?");
		return;
	}

	// Reinitialize the heap
	for (int i = 0; i < mFileHeapMax; i++) {
		mFileHeap[i] = 0;
	}
	std::make_heap(mFileHeap, mFileHeap);

	// Read folder items
	while (struct dirent *entry = readdir(d)) {
		if ((entry->d_type == DT_DIR) && (entry->d_name[0] != '.')) {
			
			double val;
			std::string str(entry->d_name);
			std::istringstream istr(str);
			// TODO catch errors, i.e. ignore all subfolders that are not numbers
			istr >> val;
			
			mFileHeap[mFileHeapCount] = val;
			mFileHeapCount++;
			std::push_heap(mFileHeap, mFileHeap + mFileHeapCount);
		}
	}
	
	std::sort(mFileHeap, mFileHeap+mFileHeapCount, wayToSort);
	/*std::cout << "begin............" << std::endl;
	for (int i = mFileHeapCount -1; i >= 0; i--) 
    	std::cout << mFileHeap[i] << "	"<< i<< std::endl;
   	std::cout << "done............" << std::endl;
  	std::cout << "mFileHeapCount " << mFileHeapCount << std::endl;
 */
	// Close folder
	closedir(d);
	
	//Fa
	windSnapshotMemoryAllocation(mWindFieldSnapshot[0]);
	//windSnapshotMemoryAllocation(mWindFieldSnapshot[1]);
	mWindFieldSnapshot[1]->WindFieldSnapshotCopy(*mWindFieldSnapshot[0]);

	// Read the two first wind fields
	ReadNextFile();
	ReadNextFile();
}

void THISCLASS::OnSimulationEnd() {
}

void THISCLASS::OnSimulationStep() {
//std::cout << "OnSimulationStep START" << std::endl;
	// Read next file if necessary
	//std::cout << "mSimulation->mSimulationTime " << mSimulation->mSimulationTime << std::endl;
//	std::cout << "mWindFieldSnapshot[1]->mTime " << mWindFieldSnapshot[1]->mTime << std::endl;
	//std::cout << "mWindFieldSnapshot[0]->mTime " << mWindFieldSnapshot[0]->mTime << std::endl;
	while (mSimulation->mSimulationTime > mWindFieldSnapshot[1]->mTime) {
		bool ok = ReadNextFile();
		if (! ok) {
			AddError("At end of wind simulation!");
			return;
		}
	}

	mTimeInterpolationFactor = (mSimulation->mSimulationTime - mWindFieldSnapshot[0]->mTime) / (mWindFieldSnapshot[1]->mTime - mWindFieldSnapshot[0]->mTime);
	/*std::cout << "mTimeInterpolationFactor = (" << mSimulation->mSimulationTime 
				<< " - " << mWindFieldSnapshot[0]->mTime << ") / (" 
				<< mWindFieldSnapshot[1]->mTime << " - " << mWindFieldSnapshot[0]->mTime 
				<< ") = " << mTimeInterpolationFactor << std::endl;*/
//std::cout << "OnSimulationStep END" << std::endl;
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	mWindFieldSnapshot[0]->WriteConfiguration(out);
	mWindFieldSnapshot[1]->WriteConfiguration(out);
	//std::cout << "windFieldDynamic line 94" << std::endl;
}

Point3 THISCLASS::GetWindSpeed(const Point3 &preal) {
//std::cout << "windFieldDynamic line 96" << std::endl;
	Point3 w0 = mWindFieldSnapshot[0]->GetWindSpeed(preal);
	Point3 w1 = mWindFieldSnapshot[1]->GetWindSpeed(preal);
	//std::cout << "w0 " << w0 << ", w1 " << w1<< std::endl;
//std::cout << "windFieldDynamic line 100" << std::endl;
	return w0*mTimeInterpolationFactor + w1*(1 - mTimeInterpolationFactor);
}

bool THISCLASS::ReadNextFile() {
//std::cout << "WindFieldDynamic ReadNextFile" << std::endl;
	// Switch the two wind fields
	WindFieldSnapshot *temp = mWindFieldSnapshot[1];
	mWindFieldSnapshot[1] = mWindFieldSnapshot[0];
	mWindFieldSnapshot[0] = temp;
	std::cout << mFileHeapCount << std::endl;
	// If there are no more files on the heap, stop the simulation
	if (mFileHeapCount <= 0) {
		return false;
	}

	// Get the next file name
	/*
	std::pop_heap(mFileHeap, mFileHeap + mFileHeapCount);
std::cout << "begin............2" << std::endl;
	for (int i = mFileHeapCount - 1; i >= 0; i--) 
    	std::cout << mFileHeap[i] <<"	"<< i<<  std::endl;
std::cout << "done............2" << std::endl;
	*/
	mFileHeapCount--;
	double time = mFileHeap[mFileHeapCount];
	mFileHeap[mFileHeapCount] = 0;


	std::ostringstream file;
	//file << mSamplesFolder << "/" << time << "/velfield_xy";
	file << mSamplesFolder << "/" << time << "/U";

	// bad idea Fa
	mWindFieldSnapshot[0]->mTime = time;
	//std::cout << "Time " << time << std::endl;
	
	// Read the next file
	TextFileReaderOpenFOAMSamples tfr(file.str());
	//std::cout << "TextFileReaderOpenFOAMSamples constructor" << std::endl;
	//if (tfr.Error()) {return false;}	//Error() method doesn't existe yet TODO
	tfr.Read(mWindFieldSnapshot[0]);
	
//std::cout << "WindFieldDynamic ReadNextFile end" << std::endl;	
	return true;
}


// read the "cellcentres" file to have the number of points to allocate memory as well as the centre of cells 
void THISCLASS::windSnapshotMemoryAllocation(WindFieldSnapshot *wfs) {	//Fa
//std::cout << "WindFieldDynamic windSnapshotMemoryAllocation" << std::endl;
	
	std::string line, prevLine;
	bool start = false;
	int size = 0, ctrPoint = 0;
	Point3 * mCellCentrestmp = NULL;
	Point3 newpoint, oldValue(-1e7,-1e7,-1e7), diff;

	wfs->mCellCentres = (Point3*) malloc (sizeof(double));
	wfs->mOrigin = Point3(1e7,1e7,1e7);
	wfs->mEnd = Point3(-1e7,-1e7,-1e7);
	wfs->mGridSize = Point3(1e8,1e8,1e8);
	wfs->SetArraySize(Point3Int(0,0,0));
	
	//open the file (no need to use open())
	std::ostringstream pointsFile;
	pointsFile << mSamplesFolder << "/0/cellCentres";
	std::ifstream infile(pointsFile.str());
	
	if(infile.is_open()){
	
		// read the file line by line
		while (std::getline(infile, line)){
			
			// the ')' at the begining of the line means the end of the file
			if (line[0]==')')
				break;
			
			// the '(' at the begining of the line means the begining of the data
			if (start == false) {
				if (line[0]=='('){
					start = true; 
					size = atoi(prevLine.c_str());
					std::cout << "size " << size << std::endl;
				} else
					prevLine = line;
				continue;
			}
			
			// we are between ( and ), it means we are reading the data
			if (line[0]=='(')
				line.erase (line.begin()); 
			
			// Memory reallocation
			ctrPoint++;
			mCellCentrestmp = (Point3*) realloc (wfs->mCellCentres, ctrPoint * sizeof(Point3));
			if(mCellCentrestmp != NULL){
    			wfs->mCellCentres = mCellCentrestmp;
    			wfs->mCellCentres[ctrPoint-1] = newpoint;
    		} else {
    			free(wfs->mCellCentres);
    			std::cout << "Error reallocating memory" << std::endl;
    			exit (1);
    		}
    		// seperate x, y and z
			std::istringstream iss(line);
    		if (!(iss >> wfs->mCellCentres[ctrPoint-1].x >> wfs->mCellCentres[ctrPoint-1].y >> wfs->mCellCentres[ctrPoint-1].z)) 
    			continue; // in case of useless lines (but it shouldn't happend!)
    		//std::cout << "point counter : " << ctrPoint << std::endl;
    		/*std::cout << wfs->mCellCentres[ctrPoint-1].x << " " 
    					<< wfs->mCellCentres[ctrPoint-1].y << " "
    					<< wfs->mCellCentres[ctrPoint-1].z << std::endl;*/
    		
    		// set the size of the measurement area
    		if (wfs->mOrigin.x > wfs->mCellCentres[ctrPoint-1].x)
    			wfs->mOrigin.x = wfs->mCellCentres[ctrPoint-1].x;
    		if (wfs->mOrigin.y > wfs->mCellCentres[ctrPoint-1].y)
    			wfs->mOrigin.y = wfs->mCellCentres[ctrPoint-1].y;
    		if (wfs->mOrigin.z > wfs->mCellCentres[ctrPoint-1].z)
    			wfs->mOrigin.z = wfs->mCellCentres[ctrPoint-1].z;
    		if (wfs->mEnd.x < wfs->mCellCentres[ctrPoint-1].x)
    			wfs->mEnd.x = wfs->mCellCentres[ctrPoint-1].x;
    		if (wfs->mEnd.y < wfs->mCellCentres[ctrPoint-1].y)
    			wfs->mEnd.y = wfs->mCellCentres[ctrPoint-1].y;
    		if (wfs->mEnd.z < wfs->mCellCentres[ctrPoint-1].z)
    			wfs->mEnd.z = wfs->mCellCentres[ctrPoint-1].z;
    		
    		// set the gridsize
    		diff = wfs->mCellCentres[ctrPoint-1] - oldValue;
    		if (fabs(diff.x) > 0) {
    			oldValue.x = wfs->mCellCentres[ctrPoint-1].x;
    			if ((fabs(diff.x) < wfs->mGridSize.x) && (fabs(diff.x) > 0.001))
    				wfs->mGridSize.x = fabs(diff.x);
    		}
    		if (fabs(diff.y) > 0) {
    			oldValue.y = wfs->mCellCentres[ctrPoint-1].y;
    			if ((fabs(diff.y) < wfs->mGridSize.y) && (fabs(diff.y) > 0.001))
    				wfs->mGridSize.y = fabs(diff.y);
    		}
    		if (fabs(diff.z) > 0) {
    			oldValue.z = wfs->mCellCentres[ctrPoint-1].z;
    			if ((fabs(diff.z) < wfs->mGridSize.z) && (fabs(diff.z) > 0.001))
    				wfs->mGridSize.z = fabs(diff.z);
    		}
		}
    } else
    	std::cout << "unable to open the file " << pointsFile.str() << std::endl;

	// close the input file (not sure if necessary)
    infile.close();
    
    // number of points on each axis
    wfs->mArraySize = (wfs->mEnd - wfs->mOrigin + wfs->mGridSize).DotDivide(wfs->mGridSize);
    
    // Memory Allocation for wind snapshot
    wfs->AllocateArray(ctrPoint);
    wfs->mCellNbr = ctrPoint;
    
    
    std::cout<< "mGridSize: " << wfs->mGridSize << std::endl;
std::cout<< "mOrigin: " << wfs->mOrigin << std::endl;
std::cout<< "mEnd: " << wfs->mEnd << std::endl;
std::cout<< "mArraySize: " << wfs->mArraySize << std::endl;
    
    
    
    //List of indexes
    int indexNumber = wfs->mArraySize.x * wfs->mArraySize.y * wfs->mArraySize.z;
    double distance = 0;
    double minDistance = wfs->mCellCentres[0].Distance(wfs->mCellCentres[ctrPoint-1]);
    int ctrIndex = 0;
    wfs->mIndexTable = (int*) malloc (indexNumber * sizeof(int));
    
    for (double c = wfs->mOrigin.z; c <= wfs->mEnd.z; c += wfs->mGridSize.z){
    	for (double b = wfs->mOrigin.y; b <= wfs->mEnd.y; b += wfs->mGridSize.y){
    		for (double a = wfs->mOrigin.x; a <= wfs->mEnd.x; a += wfs->mGridSize.x){
    		//for each possible point in the area
    			for (int i = 0; i < ctrPoint; i++){
    				//search the nearest one in the cellcentres
    				distance = Point3(a,b,c).Distance(wfs->mCellCentres[i]);
					if (distance < minDistance){
						minDistance = distance;
						wfs->mIndexTable[ctrIndex] = i;
					}
					/*if (minDistance == 0)
						continue;*/
    			}
    		ctrIndex ++;
    		minDistance = wfs->mCellCentres[0].Distance(wfs->mCellCentres[ctrPoint-1]);
    		}
    	}
    }

  	/*//write in a file
  	std::ofstream myfile;
	myfile.open("/home/rahbar/Desktop/index.txt", std::ios::app);
	for(int k=0; k<ctrIndex; k++)
		myfile << wfs->mIndexTable[k] << "\n";
	myfile.close();*/
    
    //TODO send an error in case we don't have any point!
std::cout<< "mGridSize: " << wfs->mGridSize << std::endl;
std::cout<< "mOrigin: " << wfs->mOrigin << std::endl;
std::cout<< "mEnd: " << wfs->mEnd << std::endl;
std::cout<< "mArraySize: " << wfs->mArraySize << std::endl;
//std::cout << "WindFieldDynamic windSnapshotMemoryAllocation END" << std::endl;
}
