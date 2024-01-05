// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include "WindFieldSnapshot.h"
#include "TextFileReaderDouble.h"
#include "DataFileReader.h"
#include "DataFileWriter.h"
#define	THISCLASS WindFieldSnapshot

using namespace std;
using namespace std::chrono;

THISCLASS::WindFieldSnapshot():
		mTime(0), mWind(NULL), mArraySize(), mOrigin(), mGridSize() {

}

THISCLASS::~WindFieldSnapshot() {
	free(mWind);
	AllocateArray(Point3Int(0, 0, 0));
	free(mCellCentres);
}

void THISCLASS::WindFieldSnapshotCopy(WindFieldSnapshot &W1){
	mTime = W1.mTime;
	mArraySize = W1.mArraySize;
	mOrigin = W1.mOrigin;	
	mEnd = W1.mEnd;
	mGridSize = W1.mGridSize;
	mCellNbr = W1.mCellNbr;
	
	// mCellCentres
	if(W1.mCellCentres != NULL){
		mCellCentres = (Point3*) malloc (mCellNbr * sizeof(Point3));
		for(int i=0; i<mCellNbr; i++)
			mCellCentres[i] = W1.mCellCentres[i];
	}
	// mIndexTable
	if(W1.mIndexTable != NULL){
		int indexNumber = mArraySize.x * mArraySize.y * mArraySize.z;
		mIndexTable = (int*) malloc (indexNumber * sizeof(int));
		for(int i=0; i<indexNumber; i++)
			mIndexTable[i] = W1.mIndexTable[i];
	}
	// mWind
	AllocateArray(mCellNbr);
}

void THISCLASS::AllocateArray(const Point3Int &arraysize) {
	//std::cout << "arraysize = " << arraysize << std::endl;
	/*if (arraysize == mArraySize) {
		return;
	}*/
	// Delete old array
	if (mWind) {
		delete [] mWind;
		mWind = NULL;
	}
	// Create new array
	//int cc = mArraySize.Volume();
	int cc = arraysize.Volume(); //Fa
	//std::cout << "cc=" << cc << std::endl;
	if (cc > 0) {
		mArraySize = arraysize;
		//mWind = new Point3[cc];
		mWind = (Point3 *)malloc(cc*sizeof(Point3));
		//sizeof(double)=8; sizeof(Point3)=24; sizeof(Point3*)=8; sizeof(Point3[cc])=24*cc
	}
	std::cout << "Wind Field Memory allocated : " << cc << std::endl;
}

void THISCLASS::AllocateArray(const int &arraysize) { //Fa
	//std::cout << "(FA) arraysize = " << arraysize << std::endl;
	/*if (arraysize == mArraySize.Volume()) {
		return;
	}*/
	// Delete old array
	if (mWind) {
		delete [] mWind;
		mWind = NULL;
	}
	// Create new array
	//int cc = mArraySize.Volume();
	int cc = arraysize; //Fa
	//std::cout << "cc=" << cc << std::endl;
	if (cc > 0) {
		//mArraySize = arraysize;
		//mWind = new Point3[cc];
		mWind = (Point3*) malloc (cc * sizeof(Point3));
		//sizeof(double)=8; sizeof(Point3)=24; sizeof(Point3*)=8; sizeof(Point3[cc])=24*cc
	}
	std::cout << "Wind Field Memory allocated : " << cc << std::endl;
}

int THISCLASS::XYZToI(const Point3 &p) const {
	//int i = 0;
	//double distance = 0;
	//double minDistance = mCellCentres[0].Distance(mCellCentres[mCellNbr-1]); 
	//minDistance initialized to the longest distance possible
	//int closest = -1;
	
	//safety measures
	//std::cout << "p : " << p << std::endl;
	if ((p.x > mEnd.x+mGridSize.x/2) || (p.y > mEnd.y+mGridSize.y/2) || (p.z > mEnd.z+mGridSize.z/2)){
		//std::cout << "filament out of borders!" << std::endl;
		return -1;
	}
	
	/*//find the closest point
	for (i=0; i<mCellNbr; i++){
		distance = p.Distance(mCellCentres[i]);
		if (distance < minDistance){
			minDistance = distance;
			closest = i;
		}
		if (minDistance == 0)
			break;
	}*/
	
	//find the index
	int ix,iy,iz, idx;
	ix = round((p.x - mOrigin.x) / mGridSize.x); // + mGridSize.x/2
	if(ix < 0) ix =0;
	iy = round((p.y - mOrigin.y) / mGridSize.y);
	if(iy < 0) iy =0;
	iz = round((p.z - mOrigin.z) / mGridSize.z);
	if(iz < 0) iz =0;
	//std::cout << "p : " << p << " ix : " << ix << " iy : " << iy << " iz : " << iz << std::endl;
	idx = mIndexTable[ix + mArraySize.x * iy + mArraySize.x * mArraySize.y * iz];
	
	//if (idx != closest)
	//	std::cout << "p : " << p << " closest : " << mCellCentres[closest] << " idx : " << mCellCentres[idx] << std::endl;
	//return closest;
	return idx;
}

void THISCLASS::ReadTextFile(const std::string filename) {
	TextFileReaderDouble f(filename);
}

void THISCLASS::ReadBinaryFile(const std::string filename) { //never used
	DataFileReader f(filename);

	Point3Int arraysize;
	arraysize.Read(f);
	AllocateArray(arraysize);
	mOrigin.Read(f);
	mGridSize.Read(f);
	f.mFile.read((char*)mWind, sizeof(Point3)*mArraySize.Volume());
	f.Close();
}

void THISCLASS::WriteBinaryFile(const std::string filename) { //never used
	// Note that we don't store the time in the file. Timing information comes from the filename or from the folder structure.
	DataFileWriter f(filename);
	mArraySize.Write(f);
	mOrigin.Write(f);
	mGridSize.Write(f);
	f.mFile.write((char*)mWind, sizeof(Point3)*mArraySize.Volume());
	f.Close();
}

Point3 THISCLASS::GetWindSpeed(const Point3Int &p) const {
//std::chrono::high_resolution_clock::time_point t1 = high_resolution_clock::now();
	int i = XYZToI(p);
//std::chrono::high_resolution_clock::time_point t2 = high_resolution_clock::now();
//std::cout << "XYZToI executation time : " << std::chrono::duration_cast<microseconds>(t2-t1).count() << std::endl;
	//std::cout << "p=" << p << " i=" << i << " wind=" << mWind[i] <<  std::endl;
	if (i < 0) {
		return Point3(-100, -100, -100);
	}
	/*if((mWind[i].y > 1) | (mWind[i].z > 1))
		std::cout << "position: " << mCellCentres[i] << "	wind: " << mWind[i] << std::endl;*/
	return mWind[i];
}


Point3 THISCLASS::GetWindSpeed(const Point3 &preal) const {
	int closest = 0, closestOp;
	//int i = 0;
	//double distance = 0.0, minDistance = mCellCentres[0].Distance(mCellCentres[mCellNbr-1]); 
	
	//safety measures
	/*if ((preal.x > mEnd.x) || (preal.y > mEnd.y) || (preal.z > mEnd.z))
		return Point3(-100, -100, -100);*/
	
	//find the closest points
	/*for (i=0; i<mCellNbr; i++){
		distance = preal.Distance(mCellCentres[i]);
		if (distance < minDistance){
			minDistance = distance;
			closest = i;
		}
	}*/
	closest = XYZToI(preal);
	
	if (closest < 0) {
		//std::cout << "closest < 0" << std::endl;
		return Point3(-100, -100, -100);
	}
	
	//find the opposit point
	Point3 pOp;
	if (preal.x > mCellCentres[closest].x)
		pOp.x = mCellCentres[closest].x + mGridSize.x;
	else
		pOp.x = mCellCentres[closest].x - mGridSize.x;
	if (preal.y > mCellCentres[closest].y)
		pOp.y = mCellCentres[closest].y + mGridSize.y;
	else
		pOp.y = mCellCentres[closest].y - mGridSize.y;
	if (preal.z > mCellCentres[closest].z)
		pOp.z = mCellCentres[closest].z + mGridSize.z;
	else
		pOp.z = mCellCentres[closest].z - mGridSize.z;
	
	//find the index of pOp
	/*minDistance = mCellCentres[0].Distance(mCellCentres[mCellNbr-1]); 
	for (i=0; i<mCellNbr; i++){
		distance = pOp.Distance(mCellCentres[i]);
		if (distance < minDistance){
			minDistance = distance;
			closestOp = i;
		}
	}*/
	closestOp = XYZToI(pOp);
	if (closestOp < 0) {
		//std::cout << "closestOp < 0 " << std::endl;
		return Point3(-100, -100, -100);
	}
	Point3 wind = Point3(0,0,0), wtmp;
/*
	Point3 p = preal; //Fa 
	Point3 p1 = mCellCentres[closest];
	Point3 p2 = mCellCentres[closestOp];
	Point3 pdiff1 = ((Point3)p2 - p).Abs();
	Point3 pdiff2 = (p - (Point3)p1).Abs();
	//std::cout << "p = " << p << "   p1 = " << p1 << "   p2 = " << p2 << std::endl;
	//std::cout << "pdiff1 = " << pdiff1 << "   pdiff2 = " << pdiff2 << std::endl;
	
	wtmp = mWind[closest] * pdiff1.x * pdiff1.y * pdiff1.z;
	wind += wtmp;
	//std::cout << "wind1 = " << wtmp << "  wind = " << wind << " p = " << p1 << std::endl;
	
	wtmp = mWind[XYZToI(Point3(p1.x, p1.y, p2.z))] * pdiff1.x * pdiff1.y * pdiff2.z;
	wind += wtmp;
	//std::cout << "wind2 = " << wtmp << "  wind = " << wind << " p = " << Point3(p1.x, p1.y, p2.z) << std::endl;
	
	wtmp = mWind[XYZToI(Point3(p1.x, p2.y, p1.z))] * pdiff1.x * pdiff2.y * pdiff1.z;
	wind += wtmp;
	//std::cout << "wind3 = " << wtmp << "  wind = " << wind << " p = " << Point3(p1.x, p2.y, p1.z) << std::endl;
	
	wtmp = mWind[XYZToI(Point3(p1.x, p2.y, p2.z))] * pdiff1.x * pdiff2.y * pdiff2.z;
	wind += wtmp;
	//std::cout << "wind4 = " << wtmp << "  wind = " << wind << " p = " << Point3(p1.x, p2.y, p2.z) << std::endl;
	
	wtmp = mWind[XYZToI(Point3(p2.x, p1.y, p1.z))] * pdiff2.x * pdiff1.y * pdiff1.z;
	wind += wtmp;
	//std::cout << "wind5 = " << wtmp << "  wind = " << wind << " p = " << Point3(p2.x, p1.y, p1.z) << std::endl;
	
	wtmp = mWind[XYZToI(Point3(p2.x, p1.y, p2.z))] * pdiff2.x * pdiff1.y * pdiff2.z;
	wind += wtmp;
	//std::cout << "wind6 = " << wtmp << "  wind = " << wind << " p = " << Point3(p2.x, p1.y, p2.z) << std::endl;
	
	wtmp = mWind[XYZToI(Point3(p2.x, p2.y, p1.z))] * pdiff2.x * pdiff2.y * pdiff1.z;
	wind += wtmp;
	//std::cout << "wind7 = " << wtmp << "  wind = " << wind << " p = " << Point3(p2.x, p2.y, p1.z) << std::endl;
	
	wtmp = mWind[closestOp] * pdiff2.x * pdiff2.y * pdiff2.z;
	wind += wtmp;
	//std::cout << "wind8 = " << wtmp << "  wind = " << wind << " p = " << p2 << std::endl;

	
	wind = wind / (mGridSize.x * mGridSize.y * mGridSize.z);
*/
	wind = mWind[closest];
	//std::cout << "wind = " << wind << std::endl;
	return wind;
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<WindFieldSnapshot>" << std::endl;
	out << "\t<Time>" << mTime << "</Time>" << std::endl;
	out << "\t<ArraySize>" << mArraySize << "</ArraySize>" << std::endl;
	out << "\t<Origin>" << mOrigin << "</Origin>" << std::endl;
	out << "\t<GridSize>" << mGridSize << "</GridSize>" << std::endl;
	out << "</WindFieldSnapshot>" << std::endl;
}
