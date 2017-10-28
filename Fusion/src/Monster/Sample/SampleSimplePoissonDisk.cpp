//============================================================================
//
// This file is part of the Sketch Modeling project.
//
// Copyright (c) 2017
// -Zhaoliang Lun (author of the code) / UMass-Amherst
//
// This is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this software.  If not, see <http://www.gnu.org/licenses/>.
//
//============================================================================


#include "SampleSimplePoissonDisk.h"

#include <iostream>

#include "Mesh/MeshCompute.h"

#include "SampleRandomness.h"

using namespace Monster;

const double SampleSimplePoissonDisk::CONST_RELATIVE_RADIUS = 0.6; // 0.76

//#define DEBUG_OUTPUT

#define USE_INTERPOLATED_NORMAL 0
#define USE_FACE_NORMAL 1
#define USE_WHICH_NORMAL USE_FACE_NORMAL

SampleSimplePoissonDisk::SampleSimplePoissonDisk(TTriangleMesh *mesh) {

	clearUp();

	mpMesh = mesh;

	if (!MeshCompute::computeAABB(*mpMesh, mBBMin, mBBMax)) cout << "Error: mesh AABB" << endl;

	mSampleRadius = (mBBMax - mBBMin).length() * 1e-5; // temporary radius
}

SampleSimplePoissonDisk::~SampleSimplePoissonDisk() {

	clearUp();
}

bool SampleSimplePoissonDisk::runSampling(int numSamples) {

	if (!calculateCDF()) return false;

	mDesiredNumSamples = numSamples;
	mSampleRadius = sqrt(mTotalArea / (sqrt(3.0)*numSamples / 2)) * CONST_RELATIVE_RADIUS;

	if (!buildGrids()) return false;
	if (!generateSamples()) return false;

	return true;
}

bool SampleSimplePoissonDisk::runSampling(double sampleRadius) {

	if (!calculateCDF()) return false;

	mSampleRadius = sampleRadius;
	mDesiredNumSamples = (int)((mTotalArea * 2 / sqrt(3.0)) / cml::sqr(sampleRadius / CONST_RELATIVE_RADIUS));

	if (!buildGrids()) return false;
	if (!generateSamples()) return false;

	return true;
}

bool SampleSimplePoissonDisk::runSampling(int maxNumSamples, double minSampleRadius) {

	if (!calculateCDF()) return false;

	mSampleRadius = minSampleRadius;
	mDesiredNumSamples = (int)((mTotalArea * 2 / sqrt(3.0)) / cml::sqr(minSampleRadius / CONST_RELATIVE_RADIUS));
	if (mDesiredNumSamples > maxNumSamples) {
		mDesiredNumSamples = maxNumSamples;
		mSampleRadius = sqrt(mTotalArea / (sqrt(3.0)*maxNumSamples / 2)) * CONST_RELATIVE_RADIUS;
	}
	cout << "Sampling parameters: " << mDesiredNumSamples << ", " << mSampleRadius << endl;

	if (!buildGrids()) return false;
	if (!generateSamples()) return false;

	return true;
}

bool SampleSimplePoissonDisk::calculateCDF() {

#ifdef DEBUG_OUTPUT
	cout << "Calculating CDF..." << endl;
#endif

	int numFaces = (int)mpMesh->indices.size();

	// compute triangle area
	vector<double> faceArea(numFaces, 0);
#pragma omp parallel for
	for(int faceID=0; faceID<numFaces; faceID++) {
		vec3i &idx = mpMesh->indices[faceID];
		vec3d v[3];
		for (int j = 0; j < 3; j++) v[j] = mpMesh->positions[idx[j]];
		faceArea[faceID] = cml::cross((v[1] - v[0]), (v[2] - v[0])).length() / 2;
	}

	mTotalArea = 0;
	mTriangleAreaCDF.clear();
	for (int faceID = 0; faceID < numFaces; faceID++) {
		mTotalArea += faceArea[faceID];
		mTriangleAreaCDF.push_back(mTotalArea);
	}

	if(mTotalArea == 0) {
		cout << "Error: empty mesh" << endl;
		return false;
	}

	// normalize CDF
	double denom = 1.0 / mTotalArea;
	for(double &it : mTriangleAreaCDF) {
		it *= denom;
	}

	return true;
}

bool SampleSimplePoissonDisk::buildGrids() {

#ifdef DEBUG_OUTPUT
	cout << "Building grids..." << endl;
#endif	

	// calculate grid size

	const int maxGirdSize = 500; // UNDONE: param maximum grid dimension

	vec3 bbSize = mBBMax - mBBMin;
	double maxBBSize = (double) max(bbSize[0], max(bbSize[1], bbSize[2]));

	mGridRadius = max(mSampleRadius, maxBBSize/maxGirdSize);
	mGridSize[0] = cml::clamp((int)ceil( (double)bbSize[0] / mGridRadius ), 1, maxGirdSize);
	mGridSize[1] = cml::clamp((int)ceil( (double)bbSize[1] / mGridRadius ), 1, maxGirdSize);
	mGridSize[2] = cml::clamp((int)ceil( (double)bbSize[2] / mGridRadius ), 1, maxGirdSize);

	// init grids

	long long gridAllSize = mGridSize[0]*mGridSize[1]*mGridSize[2];
	if(gridAllSize > 200000000LL) {
		// requires too much memory, infeasible
		mSamples.clear();
		cout << "Error: requires too many grids" << endl;
		return false;
	}

	mGrids.clear();
	mGrids.resize((int)gridAllSize, vector<int>(0));

	return true;
}

bool SampleSimplePoissonDisk::generateSamples() {

#ifdef DEBUG_OUTPUT
	cout << "Start sampling..." << endl;
#endif

	// Poisson disk sampling

	int numFaces = (int)mpMesh->indices.size();

	mSamples.clear();
	mSamples.reserve(mDesiredNumSamples);
	int failCount = 0;

	int sampleID;
	for(sampleID=0; sampleID<mDesiredNumSamples;) {

		// generate samples

		int faceID = SampleRandomness::samplingCDF(mTriangleAreaCDF);

		TPoissonSample sample;
		if( !sampleOnTriangle(sample, faceID) ) return false;

		// check if sample is valid

		bool valid = true;

		int gridPos;
		if( checkGrid(sample, gridPos) ) {
#ifdef DEBUG_OUTPUT
			if(sampleID%1000 == 999) cout << "\rSampling: " << (sampleID+1) << " / " << mDesiredNumSamples << " : " << failCount << "        ";
#endif

			mGrids[gridPos].push_back(sampleID);
			mSamples.push_back(sample);
			failCount = 0;
			sampleID++;
		} else {
			failCount++;
			if (failCount > 10000) break;
		}
	}
#ifdef DEBUG_OUTPUT
	cout << "\rSampling: " << (sampleID + 1) << " / " << mDesiredNumSamples << " : " << failCount << endl;
#endif

	return true;
}

bool SampleSimplePoissonDisk::sampleOnTriangle(TPoissonSample &sample, int faceID) {

	// sample within a triangle

	vec3i idx = mpMesh->indices[faceID];
	vec3 v[3], n[3];
	for(int j=0; j<3; j++) {
		v[j] = mpMesh->positions[idx[j]];
		n[j] = mpMesh->normals[idx[j]];
	}

	double rng1 = cml::random_unit();
	double rng2 = cml::random_unit();
	float r1 = (float)(1 - sqrt(rng1));
	float r2 = (float)(rng2 * sqrt(rng1));

	sample.position = v[0] + (v[1]-v[0]) * r1 + (v[2]-v[0]) * r2;
#ifndef USE_WHICH_NORMAL
	#error normal method not specified
#elif USE_WHICH_NORMAL == USE_INTERPOLATED_NORMAL
	// be careful ! vertex normal is not reliable
	sample.normal = cml::normalize( n[0] + (n[1]-n[0]) * r1 + (n[2]-n[0]) * r2 );
#elif USE_WHICH_NORMAL == USE_FACE_NORMAL
	sample.normal = cml::cross(v[1]-v[0], v[2]-v[0]).normalize();
#else
	#error normal method not specified
#endif
	sample.faceID = faceID;

	return true;
}

bool SampleSimplePoissonDisk::checkGrid(TPoissonSample &sample, int &gridPos) {

	// check grid
	bool valid = true;

	vec3 relPos = sample.position - mBBMin;
	int gridX = cml::clamp( (int)floor( (double)relPos[0] / mGridRadius ), 0, mGridSize[0]-1 );
	int gridY = cml::clamp( (int)floor( (double)relPos[1] / mGridRadius ), 0, mGridSize[1]-1 );
	int gridZ = cml::clamp( (int)floor( (double)relPos[2] / mGridRadius ), 0, mGridSize[2]-1 );
	gridPos = (gridX * mGridSize[1] + gridY) * mGridSize[2] + gridZ;

	// check within 3x3x3 neighbor cells
	for(int offsetX=-1; offsetX<=1; offsetX++) {
		int clampX = cml::clamp(gridX+offsetX, 0, mGridSize[0]-1);
		if(gridX+offsetX != clampX) continue;
		for(int offsetY=-1; offsetY<=1; offsetY++) {
			int clampY = cml::clamp(gridY+offsetY, 0, mGridSize[1]-1);
			if(gridY+offsetY != clampY) continue;
			for(int offsetZ=-1; offsetZ<=1; offsetZ++) {
				int clampZ = cml::clamp(gridZ+offsetZ, 0, mGridSize[2]-1);
				if(gridZ+offsetZ != clampZ) continue;

				int checkPos = (clampX * mGridSize[1] + clampY) * mGridSize[2] + clampZ;
				for(int nbID : mGrids[checkPos]) {
					TPoissonSample &neighbor = mSamples[nbID];
					if(cml::dot(sample.normal, neighbor.normal) > 0) { // normal compatible

						vec3 &p1 = sample.position;
						vec3 &n1 = sample.normal;
						vec3 &p2 = neighbor.position;
						vec3 &n2 = neighbor.normal;
						vec3 v = p2-p1;

						// estimate geodesic distance
						//float dist = v.length();
						//v.normalize();
						//float c1 = cml::dot(n1, v);
						//float c2 = cml::dot(n2, v);
						//dist *= fabs(c1-c2) < 1e-7 ? (1.0f/sqrt(1.0f-c1*c1)) : (asin(c1)-asin(c2))/(c1-c2);

						// use L1 distance
						//float dist = fabs(v[0]) + fabs(v[1]) + fabs(v[2]);

						// use Euclid distance
						float dist = v.length();

						if( dist < mSampleRadius ) {
							valid = false;
							break;
						}
					}
				}
				if(!valid) break;
			}
			if(!valid) break;
		}
		if(!valid) break;
	}

	return valid;
}

bool SampleSimplePoissonDisk::exportSample(TSampleSet &samples) {

	samples.positions.clear();
	samples.normals.clear();
	samples.indices.clear();

	for(auto &sample : mSamples) {
		samples.positions.push_back( sample.position );
		samples.normals.push_back( sample.normal );
		samples.indices.push_back( sample.faceID );
	}
	samples.amount = (int)samples.positions.size();
	samples.radius = (float)mSampleRadius;

	return true;
}

void SampleSimplePoissonDisk::clearUp() {

	mTriangleAreaCDF.clear();
	mGrids.clear();
	mSamples.clear();
}
