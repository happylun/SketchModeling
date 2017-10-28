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


#include "MeshEnvelope.h"

#include <iostream>
#include <map>
#include <set>

#include "Library/EigenHelper.h"

#include "Mesh/MeshCompute.h"
#include "Mesh/MeshKDTree.h"
#include "Mesh/MeshRemesh.h"
#include "Mesh/MeshSubdivision.h"

using namespace Monster;

int MeshEnvelope::PARAM_GRID_DIMENSION     = 15;
int MeshEnvelope::PARAM_FITTING_ITERATION  = 2;
int MeshEnvelope::PARAM_SUBDIVISION        = 2;

MeshEnvelope::MeshEnvelope(TPointSet *samples) {

	mpSamples = samples;

	if (!MeshKDTree::buildKdTree(mpSamples->positions, mSampleTree, mSampleTreeData)) error("build KD tree");
}

MeshEnvelope::~MeshEnvelope() {
}

bool MeshEnvelope::process() {

	if (!constructEnvelope()) return false;
	if (!fitEnvelope()) return false;

	return true;
}

bool MeshEnvelope::output(TTriangleMesh &mesh) {

	mesh = mEnvelopeMesh;

	return true;
}

bool MeshEnvelope::constructEnvelope() {

	cout << "Initializing voxel grid..." << endl;

	// initialize voxel grid

	vec3 bbMin, bbMax;
	if (!MeshCompute::computeAABB(*mpSamples, bbMin, bbMax)) return false;
	float maxLength = max(bbMax[0] - bbMin[0], max(bbMax[1] - bbMin[1], bbMax[2] - bbMin[2]));
	float offsetLength = maxLength * 0.01f;
	bbMin -= vec3(offsetLength, offsetLength, offsetLength);
	maxLength += offsetLength * 2;
	float gridSize = maxLength / PARAM_GRID_DIMENSION;

	vec3i gridDimension;
	for (int k = 0; k < 3; k++) {
		gridDimension[k] = (int)(ceil((bbMax[k] - bbMin[k]) / gridSize));
	}

	// add padding

	gridDimension += vec3i(2, 2, 2);
	for (int k = 0; k < 3; k++) bbMin[k] -= gridSize;

	vec3i vertexDimension = gridDimension + vec3i(1, 1, 1);

	int numGridVoxels = gridDimension[0] * gridDimension[1] * gridDimension[2];
	int numGridVertices = vertexDimension[0] * vertexDimension[1] * vertexDimension[2];

	cout << "Constructing voxel hull..." << endl;

	// check voxels

	vector<bool> gridVoxelFlags(numGridVoxels, false);
#pragma omp parallel for
	for (int sampleID = 0; sampleID < mpSamples->amount; sampleID++) {
		vec3 samplePos = mpSamples->positions[sampleID] - bbMin;
		vec3i sampleCoord;
		for (int k = 0; k < 3; k++) {
			sampleCoord[k] = (int)(samplePos[k] / gridSize);
		}
		int gridPos = hashGrid(sampleCoord, gridDimension);
		gridVoxelFlags[gridPos] = true;
	}

	// mark visible voxel faces

	vec3i neighborGridOffsets[6] = {
		vec3i(-1, 0, 0), vec3i(1, 0, 0),
		vec3i(0, -1, 0), vec3i(0, 1, 0),
		vec3i(0, 0, -1), vec3i(0, 0, 1)
	};

	vec3i neighborFaceVertices[6][4] = {
		{ vec3i(0, 0, 0), vec3i(0, 0, 1), vec3i(0, 1, 1), vec3i(0, 1, 0) },
		{ vec3i(1, 0, 0), vec3i(1, 1, 0), vec3i(1, 1, 1), vec3i(1, 0, 1) },
		{ vec3i(0, 0, 0), vec3i(1, 0, 0), vec3i(1, 0, 1), vec3i(0, 0, 1) },
		{ vec3i(0, 1, 0), vec3i(0, 1, 1), vec3i(1, 1, 1), vec3i(1, 1, 0) },
		{ vec3i(0, 0, 0), vec3i(0, 1, 0), vec3i(1, 1, 0), vec3i(1, 0, 0) },
		{ vec3i(0, 0, 1), vec3i(1, 0, 1), vec3i(1, 1, 1), vec3i(0, 1, 1) }
	};

	vector<int> gridVertexFlags(numGridVertices, 0);
	map<vec2i, vec2i> gridEdgeFlags; // edge pair : (first voxel ID, touch number)
	vector<vec3i> envelopeFaces(0);
	vector<int> envelopeFaceVoxel(0);
	if (true) {
		vector<bool> visited(numGridVoxels, false);
		vector<int> queue(1, numGridVoxels-1);
		visited[numGridVoxels - 1] = true;

		// BFS flood fill
		int head = 0;
		while (head < (int)queue.size()) {
			int gridPos = queue[head];
			vec3i gridCoord = dehashGrid(gridPos, gridDimension);

			for (int nbID = 0; nbID < 6; nbID++) {
				vec3i nbCoord = gridCoord + neighborGridOffsets[nbID];
				if (nbCoord[0] < 0 || nbCoord[0] >= gridDimension[0] ||
					nbCoord[1] < 0 || nbCoord[1] >= gridDimension[1] ||
					nbCoord[2] < 0 || nbCoord[2] >= gridDimension[2]) continue; // out of boundary
				int nbPos = hashGrid(nbCoord, gridDimension);
				if (visited[nbPos]) continue;

				if (gridVoxelFlags[nbPos]) {
					// get face vertices
					vec4i facePos;
					for (int k = 0; k < 4; k++) {
						vec3i vertCoord = gridCoord + neighborFaceVertices[nbID][k];
						facePos[k] = hashGrid(vertCoord, vertexDimension);
					}
					// mark vertices
					for (int k = 0; k < 4; k++) {
						if (gridVertexFlags[facePos[k]] == 0) {
							gridVertexFlags[facePos[k]] = 1;
						}
					}
					
					// mark vertices on non-manifold edge
					bool anotherEdge = false;
					for (int k = 0; k < 4; k++) {						
						vec2i key(facePos[k], facePos[(k + 1) % 4]);
						if (key[0] > key[1]) swap(key[0], key[1]);
						auto it = gridEdgeFlags.find(key);
						if (it == gridEdgeFlags.end()) {
							gridEdgeFlags[key] = vec2i(nbPos, 1);
						} else {
							it->second[1]++;
							if (it->second[1] > 2) {
								anotherEdge = true;
								gridVertexFlags[key[0]] = -it->second[0] - 1; // important: minus 1 to handle 0 index case
								gridVertexFlags[key[1]] = -it->second[0] - 1;
							}
						}
					}
					
					envelopeFaces.push_back(vec3i(facePos[0], facePos[2], facePos[1]));
					envelopeFaces.push_back(vec3i(facePos[0], facePos[3], facePos[2]));
					envelopeFaceVoxel.push_back(nbPos);
					envelopeFaceVoxel.push_back(nbPos);
				} else {
					queue.push_back(nbPos);
					visited[nbPos] = true;
				}
			}
			head++;
		}
	}
	
	// update face index (after detecting non-manifold edges)
	for (int faceID = 0; faceID < (int)envelopeFaces.size(); faceID++) {
		vec3i &face = envelopeFaces[faceID];
		int voxel = envelopeFaceVoxel[faceID];
		for (int k = 0; k < 3; k++) {
			if (gridVertexFlags[face[k]] < 0) {
				if (-gridVertexFlags[face[k]] - 1 != voxel) { // important: minus 1
					face[k] = -face[k] - 1; // important: minus 1
				}
			}
		}
	}

	// compact envelope mesh

	mEnvelopeMesh.positions.clear();
	vector<int> vertexMap(numGridVertices, -1);
	vector<int> anotherVertexMap(numGridVertices, -1);
	for (int vertID = 0; vertID < numGridVertices; vertID++) {
		if (gridVertexFlags[vertID] != 0) {
			vertexMap[vertID] = (int)mEnvelopeMesh.positions.size();
			vec3i vertCoord = dehashGrid(vertID, vertexDimension);
			vec3 vertPos = bbMin + vec3(vertCoord)*gridSize;
			mEnvelopeMesh.positions.push_back(vertPos);
			mEnvelopeMesh.normals.push_back(vec3(0.f, 0.f, 1.f)); // dummy normals
			if (gridVertexFlags[vertID] < 0) {
				anotherVertexMap[vertID] = (int)mEnvelopeMesh.positions.size();
				mEnvelopeMesh.positions.push_back(vertPos);
				mEnvelopeMesh.normals.push_back(vec3(0.f, 0.f, 1.f));
			}
		}
	}
	mEnvelopeMesh.amount = (int)mEnvelopeMesh.positions.size();
	for (vec3i &idx : envelopeFaces) {
		vec3i faceIdx;
		for (int k = 0; k < 3; k++) {
			faceIdx[k] = idx[k] >= 0 ? vertexMap[idx[k]] : anotherVertexMap[-idx[k]-1]; // important: minus 1
		}
		mEnvelopeMesh.indices.push_back(faceIdx);
	}
	if (!MeshCompute::recomputeNormals(mEnvelopeMesh)) return false;

	cout << "Subdividing mesh..." << endl;

	// subdivide mesh

	vector<int> subdivIdx;
	for (int subdiv = 0; subdiv < PARAM_SUBDIVISION; subdiv++) {
		if (!MeshSubdivision::subdivideMeshMidPoint(mEnvelopeMesh, mEnvelopeMesh, subdivIdx)) return false;
	}

	return true;
}

bool MeshEnvelope::fitEnvelope() {

	cout << "Fitting envelope..." << endl;

	for (int iterID = 0; iterID < PARAM_FITTING_ITERATION; iterID++) {
		cout << "Iteration " << (iterID + 1) << endl;
		if (!fitMatching()) return false;
		if (!fitDeformation()) return false;
		if (!fitRegularization()) return false;
	}

	//vector<int> subdivIdx;
	//if (!MeshSubdivision::subdivideMeshMidPoint(mEnvelopeMesh, mEnvelopeMesh, subdivIdx)) return false;

	//if (!fitMatching()) return false;
	//if (!fitDeformation()) return false;

	if (!MeshCompute::recomputeNormals(mEnvelopeMesh)) return false;

	return true;
}

bool MeshEnvelope::fitMatching() {

	float filteringFactor = 5.0f; // UNDONE: param filter outliers with matching distance larger than 5 times average distance

	cout << "Matching..." << endl;

	// find nearest neighbors

	mFittingMatchingFlag.assign(mEnvelopeMesh.amount, true);
	mFittingMatchingPosition.resize(mEnvelopeMesh.amount);
	vector<float> matchDist(mEnvelopeMesh.amount);
#pragma omp parallel for
	for (int vertID = 0; vertID < mEnvelopeMesh.amount; vertID++) {
		vec3 vertex = mEnvelopeMesh.positions[vertID];
		SKDT::NamedPoint queryPoint(vertex[0], vertex[1], vertex[2]);
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(1);
		mSampleTree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);
		if (queryResult.isEmpty()) {
			mFittingMatchingFlag[vertID] = false;
			continue;
		}
		int matchID = (int)mSampleTree.getElements()[queryResult[0].getIndex()].id;
		vec3 matchPos = mpSamples->positions[matchID];
		mFittingMatchingPosition[vertID] = matchPos;
		matchDist[vertID] = (matchPos - vertex).length();
	}

	float totalMatchDist = 0;
	int totalMatches = 0;
	for (int vertID = 0; vertID < mEnvelopeMesh.amount; vertID++) {
		if (!mFittingMatchingFlag[vertID]) continue;
		totalMatches++;
		totalMatchDist += matchDist[vertID];
	}
	float averageMatchDist = totalMatchDist / totalMatches;

	for (int vertID = 0; vertID < mEnvelopeMesh.amount; vertID++) {
		if (!mFittingMatchingFlag[vertID]) continue;
		if (matchDist[vertID] > averageMatchDist * filteringFactor) {
			mFittingMatchingFlag[vertID] = false;
		}
	}

	return true;
}

bool MeshEnvelope::fitDeformation() {

	// UNDONE: param coefficients for 3 terms for deformation optimization
	float coeffLaplacian = 1.0f;
	float coeffPreserve = 0.5f;
	float coeffDeform = 2.0f;

	cout << "Building linear system..." << endl;

	// find neighbors

	vector<set<int>> neighbors(mEnvelopeMesh.amount);
	for (vec3i &idx : mEnvelopeMesh.indices) {
		for (int k = 0; k < 3; k++) {
			neighbors[idx[k]].insert(idx[(k + 1) % 3]);
			neighbors[idx[k]].insert(idx[(k + 2) % 3]);
		}
	}

	// compute Laplacian

	vector<vec3> vertexLaplacian(mEnvelopeMesh.amount);
	for (int vertID = 0; vertID < mEnvelopeMesh.amount; vertID++) {
		vec3 laplacian = mEnvelopeMesh.positions[vertID];
		int k = (int)neighbors[vertID].size();
		if (k > 0) {
			for (int nbID : neighbors[vertID]) {
				laplacian -= mEnvelopeMesh.positions[nbID] / k;
			}
		}
		vertexLaplacian[vertID] = laplacian;
	}

	// build sparse linear system

	vector<Eigen::Triplet<float, int>> triplets;
	Eigen::VectorXf matBx(mEnvelopeMesh.amount);
	Eigen::VectorXf matBy(mEnvelopeMesh.amount);
	Eigen::VectorXf matBz(mEnvelopeMesh.amount);
	float c1 = coeffLaplacian, c2 = coeffPreserve, c3 = coeffDeform; // just short names

	for (int vertID = 0; vertID < mEnvelopeMesh.amount; vertID++) {

		vector<float> coeff(mEnvelopeMesh.amount, 0);
		vec3 constantVec(0.0f, 0.0f, 0.0f);

		coeff[vertID] += c1;
		constantVec += c1 * vertexLaplacian[vertID];

		int k = (int)neighbors[vertID].size();
		if (k > 0) {
			for (int nbID : neighbors[vertID]) {
				coeff[nbID] += -c1 / k;

				int knb = (int)neighbors[nbID].size();
				if (knb > 0) {
					coeff[nbID] += -c1 / knb;
					constantVec += -c1 * vertexLaplacian[nbID] / knb;
					for (int nbnbID : neighbors[nbID]) {
						coeff[nbnbID] += c1 / (knb*knb);
					}
				}
			}
		}

		coeff[vertID] += c2;
		constantVec += c2 * mEnvelopeMesh.positions[vertID];

		if (mFittingMatchingFlag[vertID]) {
			coeff[vertID] += c3;
			constantVec += c3 * mFittingMatchingPosition[vertID];
		}

		for (int colID = 0; colID < mEnvelopeMesh.amount; colID++) {
			if (coeff[colID] != 0) {
				triplets.push_back(Eigen::Triplet<float, int>(vertID, colID, coeff[colID]));
			}
		}
		
		matBx(vertID) = constantVec[0];
		matBy(vertID) = constantVec[1];
		matBz(vertID) = constantVec[2];
	}
	Eigen::SparseMatrix<float> matA(mEnvelopeMesh.amount, mEnvelopeMesh.amount);
	matA.setFromTriplets(triplets.begin(), triplets.end());

	cout << "Solving linear system..." << endl;
	
	// solve linear system

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
	solver.compute(matA);
	if (solver.info() != Eigen::Success) return error("compute matA for solver");
	Eigen::VectorXf matXx, matXy, matXz;
	matXx = solver.solve(matBx);
	if (solver.info() != Eigen::Success) return error("solver for matXx");
	matXy = solver.solve(matBy);
	if (solver.info() != Eigen::Success) return error("solver for matXy");
	matXz = solver.solve(matBz);
	if (solver.info() != Eigen::Success) return error("solver for matXz");

	// update mesh

	for (int vertID = 0; vertID < mEnvelopeMesh.amount; vertID++) {
		mEnvelopeMesh.positions[vertID] = vec3(matXx(vertID), matXy(vertID), matXz(vertID));
	}

	return true;
}

bool MeshEnvelope::fitRegularization() {

	return MeshRemesh::remesh(mEnvelopeMesh);
}

int MeshEnvelope::hashGrid(vec3i pos, vec3i dim) {
	return (pos[0] * dim[1] + pos[1]) * dim[2] + pos[2];
}

vec3i MeshEnvelope::dehashGrid(int pos, vec3i dim) {
	vec3i coord;
	coord[0] = (pos / dim[2]) / dim[1];
	coord[1] = (pos/ dim[2]) % dim[1];
	coord[2] = pos % dim[2];
	return coord;
}

bool MeshEnvelope::error(string info) {

	cout << "Error: " << info << endl;
	return false;
}