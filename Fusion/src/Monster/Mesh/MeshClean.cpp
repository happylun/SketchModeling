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


#include "MeshClean.h"

#include <map>
#include <set>

#include "MeshCompute.h"
#include "MeshKDTree.h"

using namespace Monster;

bool MeshClean::removeDuplicateVertices(
	vector<vec3> &inVertices,
	vector<vec3> &outVertices,
	vector<int> &outVertexIndices,
	double eps)
{
	// compute eps

	if (eps < 0) {
		vec3 bbMin(FLT_MAX, FLT_MAX, FLT_MAX);
		vec3 bbMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		for (vec3 &v : inVertices) {
			bbMin.minimize(v);
			bbMax.maximize(v);
		}
		float bbLen = (bbMax - bbMin).length();
		eps = bbLen * 1e-5; // UNDONE: param eps
	}
	
	// merge duplicated vertices

	SKDTree vertexTree;
	SKDTreeData vertexTreeData;
	if (!MeshKDTree::buildKdTree(inVertices, vertexTree, vertexTreeData)) return false;

	outVertexIndices.resize(inVertices.size());
#pragma omp parallel for
	for (int vertID = 0; vertID < (int)inVertices.size(); vertID++) {
		vec3 p = inVertices[vertID];
		SKDT::NamedPoint queryPoint(p[0], p[1], p[2]);
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(100);
		vertexTree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult, eps);
		int mapID = vertID; // by default map to itself
		for (int qID = 0; qID < queryResult.size(); qID++) {
			int nbID = (int)vertexTree.getElements()[queryResult[qID].getIndex()].id;
			if (nbID < mapID) mapID = nbID;
		}
		outVertexIndices[vertID] = mapID;
	}

	vector<vec3> newVertices;
	for (int vertID = 0; vertID < (int)inVertices.size(); vertID++) {
		if (outVertexIndices[vertID] == vertID) {
			outVertexIndices[vertID] = (int)newVertices.size();
			newVertices.push_back(inVertices[vertID]);
		} else {
			outVertexIndices[vertID] = outVertexIndices[outVertexIndices[vertID]];
		}
	}

	outVertices.swap(newVertices);
	
	return true;
}

bool MeshClean::removeDuplicateVertices(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh,
	vector<int> &outVertexIndices,
	double eps)
{
	// compute eps

	if (eps < 0) {
		vec3 bbMin, bbMax;
		if (!MeshCompute::computeAABB(inMesh, bbMin, bbMax)) return false;
		float bbLen = (bbMax - bbMin).length();
		eps = bbLen * 1e-5; // UNDONE: param eps
	}

	// merge duplicated vertices

	vector<vec3> newPositions;
	vector<int> newVertexIndices;
	if (!removeDuplicateVertices(inMesh.positions, newPositions, newVertexIndices, eps)) return false;

	// update vertex normals

	vector<vec3> newNormals(newPositions.size());
	for (int vertID = 0; vertID < (int)inMesh.amount; vertID++) {
		newNormals[newVertexIndices[vertID]] = inMesh.normals[vertID];
	}

	// update face indices

	vector<vec3i> newIndices = inMesh.indices;
	for (int faceID = 0; faceID < (int)newIndices.size(); faceID++) {
		vec3i &faceIdx = newIndices[faceID];
		for (int k = 0; k < 3; k++) faceIdx[k] = newVertexIndices[faceIdx[k]];
	}

	outMesh.positions.swap(newPositions);
	outMesh.normals.swap(newNormals);
	outMesh.indices.swap(newIndices);
	outMesh.amount = (int)outMesh.positions.size();

	return true;
}

bool MeshClean::removeDuplicateFaces(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh,
	vector<int> &outFaceIndices,
	double eps)
{
	// compute eps

	if (eps < 0) {
		vec3 bbMin, bbMax;
		if (!MeshCompute::computeAABB(inMesh, bbMin, bbMax)) return false;
		float bbLen = (bbMax - bbMin).length();
		eps = bbLen * 1e-5; // UNDONE: param eps
	}

	// find unique faces

	vector<vec3i> newFaces(0);
	map<vec3i, int> faceMap; // face point indices => new face ID
	outFaceIndices.resize(inMesh.indices.size());
	for (int faceID = 0; faceID < (int)inMesh.indices.size(); faceID++) {
		vec3i key = inMesh.indices[faceID];
		if (key[0] > key[1]) swap(key[0], key[1]);
		if (key[0] > key[2]) swap(key[0], key[2]);
		if (key[1] > key[2]) swap(key[1], key[2]);
		auto it = faceMap.find(key);
		if (it == faceMap.end()) {
			int newFaceID = (int)newFaces.size();
			outFaceIndices[faceID] = newFaceID;
			faceMap[key] = newFaceID;
			newFaces.push_back(inMesh.indices[faceID]);
		} else {
			outFaceIndices[faceID] = it->second;
		}
	}

	outMesh = inMesh;
	outMesh.indices.swap(newFaces);

	return true;
}

bool MeshClean::removeDegeneratedFaces(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh,
	vector<int> &outFaceIndices,
	double eps)
{
	// assumes no duplicated vertices

	// compute eps

	if (eps < 0) {
		vec3 bbMin, bbMax;
		if (!MeshCompute::computeAABB(inMesh, bbMin, bbMax)) return false;
		float bbLen = (bbMax - bbMin).length();
		eps = cml::sqr(bbLen * 1e-5); // UNDONE: param eps
	}

	// find degenerated faces & edges to flip

	int numFaces = (int)inMesh.indices.size();
	vector<bool> faceFlags(numFaces, true);
	map<vec2i, int> flipEdgeSet; // flip edge key => inner point ID
	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i faceIdx = inMesh.indices[faceID];
		vec3d faceP[3];
		for (int k = 0; k < 3; k++) faceP[k] = vec3d(inMesh.positions[faceIdx[k]]);
		double dblArea = cml::cross(faceP[1] - faceP[0], faceP[2] - faceP[0]).length();
		if (dblArea < eps) {
			faceFlags[faceID] = false;
			double flipEdgeLenSq = 0;
			int flipEdgeID = -1;
			for (int k = 0; k < 3; k++) {
				double edgeLenSq = (faceP[(k + 1) % 3] - faceP[k]).length_squared();
				if (edgeLenSq > flipEdgeLenSq) {
					flipEdgeLenSq = edgeLenSq;
					flipEdgeID = k;
				}
			}
			vec2i flipEdgeKey(faceIdx[flipEdgeID], faceIdx[(flipEdgeID + 1) % 3]);
			if (flipEdgeKey[0] > flipEdgeKey[1]) swap(flipEdgeKey[0], flipEdgeKey[1]);
			int flipPointID = faceIdx[(flipEdgeID + 2) % 3];
			flipEdgeSet[flipEdgeKey] = flipPointID;
		}
	}

	// flip edges to prevent T-vertices

	vector<vec3i> extraFaces;
	map<int, int> extraFaceIndices;
	for (int faceID = 0; faceID < numFaces; faceID++) {
		if (!faceFlags[faceID]) continue; // skip degenerated faces
		vec3i faceIdx = inMesh.indices[faceID];
		for (int k = 0; k < 3; k++) {
			vec2i edgeKey(faceIdx[k], faceIdx[(k + 1) % 3]);
			if (edgeKey[0] > edgeKey[1]) swap(edgeKey[0], edgeKey[1]);
			auto it = flipEdgeSet.find(edgeKey);
			if (it != flipEdgeSet.end()) {
				faceFlags[faceID] = false;
				extraFaceIndices[faceID] = (int)extraFaces.size();
				extraFaces.push_back(vec3i(faceIdx[k], it->second, faceIdx[(k + 2) % 3]));
				extraFaces.push_back(vec3i(it->second, faceIdx[(k + 1) % 3], faceIdx[(k + 2) % 3]));
				break;
			}
		}
	}

	// update mesh faces

	vector<vec3i> newFaces(0);
	outFaceIndices.resize(numFaces);
	for (int faceID = 0; faceID < numFaces; faceID++) {
		if (faceFlags[faceID]) {
			outFaceIndices[faceID] = (int)newFaces.size();
			newFaces.push_back(inMesh.indices[faceID]);
		} else {
			outFaceIndices[faceID] = -1;
		}
	}
	for (auto &it : extraFaceIndices) {
		outFaceIndices[it.first] = it.second + (int)newFaces.size();
	}
	newFaces.insert(newFaces.end(), extraFaces.begin(), extraFaces.end());

	outMesh = inMesh;
	outMesh.indices.swap(newFaces);

	return true;
}

bool MeshClean::weldMeshFaces(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh,
	vector<int> &outFaceIndices,
	double eps)
{
	// compute eps

	if (eps < 0) {
		vec3 bbMin, bbMax;
		if (!MeshCompute::computeAABB(inMesh, bbMin, bbMax)) return false;
		float bbLen = (bbMax - bbMin).length();
		eps = bbLen * 1e-5; // UNDONE: param eps
	}

	// remove duplicate vertices/faces

	TTriangleMesh tmpMesh;
	vector<int> vertexIndices;
	vector<int> dupliFaceIndices;
	vector<int> degenFaceIndices;
	if (!MeshClean::removeDuplicateVertices(inMesh, tmpMesh, vertexIndices, eps)) return false;
	if (!MeshClean::removeDuplicateFaces(tmpMesh, outMesh, dupliFaceIndices, eps)) return false;
	if (!MeshClean::removeDegeneratedFaces(outMesh, outMesh, degenFaceIndices)) return false;

	outFaceIndices.resize(dupliFaceIndices.size());
	for (int k = 0; k < (int)outFaceIndices.size(); k++) {
		outFaceIndices[k] = degenFaceIndices[dupliFaceIndices[k]];
	}

	return true;
}

bool MeshClean::unweldMeshVertices(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh)
{
	int numFaces = (int)inMesh.indices.size();
	vector<vec3> newVertices(numFaces * 3);
	vector<vec3> newNormals(numFaces * 3);
	vector<vec3i> newIndices(numFaces);

	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i faceIdx = inMesh.indices[faceID];
		vec3 faceVerts[3];
		for (int k = 0; k < 3; k++) {
			faceVerts[k] = inMesh.positions[faceIdx[k]];
		}
		vec3 faceNormal = cml::normalize(cml::cross(faceVerts[1] - faceVerts[0], faceVerts[2] - faceVerts[0]));
		for (int k = 0; k < 3; k++) {
			newVertices[faceID * 3 + k] = faceVerts[k];
			newNormals[faceID * 3 + k] = faceNormal;
		}
		newIndices[faceID] = vec3i(faceID * 3, faceID * 3 + 1, faceID * 3 + 2);
	}

	outMesh.positions.swap(newVertices);
	outMesh.normals.swap(newNormals);
	outMesh.indices.swap(newIndices);
	outMesh.amount = numFaces * 3;

	return true;
}

bool MeshClean::cleanUp(TTriangleMesh &mesh) {

	// mark NaN vertices
	vector<bool> validFlags(mesh.amount, true);
	for (int vertID = 0; vertID < mesh.amount; vertID++) {
		if (!(mesh.positions[vertID] == mesh.positions[vertID])) {
			validFlags[vertID] = false;
		}
	}

	// remove zero area faces
	set<vec3i> newIndicesSet;
	vector<bool> verticesFlag(mesh.amount, false);
	for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
		vec3i idx = mesh.indices[faceID];
		if (!validFlags[idx[0]] || !validFlags[idx[1]] || !validFlags[idx[2]]) continue;
		if (idx[0] == idx[1] || idx[0] == idx[2] || idx[1] == idx[2]) continue;
		vec3 pos[3];
		for (int k = 0; k < 3; k++) pos[k] = mesh.positions[idx[k]];
		vec3d v1 = pos[1] - pos[0];
		vec3d v2 = pos[2] - pos[0];
		if (cml::dot(v1, v2) < 0) v2 = -v2;
		if (cml::unsigned_angle(v1, v2) < 1e-5) continue;
		newIndicesSet.insert(idx);
		for (int k = 0; k < 3; k++) verticesFlag[idx[k]] = true;
	}
	vector<vec3i> newIndices(newIndicesSet.begin(), newIndicesSet.end());

	// remove unreferenced vertices
	vector<vec3> newPositions;
	vector<vec3> newNormals;
	vector<int> verticesMap(mesh.amount);
	for (int vertID = 0; vertID < mesh.amount; vertID++) {
		if (verticesFlag[vertID]) {
			verticesMap[vertID] = (int)newPositions.size();
			newPositions.push_back(mesh.positions[vertID]);
			newNormals.push_back(mesh.normals[vertID]);
		}
	}

	// update face indices
	for (vec3i &idx : newIndices) {
		for (int k = 0; k < 3; k++) idx[k] = verticesMap[idx[k]];
	}


	mesh.positions.swap(newPositions);
	mesh.normals.swap(newNormals);
	mesh.indices.swap(newIndices);
	mesh.amount = (int)mesh.positions.size();

	return true;
}

bool MeshClean::cleanUp(TTetrahedralMesh &mesh) {

	// compute average edge length
	double totalLength = 0;
	for (vec4i idx : mesh.indices) {
		for (int i = 0; i < 3; i++) {
			vec3d p = mesh.positions[idx[i]];
			for (int j = i + 1; j < 4; j++) {
				vec3d q = mesh.positions[idx[j]];
				totalLength += (p - q).length();
			}
		}
	}
	double averageLength = totalLength / ((int)mesh.indices.size() * 6);
	double volumeEps = pow(averageLength*1e-6, 3.0);

	// remove zero volume tetrahedra
	set<vec4i> newIndicesSet;
	vector<bool> verticesFlag(mesh.amount, false);
	for (vec4i idx : mesh.indices) {

		vec3d side[3];
		for (int k = 0; k < 3; k++) side[k] = mesh.positions[idx[k + 1]] - mesh.positions[idx[0]];
		double volume = fabs(cml::dot(side[0], cml::cross(side[1], side[2]))) / 6;
		if (volume < volumeEps) continue;
		newIndicesSet.insert(idx);
		for (int k = 0; k < 4; k++) verticesFlag[idx[k]] = true;
	}
	vector<vec4i> newIndices(newIndicesSet.begin(), newIndicesSet.end());

	// remove unreferenced vertices
	vector<vec3> newPositions;
	vector<int> verticesMap(mesh.amount);
	int firstOne = -1;
	for (int vertID = 0; vertID < mesh.amount; vertID++) {
		if (verticesFlag[vertID]) {
			verticesMap[vertID] = (int)newPositions.size();
			newPositions.push_back(mesh.positions[vertID]);
		} else {
			if (firstOne < 0) firstOne = vertID;
		}
	}

	// update face indices
	for (vec4i &idx : newIndices) {
		for (int k = 0; k < 4; k++) idx[k] = verticesMap[idx[k]];
	}

	// make consistant index ordering
	for (vec4i &idx : newIndices) {
		vec3d v[3];
		for (int k = 0; k < 3; k++) v[k] = newPositions[idx[k + 1]] - newPositions[idx[0]];
		if (cml::dot(cml::cross(v[0], v[1]), v[2]) < 0) swap(idx[0], idx[1]);
	}

	cout << "Tetrahedral mesh clean up:" << endl;
	cout << "\tremoved " << (mesh.positions.size() - newPositions.size()) << " vertices ";
	cout << " and " << (mesh.indices.size() - newIndices.size()) << " tetrahedra" << endl;

	mesh.normals.clear();
	mesh.positions.swap(newPositions);
	mesh.indices.swap(newIndices);
	mesh.amount = (int)mesh.positions.size();

	return true;
}