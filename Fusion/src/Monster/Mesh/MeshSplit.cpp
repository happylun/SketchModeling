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


#include "MeshSplit.h"

#include <map>
#include <set>
#include <unordered_set>

#include "MeshCompute.h"
#include "MeshClean.h"

using namespace Monster;

bool MeshSplit::splitMeshIntoConnectedComponents(
	TTriangleMesh &inMesh,
	vector<TTriangleMesh> &outMeshes,
	vector<vector<int>> &outFaceIndices)
{
	int numVertices = inMesh.amount;
	int numFaces = (int)inMesh.indices.size();

	// build graph

	vector<unordered_set<int>> vertexGraph(numVertices);
	for (vec3i idx : inMesh.indices) {
		for (int k = 0; k < 3; k++) {
			vertexGraph[idx[k]].insert(idx[(k + 1) % 3]);
			vertexGraph[idx[k]].insert(idx[(k + 2) % 3]);
		}
	}

	// label connected vertices

	int numComponents = 0;
	vector<int> vertexLabels(numVertices, -1);
	vector<int> vertexMap(numVertices, -1);
	outMeshes.clear();

	for (int vertID = 0; vertID < numVertices; vertID++) {

		if (vertexLabels[vertID] >= 0) continue;
		vertexLabels[vertID] = numComponents;
		vertexMap[vertID] = 0;

		// BFS

		vector<int> queue(1, vertID);
		int head = 0;
		while (head < (int)queue.size()) {
			int currentID = queue[head];
			for (int neighborID : vertexGraph[currentID]) {
				if (vertexLabels[neighborID] >= 0) continue;
				vertexLabels[neighborID] = numComponents;
				vertexMap[neighborID] = (int)queue.size();
				queue.push_back(neighborID);
			}
			head++;
		}

		// add component

		outMeshes.push_back(TTriangleMesh());
		TTriangleMesh &mesh = outMeshes.back();
		mesh.positions.clear();
		mesh.normals.clear();
		mesh.indices.clear();
		for (int currentID : queue) {
			mesh.positions.push_back(inMesh.positions[currentID]);
			mesh.normals.push_back(inMesh.normals[currentID]);
		}
		mesh.amount = (int)mesh.positions.size();

		numComponents++;
	}

	// label faces

	outFaceIndices.assign(numComponents, vector<int>(0));
	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i oldFaceIdx = inMesh.indices[faceID];
		int compID = vertexLabels[oldFaceIdx[0]];
		vec3i newFaceIdx = vec3i(vertexMap[oldFaceIdx[0]], vertexMap[oldFaceIdx[1]], vertexMap[oldFaceIdx[2]]);
		outMeshes[compID].indices.push_back(newFaceIdx);
		outFaceIndices[compID].push_back(faceID);
	}

	return true;
}

bool MeshSplit::splitMeshIntoCleanedConnectedComponents(
	TTriangleMesh &inMesh,
	vector<TTriangleMesh> &outMeshes)
{

	// get raw connected components

	vector<TTriangleMesh> rawComponents;
	vector<vector<int>> rawComponentFaceIndices;
	if (!splitMeshIntoConnectedComponents(inMesh, rawComponents, rawComponentFaceIndices)) return false;
	int numRawComponents = (int)rawComponents.size();

	// sort connected components by face number

	vector<int> sortedComponentOrder(numRawComponents);
	for (int k = 0; k < numRawComponents; k++) sortedComponentOrder[k] = k;
	sort(sortedComponentOrder.begin(), sortedComponentOrder.end(),
		 [&rawComponents](int lhs, int rhs) {
		return rawComponents[lhs].indices.size() > rawComponents[rhs].indices.size();
	});

	// weld mesh

	TTriangleMesh weldedMesh;
	vector<int> weldedFaceIndices;
	if (true) {
		// NOTE: don't really weld it -- hard to handle degenerated faces
		//if (!weldMeshFaces(inMesh, weldedMesh, weldedFaceIndices)) return false;
		TTriangleMesh tmpMesh;
		vector<int> vertexIndices;
		if (!MeshClean::removeDuplicateVertices(inMesh, tmpMesh, vertexIndices)) return false;
		if (!MeshClean::removeDuplicateFaces(tmpMesh, weldedMesh, weldedFaceIndices)) return false;
	}
	int numWeldedFaces = (int)weldedMesh.indices.size();

	// label welded mesh faces by raw connected components

	vector<int> weldedFaceLabel(numWeldedFaces, -1);

	int numWeldedComponents = 0;
	for (int orderID = 0; orderID < numRawComponents; orderID++) { // check large components first
		int rawCompID = sortedComponentOrder[orderID];
		bool hasNewComponent = false;
		for (int inFaceID : rawComponentFaceIndices[rawCompID]) {
			int weldedFaceID = weldedFaceIndices[inFaceID];
			if (weldedFaceID < 0) {
				cout << "Error: incorrect welded face index" << endl;
				return false;
			}
			int &label = weldedFaceLabel[weldedFaceID];
			if (label < 0) {
				label = numWeldedComponents;
				hasNewComponent = true;
			}
		}
		if (hasNewComponent) numWeldedComponents++;
	}

	// output components

	vector<vector<int>> weldedComponentFaces(numWeldedComponents, vector<int>(0));
	for (int faceID = 0; faceID < numWeldedFaces; faceID++) {
		int label = weldedFaceLabel[faceID];
		if (label < 0) continue;
		weldedComponentFaces[label].push_back(faceID);
	}

	outMeshes.resize(numWeldedComponents);
	int numOutMeshes = 0;
	for (int compID = 0; compID < numWeldedComponents; compID++) {
		if (!extractSubMesh(weldedMesh, weldedComponentFaces[compID], outMeshes[numOutMeshes])) return false;
		double meshArea;
		if (!MeshCompute::computeFaceArea(outMeshes[numOutMeshes], meshArea)) return false;
		if (meshArea) numOutMeshes++; // skip empty component
	}
	outMeshes.resize(numOutMeshes);

	return true;
}

bool MeshSplit::splitMeshAlongCrease(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh,
	vector<int> *outLabels)
{
	// build edge set

	map<vec2i, vec2i> edgeMap; // edge vertex ID pair => face ID pair
	for (int faceID = 0; faceID < (int)inMesh.indices.size(); faceID++) {
		vec3i faceIdx = inMesh.indices[faceID];
		for (int k = 0; k < 3; k++) {
			vec2i edgeKey(faceIdx[k], faceIdx[(k + 1) % 3]);
			if (edgeKey[0] > edgeKey[1]) swap(edgeKey[0], edgeKey[1]);
			auto it = edgeMap.find(edgeKey);
			if (it == edgeMap.end()) {
				edgeMap[edgeKey] = vec2i(faceID, -1);
			} else {
				if (it->second[0] < 0) {
					// already marked as invalid - skip
				} else if (it->second[1] < 0) {
					it->second[1] = faceID;
				} else {
					// non-manifold edge - mark it as invalid
					it->second = vec2i(-1, -1);
				}
			}
		}
	}

	// check crease edge

	for (auto &it : edgeMap) {
		if (it.second[0] < 0 || it.second[1] < 0) continue;
		vec3 faceCenters[2];
		vec3 faceNormals[2];
		for (int alterID = 0; alterID < 2; alterID++) {
			int faceID = it.second[alterID];
			vec3i faceIdx = inMesh.indices[faceID];
			vec3 facePos[3];
			for (int k = 0; k < 3; k++) facePos[k] = inMesh.positions[faceIdx[k]];
			faceNormals[alterID] = cml::normalize(cml::cross(facePos[1] - facePos[0], facePos[2] - facePos[0]));
			faceCenters[alterID] = (facePos[0] + facePos[1] + facePos[2]) / 3;
		}
		double dihAngle;
		if (!MeshCompute::computeDihedralAngle(faceCenters[0], faceNormals[0], faceCenters[1], faceNormals[1], dihAngle)) return false;

		if (dihAngle >= cml::rad(269.0)) { // UNDONE: param crease angle threshold
			it.second = vec2i(-1, -1);
		}
	}

	// build graph

	vector<set<int>> faceNeighbors(inMesh.indices.size());
	for (auto &it : edgeMap) {
		if (it.second[0] >= 0 && it.second[1] >= 0) {
			faceNeighbors[it.second[0]].insert(it.second[1]);
			faceNeighbors[it.second[1]].insert(it.second[0]);
		}
	}

	// extract connected components

	vector<int> newVertexIndices(0);
	outMesh.indices.resize(inMesh.indices.size());

	int numComponents = 0;
	vector<int> visitedFlag(inMesh.indices.size(), false);
	if (outLabels) outLabels->resize(inMesh.indices.size());
	for (int faceID = 0; faceID < (int)inMesh.indices.size(); faceID++) {
		if (visitedFlag[faceID]) continue;
		visitedFlag[faceID] = true;

		// BFS
		vector<int> queue(1, faceID);
		int head = 0;
		while (head < (int)queue.size()) {
			int currentFace = queue[head];
			for (int neighborFace : faceNeighbors[currentFace]) {
				if (!visitedFlag[neighborFace]) {
					visitedFlag[neighborFace] = true;
					queue.push_back(neighborFace);
				}
			}
			head++;
		}

		// organize face data
		map<int, int> vertexMap;
		for (int faceID : queue) {
			vec3i oldIdx = inMesh.indices[faceID];
			vec3i newIdx;
			for (int k = 0; k < 3; k++) {
				auto it = vertexMap.find(oldIdx[k]);
				if (it == vertexMap.end()) {
					newIdx[k] = (int)newVertexIndices.size();
					vertexMap[oldIdx[k]] = newIdx[k];
					newVertexIndices.push_back(oldIdx[k]);
				} else {
					newIdx[k] = it->second;
				}
			}
			outMesh.indices[faceID] = newIdx;
			if (outLabels) (*outLabels)[faceID] = numComponents;
		}

		numComponents++;
	}

	// organize vertex data
	outMesh.positions.resize(newVertexIndices.size());
	outMesh.normals.resize(newVertexIndices.size());
	for (int vertID = 0; vertID < (int)newVertexIndices.size(); vertID++) {
		int newID = newVertexIndices[vertID];
		outMesh.positions[vertID] = inMesh.positions[newID];
		outMesh.normals[vertID] = inMesh.normals[newID];
	}
	outMesh.amount = (int)outMesh.positions.size();

	return true;
}

bool MeshSplit::extractSubMesh(
	TTriangleMesh &inMesh,
	vector<int> &subMeshFaces,
	TTriangleMesh &outMesh)
{

	// get all vertices

	set<int> newVertexSet;
	for (int faceID : subMeshFaces) {
		vec3i faceIdx = inMesh.indices[faceID];
		for (int k = 0; k < 3; k++) newVertexSet.insert(faceIdx[k]);
	}

	vector<int> newVertexList(newVertexSet.begin(), newVertexSet.end());
	int numNewVertices = (int)newVertexList.size();

	// output vertices

	vector<int> vertexMap(inMesh.amount, -1);
	outMesh.positions.resize(numNewVertices);
	outMesh.normals.resize(numNewVertices);
	for (int newID = 0; newID < numNewVertices; newID++) {
		int oldID = newVertexList[newID];
		vertexMap[oldID] = newID;
		outMesh.positions[newID] = inMesh.positions[oldID];
		outMesh.normals[newID] = inMesh.normals[oldID];
	}

	// output faces

	outMesh.indices.resize(subMeshFaces.size());
	for (int faceID = 0; faceID < (int)subMeshFaces.size(); faceID++) {
		vec3i &oldIdx = inMesh.indices[subMeshFaces[faceID]];
		vec3i &newIdx = outMesh.indices[faceID];
		for (int k = 0; k < 3; k++) newIdx[k] = vertexMap[oldIdx[k]];
	}

	outMesh.amount = numNewVertices;

	return true;
}