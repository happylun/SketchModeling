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


#include "MeshRemesh.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <queue>

#include "MeshClean.h"
#include "MeshCompute.h"
#include "MeshIO.h"

using namespace Monster;

bool MeshRemesh::remesh(TTriangleMesh &mesh) {

	TMeshEdgeSet edges;
	if (!splitNonManifoldEdges(mesh)) return false;
	if (!removeNonManifoldEdges(mesh)) return false;
	if (!buildEdgeStructure(mesh, edges)) return false;

	if (!edgeSplit(mesh, edges)) return false;
	if (!edgeCollapse(mesh, edges)) return false;
	if (!edgeFlip(mesh, edges)) return false;
	if (!edgeSmooth(mesh)) return false;

	// clean up
	if (!MeshClean::cleanUp(mesh)) return false;
	if (!MeshCompute::recomputeNormals(mesh)) return false;

	return true;
}

bool MeshRemesh::buildEdgeStructure(
	TTriangleMesh &inMesh,
	TMeshEdgeSet &outMeshEdges)
{
	outMeshEdges.clear();

	for (int faceID = 0; faceID < (int)inMesh.indices.size(); faceID++) {
		vec3i idx = inMesh.indices[faceID];
		for (int k = 0; k < 3; k++) {
			vec2i key = edgeKey(idx[k], idx[(k + 1) % 3]);
			auto foundIterator = outMeshEdges.find(key);
			if (foundIterator == outMeshEdges.end()) {
				TMeshEdge edge;
				edge.edgeVertices = vec2i(idx[k], idx[(k + 1) % 3]);
				edge.neighborVertices = vec2i(idx[(k + 2) % 3], -1);
				edge.edgeLength = (inMesh.positions[key[0]] - inMesh.positions[key[1]]).length();
				outMeshEdges[key] = edge;
			} else {
				TMeshEdge edge = foundIterator->second;
				if (edge.neighborVertices[1] < 0) {
					edge.neighborVertices[1] = idx[(k + 2) % 3];
					foundIterator->second = edge;
				} else {
					cout << "Error: not a 2-manifold mesh" << endl;
					return false;
				}
			}
		}
	}

	return true;
}

bool MeshRemesh::edgeSplit(
	TTriangleMesh &mesh,
	TMeshEdgeSet &edges)
{
	cout << "Edge split..." << endl;

	// compute average edge length

	float averageEdgeLength = 0;
	for (auto &it : edges) {
		averageEdgeLength += it.second.edgeLength;
	}
	if(!edges.empty()) averageEdgeLength /= (int)edges.size();

	float splitLength = averageEdgeLength *(4.0f / 3.0f); // UNDONE: param split length threshold

	// construct initial priority queue

	typedef pair<vec2i, float> TQueueEdge;
	auto SmallerEdge = [](TQueueEdge &e1, TQueueEdge &e2) { return e1.second < e2.second; };
	typedef priority_queue<TQueueEdge, vector<TQueueEdge>, decltype(SmallerEdge)> TQueue;

	TQueue queue(SmallerEdge);
	for (auto &it : edges) {
		TQueueEdge edge;
		edge.first = it.second.edgeVertices;
		edge.second = it.second.edgeLength;
		queue.push(edge);
	}

	// split edges in order

	while (!queue.empty()) {
		TQueueEdge topEdge = queue.top();
		queue.pop();
		if (topEdge.second > splitLength) {

			// remove original edge
			vec2i oldKey = edgeKey(topEdge.first);
			auto it = edges.find(oldKey);
			TMeshEdge edge = it->second;
			edges.erase(it);

			// split it in mid-point
			vec2i edgeVI = edge.edgeVertices;
			vec2i neighborVI = edge.neighborVertices;
			vec3 newPosition = (mesh.positions[edgeVI[0]] + mesh.positions[edgeVI[1]]) / 2;
			vec3 newNormal = mesh.normals[edgeVI[0]] + mesh.normals[edgeVI[1]];
			if (newNormal.length_squared() > 0) newNormal.normalize();
			int newIndex = (int)mesh.positions.size();
			mesh.positions.push_back(newPosition);
			mesh.normals.push_back(newNormal);

			// add new edges
			vec4i allVI(edgeVI[0], edgeVI[1], neighborVI[0], neighborVI[1]);
			for (int k = 0; k < 4; k++) {
				if (allVI[k] < 0) continue;
				vec2i newKey = edgeKey(allVI[k], newIndex);
				TMeshEdge newEdge;
				newEdge.edgeVertices = newKey;
				newEdge.neighborVertices = k < 2 ? neighborVI : edgeVI;
				if (k == 1 || k == 2) {
					swap(newEdge.edgeVertices[0], newEdge.edgeVertices[1]);
				}
				newEdge.edgeLength = (mesh.positions[newKey[0]] - mesh.positions[newKey[1]]).length();
				edges[newKey] = newEdge;
				queue.push(TQueueEdge(newEdge.edgeVertices, newEdge.edgeLength));
			}

			// update neighboring edges
			vector<vec3i> neighborEdges;
			neighborEdges.push_back(vec3i(edgeVI[0], neighborVI[0], edgeVI[1]));
			neighborEdges.push_back(vec3i(edgeVI[1], neighborVI[0], edgeVI[0]));
			neighborEdges.push_back(vec3i(edgeVI[0], neighborVI[1], edgeVI[1]));
			neighborEdges.push_back(vec3i(edgeVI[1], neighborVI[1], edgeVI[0]));
			for (vec3i &nb : neighborEdges) {
				vec2i nbKey = edgeKey(nb[0], nb[1]);
				auto it = edges.find(nbKey);
				if (it != edges.end()) {
					for (int k = 0; k < 2; k++) {
						if (it->second.neighborVertices[k] == nb[2]) {
							it->second.neighborVertices[k] = newIndex;
							break;
						}
					}
				}
			}
		}
	}

	// update faces based on edge set
	set<vec3i> newFaceSet;
	for (auto &it : edges) {
		TMeshEdge &edge = it.second;
		vector<vec3i> faces;
		faces.push_back(vec3i(edge.edgeVertices[0], edge.edgeVertices[1], edge.neighborVertices[0]));
		if (edge.neighborVertices[1] >= 0) {
			faces.push_back(vec3i(edge.edgeVertices[1], edge.edgeVertices[0], edge.neighborVertices[1]));
		}
		for (auto &face : faces) {
			while (face[0] > face[1] || face[0] > face[2]) {
				int tmp = face[0]; face[0] = face[1]; face[1] = face[2]; face[2] = tmp;
			}
			newFaceSet.insert(face);
		}
	}
	mesh.indices.assign(newFaceSet.begin(), newFaceSet.end());

	mesh.amount = (int)mesh.positions.size();

	return true;
}

bool MeshRemesh::edgeCollapse(
	TTriangleMesh &mesh,
	TMeshEdgeSet &edges)
{
	cout << "Edge collapse..." << endl;

	// compute average edge length

	float averageEdgeLength = 0;
	for (auto &it : edges) {
		averageEdgeLength += it.second.edgeLength;
	}
	if (!edges.empty()) averageEdgeLength /= (int)edges.size();

	float collapseLength = averageEdgeLength *(4.0f / 5.0f); // UNDONE: param collapse length threshold

	// build neighboring faces for each vertex

	vector<set<int>> vertexFaces(mesh.amount, set<int>());
	for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
		vec3i idx = mesh.indices[faceID];
		for (int k = 0; k < 3; k++) vertexFaces[idx[k]].insert(faceID);
	}

	// construct initial priority queue & valid edge set

	typedef pair<vec2i, float> TQueueEdge;
	auto GreaterEdge = [](TQueueEdge &e1, TQueueEdge &e2) { return e1.second > e2.second; };
	typedef priority_queue<TQueueEdge, vector<TQueueEdge>, decltype(GreaterEdge)> TQueue;

	TQueue queue(GreaterEdge);
	set<vec2i> validEdges;
	for (auto &it : edges) {
		TQueueEdge edge;
		edge.first = edgeKey(it.second.edgeVertices);
		edge.second = it.second.edgeLength;
		queue.push(edge);
		validEdges.insert(edge.first);
	}

	// collapse edges in order

	while (!queue.empty()) {
		TQueueEdge topEdge = queue.top();
		queue.pop();
		if (validEdges.find(topEdge.first) == validEdges.end()) continue;
		if (topEdge.second < collapseLength) {

			vec2i edgeVI = topEdge.first;
			validEdges.erase(edgeVI);

			// add mid-point
			vec3 newPosition = (mesh.positions[edgeVI[0]] + mesh.positions[edgeVI[1]]) / 2;
			vec3 newNormal = mesh.normals[edgeVI[0]] + mesh.normals[edgeVI[1]];
			if (newNormal.length_squared() > 0) newNormal.normalize();
			int newIndex = (int)mesh.positions.size();
			mesh.positions.push_back(newPosition);
			mesh.normals.push_back(newNormal);

			// check surrounding faces & edges
			for (int vi = 0; vi < 2; vi++) {
				for (int faceID : vertexFaces[edgeVI[vi]]) {
					vec3i idx = mesh.indices[faceID];
					for (int k = 0; k < 3; k++) {
						if (idx[k] == edgeVI[vi]) {
							for (int ki = 1; ki <= 2; ki++) {
								vec2i oldKey = edgeKey(idx[k], idx[(k + ki) % 3]);
								vec2i newKey = edgeKey(newIndex, idx[(k + ki) % 3]);
								validEdges.erase(oldKey);
								if (newKey[0] == newKey[1]) continue;
								TQueueEdge newEdge;
								newEdge.first = newKey;
								newEdge.second = (mesh.positions[newKey[0]] - mesh.positions[newKey[1]]).length();								
								if (validEdges.insert(newKey).second) {
									queue.push(newEdge); // new edge
								}
							}							
							idx[k] = newIndex;
						}
					}
					mesh.indices[faceID] = idx;
				}
			}

			// update vertex's neighboring faces
			vertexFaces.push_back(set<int>());
			vertexFaces.back().insert(vertexFaces[edgeVI[0]].begin(), vertexFaces[edgeVI[0]].end());
			vertexFaces.back().insert(vertexFaces[edgeVI[1]].begin(), vertexFaces[edgeVI[1]].end());
			vertexFaces[edgeVI[0]].clear();
			vertexFaces[edgeVI[1]].clear();
		}
	}
	mesh.amount = (int)mesh.positions.size();
	if (!MeshClean::cleanUp(mesh)) return false;

	// split non-manofold edges
	if (false) {
		map<vec3i, int> faceFlag;
		vector<bool> violatedFaces(mesh.indices.size(), false);
		for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
			vec3i idx = mesh.indices[faceID];
			if (idx[0] > idx[1]) swap(idx[0], idx[1]);
			if (idx[0] > idx[2]) swap(idx[0], idx[2]);
			if (idx[1] > idx[2]) swap(idx[1], idx[2]);
			auto it = faceFlag.find(idx);
			if (it != faceFlag.end()) {
				violatedFaces[it->second] = true;
				violatedFaces[faceID] = true;
			} else {
				faceFlag[idx] = faceID;
			}
		}
		vector<vec3i> newIndices;
		for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
			if (!violatedFaces[faceID]) newIndices.push_back(mesh.indices[faceID]);
		}
		mesh.indices.swap(newIndices);
	}
	if (!splitNonManifoldEdges(mesh)) return false;
	if (!removeNonManifoldEdges(mesh)) return false;
	if (!buildEdgeStructure(mesh, edges)) return false;

	return true;
}

bool MeshRemesh::splitNonManifoldEdges(TTriangleMesh &mesh) {
	
	int numFaces = (int)mesh.indices.size();

	// extract neighboring faces for each vertex

	vector<vector<int>> vertexFaces(mesh.amount, vector<int>(0));
	for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
		vec3i idx = mesh.indices[faceID];
		for (int k = 0; k < 3; k++) vertexFaces[idx[k]].push_back(faceID);
	}

	// detect non-manifold edges

	map<vec2i, vec2i> edgeMap; // edge pair => neighbor face pair
	set<int> nonManifoldVertices; // non-manifold vertices
	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i idx = mesh.indices[faceID];
		for (int k = 0; k < 3; k++) {
			vec2i key = edgeKey(idx[k], idx[(k + 1) % 3]);
			auto it = edgeMap.find(key);
			if (it == edgeMap.end()) {
				edgeMap[key] = vec2i(faceID, -1);
			} else {
				if (it->second[0] == -1) {
					continue; // already marked as non-manifold edge
				} else if (it->second[1] == -1) {
					it->second[1] = faceID;
				} else {
					it->second = vec2i(-1, -1); // non-manifold edge
					nonManifoldVertices.insert(it->first[0]);
					nonManifoldVertices.insert(it->first[1]);
				}
			}
		}
	}

	// build graph

	vector<vector<int>> graph(numFaces, vector<int>(0));
	for (auto it : edgeMap) {
		if (it.second[1] < 0) continue; // non-manifold edge or boundary edge
		graph[it.second[0]].push_back(it.second[1]);
		graph[it.second[1]].push_back(it.second[0]);
	}

	// extract connected components

	vector<int> faceFlag(numFaces, -1);
	int numComponents = 0;
	for (int faceID = 0; faceID < numFaces; faceID++) {
		if (faceFlag[faceID] >= 0) continue;
		faceFlag[faceID] = numComponents;
		vector<int> queue(1, faceID);
		int head = 0;
		while (head < (int)queue.size()) {
			// BFS
			for (int neighborID : graph[queue[head]]) {
				if (faceFlag[neighborID] < 0) {
					faceFlag[neighborID] = numComponents;
					queue.push_back(neighborID);
				}
			}
			head++;
		}
		numComponents++;
	}

	// update faces

	for (int vertID : nonManifoldVertices) {
		map<int, int> compMap; // component ID => vertex ID
		compMap[faceFlag[vertexFaces[vertID][0]]] = vertID;
		for (int faceID : vertexFaces[vertID]) {
			int compID = faceFlag[faceID];
			auto it = compMap.find(compID);
			int newVertID;
			if (it == compMap.end()) {
				newVertID = (int)mesh.positions.size();
				mesh.positions.push_back(mesh.positions[vertID]);
				mesh.normals.push_back(mesh.normals[vertID]);
				compMap[compID] = newVertID;
			} else {
				newVertID = it->second;
			}
			if (newVertID != vertID) {
				for (int k = 0; k < 3; k++) {
					if (mesh.indices[faceID][k] == vertID) {
						mesh.indices[faceID][k] = newVertID;
					}
				}
			}
		}
	}

	// remove small components
	vector<int> componentSize(numComponents, 0);
	for (int faceID = 0; faceID < numFaces; faceID++) {
		componentSize[faceFlag[faceID]]++;
	}
	vector<vec3i> newFaces;
	for (int faceID = 0; faceID < numFaces; faceID++) {
		if (componentSize[faceFlag[faceID]] > 20) {
			newFaces.push_back(mesh.indices[faceID]);
		}
	}
	mesh.indices.swap(newFaces);

	mesh.amount = (int)mesh.positions.size();

	return true;
}

bool MeshRemesh::removeNonManifoldEdges(TTriangleMesh &mesh) {

	int numFaces = (int)mesh.indices.size();

	// detect non-manifold edges

	map<vec2i, vec2i> edgeMap; // edge pair => neighbor face pair
	set<int> nonManifoldVertices; // non-manifold vertices
	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i idx = mesh.indices[faceID];
		for (int k = 0; k < 3; k++) {
			vec2i key = edgeKey(idx[k], idx[(k + 1) % 3]);
			auto it = edgeMap.find(key);
			if (it == edgeMap.end()) {
				edgeMap[key] = vec2i(faceID, -1);
			} else {
				if (it->second[0] == -1) {
					continue; // already marked as non-manifold edge
				} else if (it->second[1] == -1) {
					it->second[1] = faceID;
				} else {
					it->second = vec2i(-1, -1); // non-manifold edge
					nonManifoldVertices.insert(it->first[0]);
					nonManifoldVertices.insert(it->first[1]);
				}
			}
		}
	}

	// remove non-manifold edges

	vector<int> vertMap(mesh.amount, 0);
	for (int vertID : nonManifoldVertices) vertMap[vertID] = -1;
	vector<vec3> newPositions(0), newNormals(0);
	int numNewVertices = 0;
	for (int vertID = 0; vertID < mesh.amount; vertID++) {
		if (vertMap[vertID] == -1) continue;
		vertMap[vertID] = numNewVertices;
		newPositions.push_back(mesh.positions[vertID]);
		newNormals.push_back(mesh.normals[vertID]);
		numNewVertices++;
	}
	vector<vec3i> newIndices(0);
	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i idx = mesh.indices[faceID];
		vec3i newIdx;
		for (int k = 0; k < 3; k++) {
			newIdx[k] = vertMap[idx[k]];
		}
		if (newIdx[0] < 0 || newIdx[1] < 0 || newIdx[2] < 0) continue;
		newIndices.push_back(newIdx);
	}
	mesh.positions.swap(newPositions);
	mesh.normals.swap(newNormals);
	mesh.indices.swap(newIndices);
	mesh.amount = numNewVertices;

	return true;
}

bool MeshRemesh::edgeFlip(
	TTriangleMesh &mesh,
	TMeshEdgeSet &edges)
{
	// follow the edge flipping rule in trimesh
	cout << "Edge flip..." << endl;

	// construct initial priority queue

	typedef pair<vec2i, float> TQueueEdge;
	auto SmallerBenefit = [](TQueueEdge &e1, TQueueEdge &e2) { return e1.second < e2.second; };
	typedef priority_queue<TQueueEdge, vector<TQueueEdge>, decltype(SmallerBenefit)> TQueue;

	TQueue queue(SmallerBenefit);
	for (auto &it : edges) {
		TQueueEdge edge;
		edge.first = edgeKey(it.second.edgeVertices);
		edge.second = edgeFlipBenefit(mesh, it.second);
		queue.push(edge);
	}

	// flip edges in order

	while (!queue.empty()) {
		TQueueEdge topEdge = queue.top();
		queue.pop();
		auto it = edges.find(topEdge.first);
		if (it == edges.end()) continue;
		if (edges.find(edgeKey(it->second.neighborVertices)) != edges.end()) continue; // cannot flip
		TMeshEdge edge = it->second;
		float currentBenefit = edgeFlipBenefit(mesh, edge);
		if (currentBenefit > 0) {
			vec2i edgeVI = edge.edgeVertices;
			vec2i neighborVI = edge.neighborVertices;

			// flip edge
			edges.erase(it);
			edge.edgeVertices = neighborVI;
			edge.neighborVertices = vec2i(edgeVI[1], edgeVI[0]);
			edges[edgeKey(neighborVI)] = edge;

			// update surrounding edges
			vec4i allVI(edgeVI[0], neighborVI[0], edgeVI[1], neighborVI[1]);
			for (int vi = 0; vi < 4; vi++) {
				int a = allVI[vi];
				int b = allVI[(vi + 1) % 4];
				int c = allVI[(vi + 2) % 4];
				int d = allVI[(vi + 3) % 4];
				TMeshEdge &nbEdge = edges[edgeKey(a, b)];
				auto tmpEdge = nbEdge;
				nbEdge.neighborVertices[(nbEdge.edgeVertices[0] == a) ? 1 : 0] = (vi % 2 ? c : d);
				TQueueEdge qEdge;
				qEdge.first = edgeKey(a, b);
				qEdge.second = edgeFlipBenefit(mesh, nbEdge);
				if (qEdge.second > 0) queue.push(qEdge);
			}
		}
	}

	// update faces based on edge set
	set<vec3i> newFaceSet;
	for (auto &it : edges) {
		TMeshEdge &edge = it.second;
		vector<vec3i> faces;
		faces.push_back(vec3i(edge.edgeVertices[0], edge.edgeVertices[1], edge.neighborVertices[0]));
		if (edge.neighborVertices[1] >= 0) {
			faces.push_back(vec3i(edge.edgeVertices[1], edge.edgeVertices[0], edge.neighborVertices[1]));
		}
		for (auto &face : faces) {
			while (face[0] > face[1] || face[0] > face[2]) {
				int tmp = face[0]; face[0] = face[1]; face[1] = face[2]; face[2] = tmp;
			}
			newFaceSet.insert(face);
		}
	}
	mesh.indices.assign(newFaceSet.begin(), newFaceSet.end());

	return true;
}

bool MeshRemesh::edgeSmooth(TTriangleMesh &mesh, int iterations) {

	cout << "Edge smooth..." << endl;

	// find neighboring vertices

	vector<set<int>> vertexNeighbors(mesh.amount, set<int>());
	for (vec3i &idx : mesh.indices) {
		for (int k = 0; k < 3; k++) {
			vertexNeighbors[idx[k]].insert(idx[(k + 1) % 3]);
			vertexNeighbors[idx[k]].insert(idx[(k + 3) % 3]);
		}
	}

	// Laplacian smoothing

	vector<vec3> newPositions(mesh.amount);
	for (int iterID = 0; iterID < iterations; iterID++) {
		for (int vertID = 0; vertID < mesh.amount; vertID++) {
			vec3 pos(0.0f, 0.0f, 0.0f);
			for (int nbID : vertexNeighbors[vertID]) {
				pos += mesh.positions[nbID];
			}
			newPositions[vertID] = pos / max(1, (int)vertexNeighbors[vertID].size());
		}
		mesh.positions.swap(newPositions);
	}

	return true;
}

float MeshRemesh::edgeFlipBenefit(TTriangleMesh &mesh, TMeshEdge &edge) {

	// ref: trimesh - edgeflip.cc

	if (edge.neighborVertices[1] < 0) return 0; // boundary edge - unable to flip
	vec3 e1 = mesh.positions[edge.edgeVertices[0]];
	vec3 e2 = mesh.positions[edge.edgeVertices[1]];
	vec3 n1 = mesh.positions[edge.neighborVertices[0]];
	vec3 n2 = mesh.positions[edge.neighborVertices[1]];

	// if flipping creates a kink, don't flip it
	if (cml::dot(cml::cross(n2 - e1, n1 - e1), cml::cross(n1 - e2, n2 - e2)) <= 0) return 0;

	return max(-faceCosMaxAngle(e1, e2, n1), -faceCosMaxAngle(e1, n2, e2)) -
		max(-faceCosMaxAngle(e1, n2, n1), -faceCosMaxAngle(n1, n2, e2));
}

float MeshRemesh::faceCosMaxAngle(vec3 p1, vec3 p2, vec3 p3) {

	// ref: trimesh - edgeflip.cc

	float a = (p3 - p2).length();
	float b = (p3 - p1).length();
	float c = (p2 - p1).length();
	float A = a * (b*b + c*c - a*a);
	float B = b * (c*c + a*a - b*b);
	float C = c * (a*a + b*b - c*c);
	return 0.5f * min(min(A, B), C) / (a*b*c);
}