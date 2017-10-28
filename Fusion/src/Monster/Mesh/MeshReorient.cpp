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


#include "MeshReorient.h"

#include <map>

#include "MeshCompute.h"
#include "MeshClean.h"
#include "MeshKDTree.h"
#include "MeshSplit.h"

using namespace Monster;



bool MeshReorient::reorientComponentFaces(TTriangleMesh &inoutMesh) {

	// assumes welded and connected mesh component

	vec3 bbMin, bbMax;
	if (!MeshCompute::computeAABB(inoutMesh, bbMin, bbMax)) return false;
	float eps = (bbMax - bbMin).length() * 1e-4f;

	int numVertices = inoutMesh.amount;
	int numFaces = (int)inoutMesh.indices.size();

	TKDTree tree;
	TKDTreeData treeData;
	if (!MeshKDTree::buildKdTree(inoutMesh, tree, treeData)) return false;

	// build face neighboring graph

	map<vec2i, vector<int>> edgeFaces; // edge key => face ID list
	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i &faceIdx = inoutMesh.indices[faceID];
		for (int k = 0; k < 3; k++) {
			vec2i edgeKey(faceIdx[k], faceIdx[(k + 1) % 3]);
			if (edgeKey[0] > edgeKey[1]) swap(edgeKey[0], edgeKey[1]);
			auto it = edgeFaces.find(edgeKey);
			if (it == edgeFaces.end()) {
				edgeFaces[edgeKey].assign(1, faceID);
			} else {
				it->second.push_back(faceID);
			}
		}
	}

	// compute face visibility

	int numRays = 16;

	vector<float> faceVisibility(numFaces);
	vector<vec3> faceNormal(numFaces);

#pragma omp parallel for
	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i &faceIdx = inoutMesh.indices[faceID];
		vec3 faceP[4];
		for (int k = 0; k < 3; k++) {
			faceP[k] = inoutMesh.positions[faceIdx[k]];
		}
		faceP[3] = (faceP[0] + faceP[1] + faceP[2]) / 3;
		for (int k = 0; k < 3; k++) faceP[k] = (faceP[k] + faceP[3]) / 2;
		vec3 faceN = cml::cross(faceP[1] - faceP[0], faceP[2] - faceP[0]);
		if (faceN.length_squared()) faceN.normalize();
		faceNormal[faceID] = faceN; // direction does not matter

		float maxVisibility = 0;
		bool flipFace = false;
		for (int k = 0; k < 4; k++) {
			int visibleCountP = 0;
			int visibleCountN = 0;
			for (int rayID = 0; rayID < numRays; rayID++) {
				double r1 = cml::random_unit();
				double r2 = cml::random_unit();
				vec3 rayDirP = vec3d(
					2.0 * cos(cml::constantsd::two_pi()*r1) * cml::sqrt_safe(r2*(1 - r2)),
					2.0 * sin(cml::constantsd::two_pi()*r1) * cml::sqrt_safe(r2*(1 - r2)),
					1.0 - 2.0*r2); // random direction on unit sphere
				if (cml::dot(rayDirP, faceN) < 0) rayDirP = -rayDirP;
				vec3 rayDirN = -rayDirP;

				vec3 rayOriginP = faceP[k] + rayDirP * eps; // add eps for offset
				vec3 rayOriginN = faceP[k] + rayDirN * eps;
				Thea::Ray3 rayP(G3D::Vector3(rayOriginP.data()), G3D::Vector3(rayDirP.data()));
				Thea::Ray3 rayN(G3D::Vector3(rayOriginN.data()), G3D::Vector3(rayDirN.data()));
				if (tree.rayIntersectionTime(rayP) < -0.5) visibleCountP++;
				if (tree.rayIntersectionTime(rayN) < -0.5) visibleCountN++;
			}
			float visibility = max(visibleCountP, visibleCountN) / (float)numRays;
			if (visibility > maxVisibility) {
				maxVisibility = visibility;
				flipFace = visibleCountN > visibleCountP;
			}
		}

		faceVisibility[faceID] = maxVisibility;
		if (flipFace) swap(faceIdx[1], faceIdx[2]);
	}

	// iteratively flip faces based on neighboring faces

	bool finish = false;
	int iteration = 0;
	while (!finish) {
		finish = true;
		vector<vec3i> newIndices = inoutMesh.indices;
		vector<bool> flipFlags(numFaces, false);
		int flipCount = 0;
#pragma omp parallel for shared(flipCount, flipFlags)
		for (int faceID = 0; faceID < numFaces; faceID++) {
			vec3i faceIdx = inoutMesh.indices[faceID];
			float flipVote = 0;
			float unflipVote = 0;
			bool canFlip = true;
			for (int k = 0; k < 3; k++) {
				vec2i edge(faceIdx[k], faceIdx[(k + 1) % 3]);
				vec2i edgeKey = edge;
				if (edgeKey[0] > edgeKey[1]) swap(edgeKey[0], edgeKey[1]);
				auto &nbList = edgeFaces[edgeKey];
				for (int nbID : nbList) {
					if (nbID == faceID) continue;
					if (flipFlags[nbID]) canFlip = false;
					float normalCosine = fabs(cml::dot(faceNormal[faceID], faceNormal[nbID]));
					vec3i &nbIdx = inoutMesh.indices[nbID];
					int ptr = 0;
					for (ptr = 0; ptr < 3; ptr++) if (nbIdx[ptr] == edge[0]) break;
					if (nbIdx[(ptr + 1) % 3] == edge[1]) {
						flipVote += (faceVisibility[nbID] + 0.1f);// *normalCosine;
					} else {
						unflipVote += (faceVisibility[nbID] + 0.1f);// *normalCosine;
					}
				}
			}
			if (canFlip && flipVote > unflipVote) {
				swap(newIndices[faceID][1], newIndices[faceID][2]);
				flipFlags[faceID] = true;
				finish = false;
#pragma omp atomic
				flipCount++;
			}
		}

		if (!finish) inoutMesh.indices.swap(newIndices);

		//cout << "Iteration " << iteration << " : flipped " << flipCount << " faces" << endl;
		iteration++;
		if (iteration > 100) break; // UNDONE: param maximum iteration

		//if (!MeshReorient::saveMesh(outMeshName, mesh)) return false;
		//system("pause");
	}

	return true;

	// sort face visibility

	vector<int> faceOrder(numFaces);
	for (int k = 0; k < numFaces; k++) faceOrder[k] = k;
	sort(faceOrder.begin(), faceOrder.end(),
		 [&faceVisibility](int id1, int id2){return faceVisibility[id1] > faceVisibility[id2]; });

	// determine face orientations by chaining

	vector<bool> faceProcessedFlag(numFaces, false);
	for (int orderID = 0; orderID < numFaces; orderID++) {
		int faceID = faceOrder[orderID];
		if (faceProcessedFlag[faceID]) continue;
		faceProcessedFlag[faceID] = true;

		// process all adjacent faces (BFS)

		vector<int> queue(1, faceID);
		int head = 0;
		while (head < (int)queue.size()) {
			int currentID = queue[head];
			vec3i faceIdx = inoutMesh.indices[currentID];
			for (int k = 0; k < 3; k++) {
				vec2i edge(faceIdx[k], faceIdx[(k + 1) % 3]);
				vec2i edgeKey = edge;
				if (edgeKey[0] > edgeKey[1]) swap(edgeKey[0], edgeKey[1]);
				auto &nbList = edgeFaces[edgeKey];
				for (int nbID : nbList) {
					if (faceProcessedFlag[nbID]) continue;
					faceProcessedFlag[nbID] = true;
					queue.push_back(nbID);
					vec3i &nbIdx = inoutMesh.indices[nbID];
					int ptr = 0;
					for (ptr = 0; ptr < 3; ptr++) if (nbIdx[ptr] == edge[0]) break;
					if (nbIdx[(ptr + 1) % 3] == edge[1]) {
						swap(nbIdx[1], nbIdx[2]); // flip face
					}
				}
			}
			head++;
		}
	}

	return true;
}

bool MeshReorient::reorientMeshFaces(TTriangleMesh &inoutMesh) {

	// assumes welded mesh

	// compute face normals

	vector<vec3> faceNormals(inoutMesh.indices.size());
	for (int faceID = 0; faceID < (int)inoutMesh.indices.size(); faceID++) {
		vec3i faceIdx = inoutMesh.indices[faceID];
		vec3 faceP[3];
		for (int k = 0; k < 3; k++) faceP[k] = inoutMesh.positions[faceIdx[k]];
		faceNormals[faceID] = cml::normalize(cml::cross(faceP[1] - faceP[0], faceP[2] - faceP[0]));
	}

	// split in connected components

	vector<TTriangleMesh> components;
	vector<vector<int>> componentFaceIndices;
	if (!MeshSplit::splitMeshIntoConnectedComponents(inoutMesh, components, componentFaceIndices)) return false;

	// re-orient each component

	int numComponent = (int)components.size();
	for (int compID = 0; compID < numComponent; compID++) {
		auto &component = components[compID];
		auto &faceIndices = componentFaceIndices[compID];
		if (!reorientComponentFaces(component)) return false;

		// determine flipping on original face
		for (int compFaceID = 0; compFaceID < (int)faceIndices.size(); compFaceID++) {
			int origFaceID = faceIndices[compFaceID];
			vec3i compFaceIdx = component.indices[compFaceID];
			vec3 compFaceP[3];
			for (int k = 0; k < 3; k++) compFaceP[k] = component.positions[compFaceIdx[k]];
			vec3 compFaceN = cml::normalize(cml::cross(compFaceP[1] - compFaceP[0], compFaceP[2] - compFaceP[0]));
			if (cml::dot(compFaceN, faceNormals[origFaceID]) < 0) {
				vec3i &origFaceIdx = inoutMesh.indices[origFaceID];
				swap(origFaceIdx[1], origFaceIdx[2]);
			}
		}
	}

	return true;
}

bool MeshReorient::reorientAnyMesh(TTriangleMesh &inoutMesh) {

	// no assumption on mesh

	// compute face normals

	vector<vec3> faceNormals(inoutMesh.indices.size());
	for (int faceID = 0; faceID < (int)inoutMesh.indices.size(); faceID++) {
		vec3i faceIdx = inoutMesh.indices[faceID];
		vec3 faceP[3];
		for (int k = 0; k < 3; k++) faceP[k] = inoutMesh.positions[faceIdx[k]];
		faceNormals[faceID] = cml::normalize(cml::cross(faceP[1] - faceP[0], faceP[2] - faceP[0]));
	}

	// weld & re-orient mesh

	TTriangleMesh weldedMesh;
	vector<int> weldedFaceIndices;
	if (!MeshClean::weldMeshFaces(inoutMesh, weldedMesh, weldedFaceIndices)) return false;
	if (!reorientMeshFaces(weldedMesh)) return false;

	// compute welded mesh face normals

	vector<vec3> weldedFaceNormals(weldedMesh.indices.size());
	for (int faceID = 0; faceID < (int)weldedMesh.indices.size(); faceID++) {
		vec3i faceIdx = weldedMesh.indices[faceID];
		vec3 faceP[3];
		for (int k = 0; k < 3; k++) faceP[k] = weldedMesh.positions[faceIdx[k]];
		weldedFaceNormals[faceID] = cml::normalize(cml::cross(faceP[1] - faceP[0], faceP[2] - faceP[0]));
	}

	// re-orient original mesh

	for (int faceID = 0; faceID < (int)inoutMesh.indices.size(); faceID++) {
		int weldedFaceID = weldedFaceIndices[faceID];
		if (cml::dot(faceNormals[faceID], weldedFaceNormals[weldedFaceID]) < 0) {
			vec3i &faceIdx = inoutMesh.indices[faceID];
			swap(faceIdx[1], faceIdx[2]);
		}
	}

	return true;
}