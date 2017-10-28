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


#include "MeshSubdivision.h"

#include <iostream>
#include <map>

#include "MeshCompute.h"

using namespace Monster;

bool MeshSubdivision::subdivideMesh(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh,
	vector<int> &outFaceIndices,
	double radius)
{

	// clone existing vertices
	outMesh.positions.assign(inMesh.positions.begin(), inMesh.positions.end());
	outMesh.normals.assign(inMesh.normals.begin(), inMesh.normals.end());

	// compute face area
	int numFaces = (int)inMesh.indices.size();

	float maxLength;
	if (radius) {
		maxLength = (float)(radius * 2);
	} else {
		// compute radius from default configuration
		float totalArea = 0;
		for (int faceID = 0; faceID < numFaces; faceID++) {
			vec3i faceIdx = inMesh.indices[faceID];
			vec3 facePos[3];
			for (int j = 0; j < 3; j++) facePos[j] = inMesh.positions[faceIdx[j]];
			float area = cml::cross(facePos[1] - facePos[0], facePos[2] - facePos[0]).length();
			totalArea += area;
		}
		int numSamples = 20000; // TODO: default configuration
		maxLength = sqrt(totalArea / (sqrt(3.0f)*numSamples / 2)) * 0.76f * 2;
	}

	// process each triangle
	outMesh.indices.clear();
	outFaceIndices.clear();
	for (int faceID = 0; faceID < numFaces; faceID++) {

		vec3i faceIdx = inMesh.indices[faceID];
		vec3 facePos[3];
		for (int j = 0; j < 3; j++) facePos[j] = inMesh.positions[faceIdx[j]];
		float length = 0;
		for (int j = 0; j < 3; j++) length = max(length, (facePos[(j + 1) % 3] - facePos[j]).length());

		if (length > maxLength) {
			// subdivide long triangle
			int div = (int)ceil(length / maxLength);

			// add new vertices
			vector<vector<int>> divPointIdx(div + 1, vector<int>(div + 1));
			for (int j = 0; j <= div; j++) {
				for (int v = 0; v <= j; v++) {
					int u = j - v;
					if (u == 0 && v == 0) divPointIdx[u][v] = faceIdx[0];
					else if (u == div && v == 0) divPointIdx[u][v] = faceIdx[1];
					else if (u == 0 && v == div) divPointIdx[u][v] = faceIdx[2];
					else {
						divPointIdx[u][v] = (int)outMesh.positions.size();
						outMesh.positions.push_back(facePos[0] + ((facePos[1] - facePos[0]) * u + (facePos[2] - facePos[0]) * v) / div);
						outMesh.normals.push_back(vec3()); // will calculate later
					}

				}
			}

			// add new faces
			for (int j = 0; j <= div; j++) {
				for (int v = 0; v <= j; v++) {
					int u = j - v;
					if (j < div) {
						outMesh.indices.push_back(vec3i(
							divPointIdx[u][v],
							divPointIdx[u + 1][v],
							divPointIdx[u][v + 1]));
						outFaceIndices.push_back(faceID);
						if (u == 0) continue;
						outMesh.indices.push_back(vec3i(
							divPointIdx[u][v],
							divPointIdx[u][v + 1],
							divPointIdx[u - 1][v + 1]));
						outFaceIndices.push_back(faceID);
					}
				}
			}
		} else {
			if (length == 0) continue; // skip zero area faces
			// retain original triangle
			outMesh.indices.push_back(faceIdx);
			outFaceIndices.push_back(faceID);
		}
	}
	outMesh.amount = (int)outMesh.positions.size();

	// compute normal
	if (!MeshCompute::recomputeNormals(outMesh)) return false;

	return true;
}

bool MeshSubdivision::subdivideMeshKeepTopology(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh,
	vector<int> &outFaceIndices,
	double radius)
{

	// clone existing vertices
	outMesh.positions.assign(inMesh.positions.begin(), inMesh.positions.end());
	outMesh.normals.assign(inMesh.normals.begin(), inMesh.normals.end());

	int numFaces = (int)inMesh.indices.size();

	// compute max length of edge

	float maxLength;
	if (radius) {
		maxLength = (float)(radius * 2);
	} else {
		// compute radius from default configuration
		float totalArea = 0;
		for (int faceID = 0; faceID < numFaces; faceID++) {
			vec3i faceIdx = inMesh.indices[faceID];
			vec3 facePos[3];
			for (int j = 0; j < 3; j++) facePos[j] = inMesh.positions[faceIdx[j]];
			float area = cml::cross(facePos[1] - facePos[0], facePos[2] - facePos[0]).length();
			totalArea += area;
		}
		int numSamples = 20000; // TODO: default configuration
		maxLength = sqrt(totalArea / (sqrt(3.0f)*numSamples / 2)) * 0.76f;
	}

	// divide each edge (add new vertices)

	vector<vector<int>> edgePointList; // point ID : # points on edge : # of edges
	map<vec2i, int> edgeMap; // edge ID : point ID pair (small first)

	for (int faceID = 0; faceID < numFaces; faceID++) {

		vec3i faceIdx = inMesh.indices[faceID];
		for (int j = 0; j < 3; j++) {
			int pID1 = faceIdx[j];
			int pID2 = faceIdx[(j + 1) % 3];
			if (pID1 > pID2) swap(pID1, pID2);
			vec2i edgeKey(pID1, pID2);
			auto it = edgeMap.find(edgeKey);
			if (it == edgeMap.end()) {
				vec3 p1 = inMesh.positions[pID1];
				vec3 n1 = inMesh.normals[pID1];
				vec3 p2 = inMesh.positions[pID2];
				vec3 n2 = inMesh.normals[pID2];
				float length = (p2 - p1).length();
				vector<int> pointList;
				pointList.push_back(pID1);
				if (length > maxLength) {
					// divide edge (add divide points)
					int div = (int)(ceil(length / maxLength));
					for (int k = 1; k < div; k++) {
						float r = k / (float)div;
						vec3 pNew = p2*r + p1*(1 - r);
						vec3 nNew = cml::normalize(n2*r + n1*(1 - r));
						int iNew = (int)outMesh.positions.size();
						outMesh.positions.push_back(pNew);
						outMesh.normals.push_back(nNew);
						pointList.push_back(iNew);
					}
				}
				pointList.push_back(pID2);
				edgeMap[edgeKey] = (int)edgePointList.size();
				edgePointList.push_back(pointList);
			}
		}
	}

	// first pass: divide triangle along one edge (the one to the vertex with smallest ID)

	vector<vec3i> tmpFaces(0);
	vector<int> tmpFaceIndices(0);

	for (int faceID = 0; faceID < numFaces; faceID++) {

		vec3i faceIdx = inMesh.indices[faceID];

		// rotate indices to make first index smallest
		if (faceIdx[1] <= faceIdx[0] && faceIdx[1] <= faceIdx[2]) {
			faceIdx = vec3i(faceIdx[1], faceIdx[2], faceIdx[0]);
		} else if (faceIdx[2] <= faceIdx[0] && faceIdx[2] <= faceIdx[1]) {
			faceIdx = vec3i(faceIdx[2], faceIdx[0], faceIdx[1]);
		}

		int topPointID = faceIdx[0];
		vec2i bottomEdgeKey = vec2i(faceIdx[1], faceIdx[2]);
		bool isSwapped = false;
		if (bottomEdgeKey[0] > bottomEdgeKey[1]) {
			swap(bottomEdgeKey[0], bottomEdgeKey[1]);
			isSwapped = true;
		}
		int bottomEdgeID = edgeMap[bottomEdgeKey];
		auto bottomEdgePointList = edgePointList[bottomEdgeID];
		for (int k = 0; k < (int)bottomEdgePointList.size() - 1; k++) {
			if (k > 0) {
				// add internal edge
				int pID1 = topPointID;
				int pID2 = bottomEdgePointList[k];
				vec2i edgeKey(pID1, pID2);
				vec3 p1 = outMesh.positions[pID1];
				vec3 n1 = outMesh.normals[pID1];
				vec3 p2 = outMesh.positions[pID2];
				vec3 n2 = outMesh.normals[pID2];
				float length = (p2 - p1).length();
				vector<int> pointList;
				pointList.push_back(pID1);
				if (length > maxLength) {
					// divide edge (add divide points)
					int div = (int)(ceil(length / maxLength));
					for (int k = 1; k < div; k++) {
						float r = k / (float)div;
						vec3 pNew = p2*r + p1*(1 - r);
						vec3 nNew = cml::normalize(n2*r + n1*(1 - r));
						int iNew = (int)outMesh.positions.size();
						outMesh.positions.push_back(pNew);
						outMesh.normals.push_back(nNew);
						pointList.push_back(iNew);
					}
				}
				pointList.push_back(pID2);
				edgeMap[edgeKey] = (int)edgePointList.size();
				edgePointList.push_back(pointList);
			}
			vec3i face = vec3i(topPointID, bottomEdgePointList[k], bottomEdgePointList[k + 1]);
			if (isSwapped) swap(face[1], face[2]);
			tmpFaces.push_back(face);
			tmpFaceIndices.push_back(faceID);
		}
	}

	// second pass: divide triangle along two divided edge

	outMesh.indices.clear();
	outFaceIndices.clear();

	for (int faceID = 0; faceID < (int)tmpFaces.size(); faceID++) {

		vec3i faceIdx = tmpFaces[faceID];
		int edgeID1 = edgeMap[vec2i(faceIdx[0], faceIdx[1])];
		int edgeID2 = edgeMap[vec2i(faceIdx[0], faceIdx[2])];
		auto &pointList1 = edgePointList[edgeID1];
		auto &pointList2 = edgePointList[edgeID2];
		if (pointList1[0] != faceIdx[0] || pointList2[0] != faceIdx[0]) {
			cout << "Error: incorrect point list" << endl;
			return false;
		}
		int n1 = (int)pointList1.size();
		int n2 = (int)pointList2.size();
		int p1 = 1;
		int p2 = 1;
		float l1 = (outMesh.positions[pointList1[p1]] - outMesh.positions[pointList1[0]]).length_squared();
		float l2 = (outMesh.positions[pointList2[p2]] - outMesh.positions[pointList2[0]]).length_squared();
		outMesh.indices.push_back(vec3i(faceIdx[0], pointList1[p1], pointList2[p2]));
		outFaceIndices.push_back(tmpFaceIndices[faceID]);
		while (true) {
			int oldP1 = p1;
			int oldP2 = p2;
			if (p1 == n1 - 1) p2++;
			else if (p2 == n2 - 1) p1++;
			else if (l1 < l2) p1++;
			else p2++;
			if (p1 >= n1 || p2 >= n2) {
				break;
			}
			if (oldP1 != p1) {
				l1 = (outMesh.positions[pointList1[p1]] - outMesh.positions[pointList1[0]]).length_squared();
				outMesh.indices.push_back(vec3i(pointList1[oldP1], pointList1[p1], pointList2[oldP2]));
				outFaceIndices.push_back(tmpFaceIndices[faceID]);
			} else {
				l2 = (outMesh.positions[pointList2[p2]] - outMesh.positions[pointList2[0]]).length_squared();
				outMesh.indices.push_back(vec3i(pointList1[oldP1], pointList2[p2], pointList2[oldP2]));
				outFaceIndices.push_back(tmpFaceIndices[faceID]);
			}
		}
	}

	outMesh.amount = (int)outMesh.positions.size();

	// compute normal
	if (!MeshCompute::recomputeNormals(outMesh)) return false;

	return true;
}

bool MeshSubdivision::subdivideMeshMidPoint(
	TTriangleMesh &inMesh,
	TTriangleMesh &outMesh,
	vector<int> &outFaceIndices,
	double radius)
{
	TTriangleMesh tmpMesh;
	tmpMesh.positions = inMesh.positions;
	tmpMesh.normals = inMesh.normals;

	outFaceIndices.clear();
	map<vec2i, int> edgeMap;
	vec4i caseCount(0, 0, 0, 0);
	for (int faceID = 0; faceID < (int)inMesh.indices.size(); faceID++) {
		vec3i facePoints = inMesh.indices[faceID];

		// compute mid point on edges
		vec3i midPoints(-1, -1, -1);
		for (int k = 0; k < 3; k++) {
			vec2i edgeID(facePoints[k], facePoints[(k + 1) % 3]);
			if (edgeID[0] > edgeID[1]) swap(edgeID[0], edgeID[1]);
			auto it = edgeMap.find(edgeID);
			if (it == edgeMap.end()) {
				vec3 p0 = inMesh.positions[edgeID[0]];
				vec3 p1 = inMesh.positions[edgeID[1]];
				float edgeLen = (p0 - p1).length();
				if (edgeLen < radius) {
					edgeMap[edgeID] = -1;
					midPoints[k] = -1;
				} else {
					vec3 midPosition = (p0 + p1) / 2;
					vec3 midNormal = inMesh.normals[edgeID[0]] + inMesh.normals[edgeID[1]];
					if (midNormal.length_squared() > 0) midNormal.normalize();
					int midIndex = (int)tmpMesh.positions.size();
					tmpMesh.positions.push_back(midPosition);
					tmpMesh.normals.push_back(midNormal);
					edgeMap[edgeID] = midIndex;
					midPoints[k] = midIndex;
				}
			} else {
				midPoints[k] = it->second;
			}
		}
		// handle different cases
		int collapseCount = 0;
		for (int k = 0; k < 3; k++) if (midPoints[k] < 0) collapseCount++;
		if (collapseCount == 0) {
			tmpMesh.indices.push_back(vec3i(facePoints[0], midPoints[0], midPoints[2]));
			tmpMesh.indices.push_back(vec3i(midPoints[0], facePoints[1], midPoints[1]));
			tmpMesh.indices.push_back(vec3i(midPoints[2], midPoints[1], facePoints[2]));
			tmpMesh.indices.push_back(vec3i(midPoints[0], midPoints[1], midPoints[2]));
			for (int k = 0; k < 4; k++) outFaceIndices.push_back(faceID);
			caseCount[0]++;
		} else if (collapseCount == 1) {
			int collapseID = -1;
			for (int k = 0; k < 3; k++) if (midPoints[k] < 0) collapseID = k;
			vec3i i(collapseID, (collapseID + 1) % 3, (collapseID + 2) % 3);
			tmpMesh.indices.push_back(vec3i(facePoints[i[2]], midPoints[i[2]], midPoints[i[1]]));
			float l1 = (tmpMesh.positions[facePoints[i[0]]] - tmpMesh.positions[midPoints[i[1]]]).length();
			float l2 = (tmpMesh.positions[facePoints[i[1]]] - tmpMesh.positions[midPoints[i[2]]]).length();
			if (l1 < l2) {
				tmpMesh.indices.push_back(vec3i(facePoints[i[0]], midPoints[i[1]], midPoints[i[2]]));
				tmpMesh.indices.push_back(vec3i(facePoints[i[0]], facePoints[i[1]], midPoints[i[1]]));
			} else {
				tmpMesh.indices.push_back(vec3i(facePoints[i[0]], facePoints[i[1]], midPoints[i[2]]));
				tmpMesh.indices.push_back(vec3i(facePoints[i[1]], midPoints[i[1]], midPoints[i[2]]));
			}
			for (int k = 0; k < 3; k++) outFaceIndices.push_back(faceID);
			caseCount[1]++;
		} else if (collapseCount == 2) {
			int subID = -1;
			for (int k = 0; k < 3; k++) if (midPoints[k] >= 0) subID = k;
			vec3i i(subID, (subID + 1) % 3, (subID + 2) % 3);
			tmpMesh.indices.push_back(vec3i(facePoints[i[0]], midPoints[i[0]], facePoints[i[2]]));
			tmpMesh.indices.push_back(vec3i(midPoints[i[0]], facePoints[i[1]], facePoints[i[2]]));
			for (int k = 0; k < 2; k++) outFaceIndices.push_back(faceID);
			caseCount[2]++;
		} else if (collapseCount == 3) {
			tmpMesh.indices.push_back(facePoints);
			outFaceIndices.push_back(faceID);
			caseCount[3]++;
		}
	}

	outMesh.positions.swap(tmpMesh.positions);
	outMesh.normals.swap(tmpMesh.normals);
	outMesh.indices.swap(tmpMesh.indices);
	outMesh.amount = (int)outMesh.positions.size();

	return true;
}