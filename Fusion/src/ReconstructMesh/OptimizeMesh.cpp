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


#include "OptimizeMesh.h"

#include <set>
#include <queue>

#include "ExtractContour.h"
#include "ExtractStroke.h"

#include "Mesh/MeshIO.h"
#include "Mesh/MeshView.h"
#include "Mesh/MeshCompute.h"
#include "Mesh/MeshKDTree.h"
#include "Mesh/MeshSubdivision.h"
#include "Mesh/MeshRemesh.h"

#include "Sample/SampleSimplePoissonDisk.h"

#include "Utility/FileUtil.h"
#include "Utility/TimerUtility.h"

#include "Format/PlyExporter.h"

using namespace Monster;

#define DEBUG_VISUALIZATION

bool OptimizeMesh::init(
	string &sketchViews,
	string &sketchFolder,
	string &mapFolder,
	string &resultFolder,
	string &viewPointFile)
{

	cout << "Loading data..." << endl;

	mResultFolder = resultFolder;
	mVisualFolder = mResultFolder + "/visual/";
	if (!FileUtil::makedir(mVisualFolder)) return false;

	// load view points

	if (!MeshView::loadViewPoints(viewPointFile, mViewPoints, &mViewGroups)) return false;

	// load sketch data

	int numMaps = (int)mViewPoints.size();
	if (!mViewMaps.loadSketch(sketchViews, sketchFolder)) return false;
	if (!preProcessSketch()) return false;
	int sketchSize = mViewMaps.mSketchSize;

	// load mesh

	string meshName = resultFolder + "mesh.ply";
	if (!MeshIO::loadMesh(meshName, mMesh)) return false;
	if (!preProcessMesh()) return false;

	// load view matrices

	Eigen::Matrix4d projMat, imgMat;
	if (!MeshView::buildProjMatrix(projMat)) return false;
	if (!MeshView::buildImageMatrix(imgMat, sketchSize, sketchSize)) return false;

	int numViews = (int)mViewPoints.size();
	mViewMatrices.resize(numViews);
	mRotateMatrices.resize(numViews);

	for (int viewID = 0; viewID < numViews; viewID++) {
		Eigen::Matrix4d viewMat;
		if (!MeshView::buildViewMatrix(mViewPoints[viewID], viewMat)) return false;
		mViewMatrices[viewID] = imgMat * projMat * viewMat;
		mRotateMatrices[viewID].setIdentity();
		mRotateMatrices[viewID].topLeftCorner(3, 3) = viewMat.topLeftCorner(3, 3);
	}

	return true;
}

bool OptimizeMesh::process() {

	int numIterations = 1; // UNDONE: param fitting optimization iteration

	for (int iterID = 0; iterID < numIterations; iterID++) {
		cout << "======== Optimization " << iterID << "========" << endl;

		auto timer = TimerUtility::tic();

		mStrokeFlags.assign(mMesh.amount, false);
		if (iterID+1 == numIterations) { // only do it in the last iteration
			if (!doStrokeMatching()) return false;
		}

		mContourHandles.resize(mMesh.amount);
		mContourFlags.assign(mMesh.amount, false);
		if (!doContourMatching()) return false;
		
		if (!doDeformation()) return false;

		if (!MeshIO::saveMesh(mResultFolder + "deform-" + to_string(iterID) + ".ply", mMesh)) return false;

		cout << "Timer: " << TimerUtility::toString(TimerUtility::toc(timer)) << endl;
	}
	if (!MeshIO::saveMesh(mResultFolder + "deform.ply", mMesh)) return false;

	return true;
}

bool OptimizeMesh::preProcessMesh() {

	int numVertices = mMesh.amount;

	// build graph

	mMeshGraph.assign(numVertices, unordered_set<int>());
	for (vec3i idx : mMesh.indices) {
		for (int k = 0; k < 3; k++) {
			int v1 = idx[k];
			int v2 = idx[(k + 1) % 3];
			mMeshGraph[v1].insert(v2);
			mMeshGraph[v2].insert(v1);
		}
	}

	// extract connected component

	vector<int> maxComponent(0);
	vector<bool> vertexFlags(numVertices, false);
	for (int vertID = 0; vertID < numVertices; vertID++) {
		if (vertexFlags[vertID]) continue;
		vertexFlags[vertID] = true;
		vector<int> queue(1, vertID);
		int head = 0;
		while (head < (int)queue.size()) {
			int curID = queue[head];
			for (int nbID : mMeshGraph[curID]) {
				if (!vertexFlags[nbID]) {
					queue.push_back(nbID);
					vertexFlags[nbID] = true;
				}
			}
			head++;
		}
		if (queue.size() > maxComponent.size()) maxComponent.swap(queue);
	}

	// prune mesh

	int numNewVertices = (int)maxComponent.size();
	vector<int> vertexMap(numVertices, -1);
	TTriangleMesh newMesh;
	newMesh.positions.resize(numNewVertices);
	newMesh.normals.resize(numNewVertices);
	for (int newID = 0; newID < numNewVertices; newID++) {
		int oldID = maxComponent[newID];
		vertexMap[oldID] = newID;
		newMesh.positions[newID] = mMesh.positions[oldID];
		newMesh.normals[newID] = mMesh.normals[oldID];
	}
	newMesh.indices.clear();
	for (vec3i idx : mMesh.indices) {
		if (vertexMap[idx[0]] < 0) continue;
		vec3i newIdx;
		for (int k = 0; k < 3; k++) newIdx[k] = vertexMap[idx[k]];
		newMesh.indices.push_back(newIdx);
	}
	newMesh.amount = numNewVertices;

	// output mesh

	mMesh.positions.swap(newMesh.positions);
	mMesh.normals.swap(newMesh.normals);
	mMesh.indices.swap(newMesh.indices);
	swap(mMesh.amount, newMesh.amount);

	// subdivide mesh

	//if (true) {
	//	vec3 meshCenter;
	//	float meshRadius;
	//	if (!MeshCompute::computeBoundingSphere(mMesh, meshCenter, meshRadius)) return false;
	//	double subdivRadius = meshRadius * 0.01;
	//	TTriangleMesh subdivMesh;
	//	vector<int> faceIndices;
	//	if (!MeshSubdivision::subdivideMeshKeepTopology(mMesh, subdivMesh, faceIndices, subdivRadius)) return false;
	//	mMesh = subdivMesh;
	//}

	// remesh to make it more regular

	if (true) {
		if (!MeshCompute::recomputeNormals(mMesh)) return false;
		if (!MeshRemesh::remesh(mMesh)) return false;
#ifdef DEBUG_VISUALIZATION
		if (!MeshIO::saveMesh(mVisualFolder + "remesh.ply", mMesh)) return false;
#endif
	}

	// rebuild mesh graph

	mMeshGraph.assign(mMesh.amount, unordered_set<int>());
	for (vec3i idx : mMesh.indices) {
		for (int k = 0; k < 3; k++) {
			int v1 = idx[k];
			int v2 = idx[(k + 1) % 3];
			mMeshGraph[v1].insert(v2);
			mMeshGraph[v2].insert(v1);
		}
	}

	if (!MeshCompute::recomputeNormals(mMesh)) return false;

	return true;
}

bool OptimizeMesh::preProcessSketch() {

	if (!extractSketchContour()) return false;
	if (!extractSketchStroke()) return false;

	return true;
}

bool OptimizeMesh::doStrokeMatching() {

	int numVertices = mMesh.amount;
	int numSketches = (int)mViewMaps.mSketches.size();
	int sketchSize = mViewMaps.mSketchSize;

	Eigen::Matrix4Xd vertexMat(4, numVertices);
	Eigen::Matrix4Xd normalMat(4, numVertices);
	for (int vertID = 0; vertID < numVertices; vertID++) {
		vec3 p = mMesh.positions[vertID];
		vec3 n = mMesh.normals[vertID];
		vertexMat.col(vertID) = Eigen::Vector4d(p[0], p[1], p[2], 1.0);
		normalMat.col(vertID) = Eigen::Vector4d(n[0], n[1], n[2], 1.0);
	}

	for (int sketchID = 0; sketchID < numSketches; sketchID++) {
		if (sketchID == 1) continue; // skip side sketch

		vector<vector<vec2d>> &strokes = mSketchStrokes[sketchID];
		int numStrokes = (int)strokes.size();

		// project vertex points to image space

		Eigen::Matrix4Xd projVertex = mViewMatrices[sketchID] * vertexMat;
		Eigen::Matrix4Xd projNormal = mRotateMatrices[sketchID] * normalMat;
		const double normalThreshold = 0.2;
		vector<int> meshPointIndices(0);
		vector<bool> checkFlags(numVertices, false);
		for (int vertID = 0; vertID < numVertices; vertID++) {
			if (!checkFlags[vertID] && projNormal(2, vertID) > normalThreshold) {
				checkFlags[vertID] = true;
				vector<int> queue(1, vertID);
				int head = 0;
				while (head < (int)queue.size()) {
					int curID = queue[head];
					for (int nbID : mMeshGraph[curID]) {
						if (!checkFlags[nbID] && projNormal(2, nbID) > normalThreshold) {
							queue.push_back(nbID);
							checkFlags[nbID] = true;
						}
					}
					head++;
				}
				if ((int)queue.size() > 20) {
					meshPointIndices.insert(meshPointIndices.end(), queue.begin(), queue.end());
				}
			}
		}
		vector<vec3> meshPoints(0);
		for (int id : meshPointIndices) {
			meshPoints.push_back(vec3d(projVertex(1, id), projVertex(0, id), 0.0)); // NOTE: in H-W coords
		}
		int numMeshPoints = (int)meshPoints.size();

#ifdef DEBUG_VISUALIZATION
		if (true) {
			vector<vec3> strokePoints(0);
			for (auto &chain : strokes) {
				for (vec2d sample : chain) {
					strokePoints.push_back(vec3d(sample[0], sample[1], 0.0));
				}
			}
			PlyExporter pe;
			if (!pe.addPoint(&meshPoints, 0, cml::identity_4x4(), vec3i(0, 0, 255))) return false;
			if (!pe.addPoint(&strokePoints, 0, cml::identity_4x4(), vec3i(255, 0, 0))) return false;
			if (!pe.output(mVisualFolder + "stroke-vertex.ply")) return false;
		}
#endif

		// build KD tree

		SKDTree meshTree;
		SKDTreeData meshTreeData;
		if (!MeshKDTree::buildKdTree(meshPoints, meshTree, meshTreeData)) return false;

		// find vertex path for each stroke

		double distBound = 2.0; // pixels
		for (int strokeID = 0; strokeID < numStrokes; strokeID++) {
			vector<vec2d> &chain = strokes[strokeID];
			int numStrokePoints = (int)chain.size();

			vector<vector<int>> strokeVerts(numStrokePoints, vector<int>(0));
			vector<vector<double>> strokeDists(numStrokePoints, vector<double>(0));
#pragma omp parallel for
			for (int strokePointID = 0; strokePointID < numStrokePoints; strokePointID++) {
				vec3 strokePoint = vec3d(chain[strokePointID][0], chain[strokePointID][1], 0.0);
				SKDT::NamedPoint queryPoint(strokePoint[0], strokePoint[1], strokePoint[2]);
				Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(3); // UNDONE: param number of neighbors for stroke verts
				meshTree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult, distBound);
				for (int qID = 0; qID < queryResult.size(); qID++) {
					int meshPointID = (int)meshTree.getElements()[queryResult[qID].getIndex()].id;
					int vertID = meshPointIndices[meshPointID];
					strokeVerts[strokePointID].push_back(vertID);
					strokeDists[strokePointID].push_back((meshPoints[meshPointID] - strokePoint).length());
				}
			}

			vector<int> vertMaps(numVertices, -1);
			vector<int> vertStrokes(numVertices, -1);
			vector<double> vertDists(numVertices, DBL_MAX);
			vector<int> vertexOfInterest(0);
			for (int strokePointID = 0; strokePointID < numStrokePoints; strokePointID++) {
				int numNNs = (int)strokeVerts[strokePointID].size();
				for (int nnID = 0; nnID < numNNs; nnID++) {
					int vertID = strokeVerts[strokePointID][nnID];
					double dist = strokeDists[strokePointID][nnID];
					if (vertMaps[vertID] < 0) {
						vertMaps[vertID] = (int)vertexOfInterest.size();
						vertexOfInterest.push_back(vertID);
					}
					if (dist < vertDists[vertID]) {
						vertDists[vertID] = dist;
						vertStrokes[vertID] = strokePointID;
					}
				}
			}

			int numVOI = (int)vertexOfInterest.size();
			vector<vector<pair<int, double>>> graph(numVOI);
			for (int vid = 0; vid < numVOI; vid++) {
				int vertID = vertexOfInterest[vid];
				for (int neighborID : mMeshGraph[vertID]) {
					int nid = vertMaps[neighborID];
					if (nid >= 0) {
						double weight = vertDists[vertID] + vertDists[neighborID];
						graph[vid].push_back(make_pair(nid, weight));
					}
				}
			}

			if (vertexOfInterest.empty()) continue;
			int start, middle, finish;
			int numStrokeVerts = (int)strokeVerts.size();
			if (!strokeVerts[0].empty()) start = vertMaps[strokeVerts[0][0]];
			else start = vertMaps[vertexOfInterest[0]];
			if (!strokeVerts[numStrokeVerts/2].empty()) middle = vertMaps[strokeVerts[numStrokeVerts/2][0]];
			else middle = vertMaps[vertexOfInterest[numVOI/2]];
			if (!strokeVerts[numStrokeVerts-1].empty()) finish = vertMaps[strokeVerts[numStrokeVerts-1][0]];
			else finish = vertMaps[vertexOfInterest[numVOI-1]];
			vector<int> pathSM, pathMT, pathTM, pathMS;
			if (!findShortestPath(graph, start, middle, pathSM)) return false;
			if (!findShortestPath(graph, middle, finish, pathMT)) return false;
			if (!findShortestPath(graph, finish, middle, pathTM)) return false;
			if (!findShortestPath(graph, middle, start, pathMS)) return false;
			vector<int> path;
			if (pathSM.size() + pathMT.size() > pathTM.size() + pathMS.size()) {
				path = pathSM;
				path.insert(path.end(), pathMT.begin() + 1, pathMT.end());
			} else {
				path = pathTM;
				path.insert(path.end(), pathMS.begin() + 1, pathMS.end());
			}
			int pathSize = (int)path.size();
			if (pathSize < 10) continue; // UNDONE: minimum path length

			for (int pid = 0; pid < pathSize; pid++) {
				int vertID = vertexOfInterest[path[pid]];
				mStrokeFlags[vertID] = true;
			}

			// move vertices towards stroke

			Eigen::Matrix4d reprojMat = mViewMatrices[sketchID].inverse();
			double depthGap = (MeshView::VIEW_RADIUS*2.0 - 0.1) / (sketchSize*MeshView::VIEW_RADIUS);

#pragma omp parallel for
			for (int pid = 0; pid < pathSize; pid++) {
				int vertID = vertexOfInterest[path[pid]];
				Eigen::Vector4d point = projVertex.col(vertID);
				Eigen::Vector4d normal = projNormal.col(vertID);
				int strokePointID = vertStrokes[vertID];
				vec3 strokePoint = vec3d(chain[strokePointID][0], chain[strokePointID][1], 0.0);
				Eigen::Vector4d matchedPoint(strokePoint[1], strokePoint[0], 0.0, 1.0); // NOTE: H-W corresponds to y-x
				matchedPoint[2] = point[2];
				// NOTE: it's hard to explain; but the calculation below should be correct...
				if (normal[2]) matchedPoint[2] += ((matchedPoint[0] - point[0]) * normal[0] + (point[1] - matchedPoint[1]) * normal[1]) / normal[2] * depthGap;

				matchedPoint = reprojMat * matchedPoint;
				mMesh.positions[vertID] = vec3d(matchedPoint[0], matchedPoint[1], matchedPoint[2]);
			}
		}
	}

#ifdef DEBUG_VISUALIZATION
	// visualize matching
	if (true) {
		if (!MeshIO::saveMesh(mVisualFolder + "stroke-base.ply", mMesh)) return false;
		vector<vec3> matchedPoints;
		for (int vertID = 0; vertID < numVertices; vertID++) {
			if (mStrokeFlags[vertID]) {
				matchedPoints.push_back(mMesh.positions[vertID]);
			}
		}
		PlyExporter pe;
		if (!pe.addPoint(&matchedPoints, 0, cml::identity_4x4(), vec3i(255, 0, 0))) return false;
		if (!pe.output(mVisualFolder + "stroke-handle.ply")) return false;
	}
#endif

	return true;
}

bool OptimizeMesh::doContourMatching() {

	int numVertices = mMesh.amount;
	int numSketches = (int)mViewMaps.mSketches.size();

	// match sketch contour point to mesh vertices

	for (int sketchID = 0; sketchID < numSketches; sketchID++) {

		// extract mesh contour vertices

		vector<int> meshContourVertexList;
		if (!extractMeshContour(sketchID, meshContourVertexList)) return false;
		int numContourVertices = (int)meshContourVertexList.size();
		Eigen::Matrix4Xd meshMat(4, numContourVertices);
		for (int contID = 0; contID < numContourVertices; contID++) {
			int vertID = meshContourVertexList[contID];
			vec3 p = mMesh.positions[vertID];
			meshMat.col(contID) = Eigen::Vector4d(p[0], p[1], p[2], 1.0);
		}
		
		// build KD tree for projected vertices

		Eigen::Matrix4d invProjMat = mViewMatrices[sketchID].inverse();
		Eigen::Matrix4Xd projMesh = mViewMatrices[sketchID] * meshMat;
		vector<vec3> projPoints(numContourVertices);
		for (int contID = 0; contID < numContourVertices; contID++) {
			projPoints[contID] = vec3d(projMesh(1, contID), projMesh(0, contID), 0.0); // NOTE: be careful about row/col order
		}
		SKDTree tree;
		SKDTreeData treeData;
		if (!MeshKDTree::buildKdTree(projPoints, tree, treeData)) return false;

		// find nearest vertice for each contour point

		for (vec2d contour : mSketchContours[sketchID]) {
			SKDT::NamedPoint queryPoint((float)contour[0], (float)contour[1], 0.0f);
			Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(3); // UNDONE: number of nearest neighbors in finding contour handles
			tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);

			for (int queryID = 0; queryID < queryResult.size(); queryID++) {
				int contID = (int)tree.getElements()[queryResult[queryID].getIndex()].id;
				int vertID = meshContourVertexList[contID];

				Eigen::Vector4d handle((double)contour[1], (double)contour[0], projMesh(2, contID), 1.0);
				handle = invProjMat * handle;
				vec3d newHandle = vec3d(handle[0], handle[1], handle[2]);
				if (mContourFlags[vertID]) {
					vec3d vertPos = mMesh.positions[vertID];
					double oldDist = (mContourHandles[vertID] - vertPos).length();
					double newDist = (newHandle - vertPos).length();
					if (newDist < oldDist) {
						mContourHandles[vertID] = newHandle;
					}
				} else {
					mContourHandles[vertID] = newHandle;
				}

				mContourFlags[vertID] = true;
			}
		}

	}

#ifdef DEBUG_VISUALIZATION
	// visualize matching
	if (true) {
		vector<vec3> pointPairs;
		for (int vertID = 0; vertID < numVertices; vertID++) {
			if (!mContourFlags[vertID]) continue;
			vec3 p1 = mMesh.positions[vertID];
			vec3 p2 = mContourHandles[vertID];
			pointPairs.push_back(p1);
			pointPairs.push_back(p2);
		}
		PlyExporter pe;
		if (!pe.addLine(&pointPairs)) return false;
		if (!pe.output(mVisualFolder + "contour-handle.ply")) return false;
		if (!MeshIO::saveMesh(mVisualFolder + "contour-base.ply", mMesh)) return false;
	}
#endif

	return true;
}

bool OptimizeMesh::doDeformation() {

	double coeffLaplacian = 10.0;
	double coeffContour = 1.0;

	int numVertices = mMesh.amount;

	cout << "Optimizing..." << endl;

	// compute Laplacian weights

	map<vec2i, double> laplacianWeights; // edge key => weight
	for (vec3i idx : mMesh.indices) {
		vec3d p[3];
		for (int k = 0; k < 3; k++) p[k] = mMesh.positions[idx[k]];
		double cotP[3];
		for (int k = 0; k < 3; k++) {
			vec3d v1 = cml::normalize(p[(k + 1) % 3] - p[k]);
			vec3d v2 = cml::normalize(p[(k + 2) % 3] - p[k]);
			cotP[k] = cml::dot(v1, v2) / cml::cross(v1, v2).length();
		}
		for (int k = 0; k < 3; k++) {
			double weight = max(0.0, cotP[k] / 2); // cotangent weight
			//double weight = 1.0; // uniform weight
			vec2i key(idx[(k + 1) % 3], idx[(k + 2) % 3]);
			if (key[0] > key[1]) swap(key[0], key[1]);
			auto &it = laplacianWeights.find(key);
			if (it == laplacianWeights.end()) {
				laplacianWeights[key] = weight;
			} else {
				it->second += weight;
			}
		}
	}
	vector<double> vertexTotalWeights(numVertices, 0);
	for (int vertID = 0; vertID < numVertices; vertID++) {
		for (int nbID : mMeshGraph[vertID]) {
			vec2i key(vertID, nbID);
			if (key[0] > key[1]) swap(key[0], key[1]);
			double weight = laplacianWeights[key];
			vertexTotalWeights[vertID] += weight;
		}
	}

	// compute Laplacian

	vector<vec3d> vertexLaplacian(numVertices, vec3d(0.0, 0.0, 0.0));
	for (int vertID = 0; vertID < numVertices; vertID++) {
		int k = (int)mMeshGraph[vertID].size();
		if (k > 0) {
			vec3d laplacian = mMesh.positions[vertID];
			double totalWeight = vertexTotalWeights[vertID];
			for (int nbID : mMeshGraph[vertID]) {
				vec2i key(vertID, nbID);
				if (key[0] > key[1]) swap(key[0], key[1]);
				double weight = laplacianWeights[key] / totalWeight;
				laplacian -= mMesh.positions[nbID] * weight;
			}

			if (mContourFlags[vertID]) {
				// scale and rotate Laplacian
				double scaleFactor = 1.0; // UNDONE: param feature point Laplacian scale factor
				double magnitude = laplacian.length() * scaleFactor;
				//magnitude = min(magnitude, 0.003);
				vec3d direction = mMesh.normals[vertID];
				//vec3d direction = cml::normalize(laplacian);
				if (cml::dot(direction, laplacian) < 0) direction = -direction;
				if(magnitude) laplacian = direction * magnitude;
			}

			if (mStrokeFlags[vertID]) {
				// scale and rotate Laplacian
				//double scaleFactor = 5.0; // UNDONE: param feature point Laplacian scale factor
				//double magnitude = laplacian.length() * scaleFactor;
				//magnitude = min(magnitude, 0.002);
				double magnitude = 0.002; // HACK: fixed magnitude
				//vec3d direction = cml::normalize(laplacian);
				vec3d direction = mMesh.normals[vertID];
				//if (cml::dot(direction, laplacian) < 0) direction = -direction;
				if (magnitude) laplacian = direction * magnitude;
			}

			vertexLaplacian[vertID] = laplacian;
		}
	}

#ifdef DEBUG_VISUALIZATION
	if (true) {
		// visualize Laplacian
		vector<vec3i> lapColor(numVertices);
		double maxValue = 0;
		for (vec3d lap : vertexLaplacian) {
			//for (int k = 0; k < 3; k++) maxValue = max(maxValue, lap[k]);
			maxValue = max(maxValue, lap.length());
		}
		for (int vertID = 0; vertID < numVertices; vertID++) {
			int color = (int)(vertexLaplacian[vertID].length() / maxValue * 255);
			for (int k = 0; k < 3; k++) {
				//lapColor[vertID][k] = (int)(vertexLaplacian[vertID][k] / maxValue * 255);
				lapColor[vertID][k] = color;
			}
		}
		PlyExporter pe;
		if (!pe.addPoint(&mMesh.positions, &mMesh.normals, &lapColor)) return false;
		if (!pe.output(mVisualFolder + "laplacian.ply")) return false;
	}
#endif

	// build sparse linear system

	vector<Eigen::Triplet<double, int>> triplets(0);
	vector<vec3d> values(0);

	int eqnID = 0;
	for (int vertID = 0; vertID < numVertices; vertID++) {

		// Delta(V) = Delta(V')
		if (true) {
			int k = (int)mMeshGraph[vertID].size();
			if (k > 0) {
				double totalWeight = vertexTotalWeights[vertID];
				for (int neighborID : mMeshGraph[vertID]) {
					vec2i key(vertID, neighborID);
					if (key[0] > key[1]) swap(key[0], key[1]);
					double weight = laplacianWeights[key] / totalWeight;
					double lhs = -weight * coeffLaplacian;
					triplets.push_back(Eigen::Triplet<double, int>(eqnID, neighborID, lhs));
				}
				double lhs = 1.0 * coeffLaplacian;
				triplets.push_back(Eigen::Triplet<double, int>(eqnID, vertID, lhs));
				vec3d rhs = vertexLaplacian[vertID] * coeffLaplacian;
				values.push_back(rhs);
				eqnID++;
			}
		}

		// V = Vc
		if (mContourFlags[vertID]) {
			vec3d vc = mContourHandles[vertID];
			double lhs = 1.0 * coeffContour;
			vec3d rhs = vc * coeffContour;
			triplets.push_back(Eigen::Triplet<double, int>(eqnID, vertID, lhs));
			values.push_back(rhs);
			eqnID++;
		}
	}
	int numEquations = eqnID;
	cout << "# eqns = " << numEquations << endl;

	Eigen::SparseMatrix<double> matA(numEquations, numVertices);
	matA.setFromTriplets(triplets.begin(), triplets.end());

	Eigen::MatrixX3d matB(numEquations, 3);
	for (int eqnID = 0; eqnID < numEquations; eqnID++) {
		matB.row(eqnID) = Eigen::RowVector3d(values[eqnID].data());
	}

	// get initial guess

	Eigen::MatrixX3d matG(numVertices, 3);
	for (int vertID = 0; vertID < numVertices; vertID++) {
		vec3d p = mMesh.positions[vertID];
		//if(mContourFlags[vertID]) p = mContourHandles[vertID];
		matG.row(vertID) = Eigen::RowVector3d(p.data());
	}

	// solve sparse linear system

	Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double>> solver;
	cout << "\t initializing solver" << endl;
	solver.compute(matA);
	if (solver.info() != Eigen::Success) return error("compute matA for solver");
	cout << "\t solving" << endl;
	solver.setMaxIterations(3000);
	solver.setTolerance(1e-6);
	Eigen::MatrixX3d matX = solver.solveWithGuess(matB, matG);
	cout << "\t # iters = " << solver.iterations() << endl;
	cout << "\t error = " << solver.error() << endl;
	cout << "\t status = " << solver.info() << endl;
	cout << "\t done." << endl;

	cout << "Residual error = " << (matA*matX - matB).norm() / matB.norm() << endl;
	cout << "Before: error = " << (matA*matG - matB).norm() << endl;
	cout << "After: error = " << (matA*matX - matB).norm() << endl;

	// update mesh

	for (int vertID = 0; vertID < numVertices; vertID++) {
		vec3d newVertex(matX(vertID, 0), matX(vertID, 1), matX(vertID, 2));
		mMesh.positions[vertID] = newVertex;
	}
	if (!MeshCompute::recomputeNormals(mMesh)) return false;

	return true;
}

bool OptimizeMesh::extractSketchContour() {

	cout << "Extracting contour..." << endl;

	int numSketches = (int)mViewMaps.mSketches.size();
	mSketchContours.resize(numSketches);

	for (int sketchID = 0; sketchID < numSketches; sketchID++) {
		vector<vector<double>> &sketch = mViewMaps.mSketches[sketchID];
		vector<vec2d> &contour = mSketchContours[sketchID];
		string visualFolder = mVisualFolder + "contour-" + to_string(sketchID) + "/";
		if (!FileUtil::makedir(visualFolder)) return false;
		if (!ExtractContour::extract(sketch, contour, visualFolder)) return false;
	}

	return true;
}

bool OptimizeMesh::extractSketchStroke() {

	cout << "Extracting stroke..." << endl;

	int numSketches = (int)mViewMaps.mSketches.size();
	int sketchSize = mViewMaps.mSketchSize;
	mSketchStrokes.resize(numSketches);

	for (int sketchID = 0; sketchID < numSketches; sketchID++) {
		vector<vector<double>> sketch = mViewMaps.mSketches[sketchID];
		vector<vec2d> &contour = mSketchContours[sketchID];

		// mark out contour sketches
		int kernel = mViewMaps.mSketchSize / 80; // UNDONE: param contour sketch marking out kernel size
		for (vec2d pixel : contour) {
			for (int row = max(0, (int)pixel[0] - kernel); row < min(sketchSize - 1, (int)pixel[0] + kernel); row++) {
				for (int col = max(0, (int)pixel[1] - kernel); col < min(sketchSize - 1, (int)pixel[1] + kernel); col++) {
					sketch[row][col] = 0.0;
				}
			}
		}

		// extract strokes
		vector<vector<vec2d>> &strokes = mSketchStrokes[sketchID];
		string visualFolder = mVisualFolder + "stroke-" + to_string(sketchID) + "/";
		if (!FileUtil::makedir(visualFolder)) return false;
		if (!ExtractStroke::extract(sketch, strokes, visualFolder)) return false;
	}

	return true;
}

bool OptimizeMesh::extractMeshContour(int viewID, vector<int> &contourVertices) {

	// find contours (edges with one front face and one back face)

	TTriangleMesh smoothMesh = mMesh;
	//if (!MeshRemesh::edgeSmooth(smoothMesh, 10)) return false;

	set<vec2i> frontEdges, backEdges;
	for (vec3i idx : smoothMesh.indices) {
		Eigen::Vector3d p[3];
		for (int k = 0; k < 3; k++) p[k] = Eigen::Vector3f(smoothMesh.positions[idx[k]].data()).cast<double>();
		Eigen::Vector3d n = (p[1] - p[0]).cross(p[2] - p[0]);
		bool isFront = (n.dot(mViewPoints[viewID]) >= 0);
		for (int k = 0; k < 3; k++) {
			vec2i key(idx[k], idx[(k + 1) % 3]);
			if (key[0]>key[1]) swap(key[0], key[1]);
			if (isFront) frontEdges.insert(key);
			else backEdges.insert(key);
		}
	}
	vector<vec2i> contourEdges;
	set_intersection(frontEdges.begin(), frontEdges.end(), backEdges.begin(), backEdges.end(),
					 inserter(contourEdges, contourEdges.begin()));

#ifdef DEBUG_VISUALIZATION
	// visualize contours
	if (true) {
		PlyExporter pe;
		if (!pe.addLine(&contourEdges, &mMesh.positions)) return false;
		if (!pe.output(mVisualFolder + "mesh-contour.ply")) return false;
	}
#endif

	// extract contour vertices

	if (true) {
		set<int> vertSet;
		for (vec2i edge : contourEdges) {
			vertSet.insert(edge[0]);
			vertSet.insert(edge[1]);
		}

		// expand vertex set

		int numIterations = 5;
		for (int iterID = 0; iterID < numIterations; iterID++) {
			set<int> newVertSet = vertSet;
			for (int vertID : vertSet) {
				for (int nbID : mMeshGraph[vertID]) {
					newVertSet.insert(nbID);
				}
			}
			vertSet.swap(newVertSet);
		}

		contourVertices.assign(vertSet.begin(), vertSet.end());
	}

	return true;
}

bool OptimizeMesh::findShortestPath(vector<vector<pair<int, double>>> &graph, int start, int finish, vector<int> &path) {

	int n = (int)graph.size();
	vector<double> dist(n, DBL_MAX);
	vector<int> from(n, -1);
	dist[start] = 0;

	auto cmp = [](pair<int, double> &lhs, pair<int, double> &rhs){return lhs.second > rhs.second; };
	priority_queue < pair<int, double>, vector<pair<int, double>>,
		std::function<bool(pair<int, double> &, pair<int, double> &)> > queue(cmp);
	queue.push(make_pair(start, 0.0));

	// find shortest path

	while (!queue.empty()) {
		pair<int, double> top = queue.top();
		queue.pop();
		if (dist[top.first] < top.second) continue;
		if (top.first == finish) break;
		for (pair<int, double> edge : graph[top.first]) {
			pair<int, double> next = make_pair(edge.first, edge.second + top.second);
			if (next.second < dist[next.first]) {
				dist[next.first] = next.second;
				from[next.first] = top.first;
				queue.push(next);
			}
		}
	}

	// reconstruct path

	if (from[finish] < 0) {
		double finishDist = 0;
		for (int id = 0; id < n; id++) {
			if (from[id] >= 0 && dist[id] > finishDist) {
				finish = id;
				finishDist = dist[id];
			}
		}
	}
	vector<int> reversePath(1, finish);
	int id = finish;
	while (from[id] >= 0) {
		id = from[id];
		reversePath.push_back(id);
	}
	path.assign(reversePath.rbegin(), reversePath.rend());

	return true;
}

bool OptimizeMesh::error(string s) {

	cout << "Error: " << s << endl;
	system("pause");

	return false;
}