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


#include "OptimizeViews.h"

#include <unordered_set>

#include "ProjectViews.h"
#include "OptimizeDepths.h"

#include "Mesh/MeshView.h"
#include "Mesh/MeshIO.h"
#include "Mesh/MeshCompute.h"
#include "Mesh/MeshEnvelope.h"

#include "Image/ImageIO.h"

#include "Match/MatchRigidICP.h"

#include "Sample/SampleSimplePoissonDisk.h"

#include "Utility/FileUtil.h"

using namespace Monster;

bool OptimizeViews::init(
	string &sketchViews,
	string &sketchFolder,
	string &mapFolder,
	string &resultFolder,
	string &viewPointFile)
{

	mResultFolder = resultFolder;
	if (!FileUtil::makedir(mResultFolder)) return false;

	// load view points

	if (!MeshView::loadViewPoints(viewPointFile, mViewPoints, &mViewGroups)) return false;
	if (!establishAlignmentOrder()) return false;

	// load view maps

	int numMaps = (int)mViewPoints.size();
	if (!mViewMaps.loadSketch(sketchViews, sketchFolder)) return false;
	if (!mViewMaps.loadMap(mapFolder, numMaps, "pred")) return false;
	if (!mViewMaps.loadMask(mapFolder, numMaps)) return false;
	int mapSize = mViewMaps.mMapSize;

	// load & compute viewing matrices

	Eigen::Matrix4d projMat, imgMat;
	if (!MeshView::buildProjMatrix(projMat)) return false;
	if (!MeshView::buildImageMatrix(imgMat, mapSize, mapSize)) return false;

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

bool OptimizeViews::process(bool skipOptimization, bool symmetrization) {

	if (!skipOptimization) {
		int numIterations = 2;
		for (int iterID = 0; iterID < numIterations; iterID++) {
			cout << "================= Optimization " << iterID << " =================" << endl;

			string bkResultFolder = mResultFolder;
			mResultFolder = mResultFolder + "iter-" + to_string(iterID) + "/";
			if (!FileUtil::makedir(mResultFolder)) return false;

			if (iterID > 0 && !optimizePointCloud()) return false;
			if (!prunePointCloud()) return false;
			if (!extractPointCloud()) return false;
			if (!alignPointCloud()) return false;

			mResultFolder = bkResultFolder;
		}
	} else {
		if (!extractPointCloud()) return false;
	}

	if (!wrapPointCloud()) return false;
	if (!unionPointCloud()) return false;
	if (symmetrization) {
		if (!symmetrizePointCloud()) return false; // for airplane only
	}

	if (!skipOptimization) {
		if (!trimPointCloud()) return false;
		if (!reorientPointCloud()) return false;
	}

	if (!MeshIO::savePointSet(mResultFolder + "points.ply", mPointCloud)) return false;

	return true;
}

bool OptimizeViews::establishAlignmentOrder() {

	int numViews = (int)mViewPoints.size();

	vector<unordered_set<int>> neighbors(numViews, unordered_set<int>());
	for (vector<int> &group : mViewGroups) {
		for (int k = 0; k < (int)group.size(); k++) {
			int v1 = group[k];
			int v2 = group[(k + 1) % (int)group.size()];
			neighbors[v1].insert(v2);
			neighbors[v2].insert(v1);
		}
	}

	mViewNeighbors.resize(numViews);
	for (int viewID = 0; viewID < numViews; viewID++) {
		mViewNeighbors[viewID].assign(neighbors[viewID].begin(), neighbors[viewID].end());
	}

	vector<bool> visited(numViews, false);
	visited[0] = true;
	mViewOrder.assign(1, 0);
	int head = 0;
	while (head < (int)mViewOrder.size()) {
		for (int nb : neighbors[mViewOrder[head]]) {
			if (!visited[nb]) {
				mViewOrder.push_back(nb);
				visited[nb] = true;
			}
		}
		head++;
	}

	cout << "Fusing order: ";
	for (int view : mViewOrder) cout << view << " ";
	cout << endl;

	return true;
}

bool OptimizeViews::prunePointCloud() {

	int numViews = (int)mViewPoints.size();
	int numSketchViews = (int)mViewMaps.mSketches.size();
	int mapSize = mViewMaps.mMapSize;

	// prune by neighboring views

	int kernel = 3; // UNDONE: param mask dilation kernel size
	cout << "Pruning";
	for (int pruneViewID = numSketchViews; pruneViewID < numViews; pruneViewID++) { // HACK: don't prune sketch views
		//cout << "Pruning view " << pruneViewID << endl;
		cout << ".";

		Eigen::Matrix4d xform = mViewMatrices[pruneViewID].inverse();
		for (int h = 0; h < mapSize; h++) {
			for (int w = 0; w < mapSize; w++) {
				if (!mViewMaps.mMasks[pruneViewID][h][w]) continue;
				double d = mViewMaps.mDepths[pruneViewID][h][w];
				Eigen::Vector4d p = xform * Eigen::Vector4d((double)w, (double)h, d, 1.0);
				int pruneVote = 0;
				int keepVote = 0;
				for (int maskViewID : mViewNeighbors[pruneViewID]) {
					Eigen::Vector4d q = mViewMatrices[maskViewID] * p;
					int u = (int)q[0];
					int v = (int)q[1];
					bool valid = false;
					for (int uu = max(0, u - kernel); uu <= min(mapSize - 1, u + kernel); uu++) {
						for (int vv = max(0, v - kernel); vv <= min(mapSize - 1, v + kernel); vv++) {
							if (mViewMaps.mMasks[maskViewID][vv][uu]) {
								valid = true;
								break;
							}
						}
						if (valid) break;
					}
					int vote = 1;
					if (maskViewID < numSketchViews) vote = 3; // HACK: give more votes for sketch views
					if (valid) keepVote += vote;
					else pruneVote += vote;
				}
				if (pruneVote > keepVote) {
					mViewMaps.mMasks[pruneViewID][h][w] = false;
					mViewMaps.mDepths[pruneViewID][h][w] = 1.0f;
					mViewMaps.mNormals[pruneViewID][h][w] = vec3d(1.0, 1.0, 1.0);
				}
			}
		}
	}
	cout << endl;

	//for (int viewID = 0; viewID < numViews; viewID++) {
	//	if (!MapsData::cleanMask(mViewMaps.mMasks[viewID])) return false;
	//	if (!MapsData::visualizeMask(mResultFolder + "mask-" + to_string(viewID) + ".png", mViewMaps.mMasks[viewID])) return false;
	//}

	return true;
}

bool OptimizeViews::extractPointCloud() {

	int numViews = (int)mViewPoints.size();
	int mapSize = mViewMaps.mMapSize;

	mViewPointCloudsPosition.resize(numViews);
	mViewPointCloudsNormal.resize(numViews);

	cout << "Extracting";
	for (int viewID = 0; viewID < numViews; viewID++) {
		//cout << "Extracting point cloud for view " << viewID << endl;
		cout << ".";

		int numPoints = 0;
		for (int h = 0; h < mapSize; h++) {
			for (int w = 0; w < mapSize; w++) {
				if (mViewMaps.mMasks[viewID][h][w]) numPoints++;
			}
		}

		Eigen::Matrix3Xd &matP = mViewPointCloudsPosition[viewID];
		Eigen::Matrix3Xd &matN = mViewPointCloudsNormal[viewID];
		matP.resize(3, numPoints);
		matN.resize(3, numPoints);

		Eigen::Matrix4d xformMat = mViewMatrices[viewID].inverse();
		Eigen::Matrix4d rotateMat = mRotateMatrices[viewID].transpose();
		int pointID = 0;
		for (int h = 0; h < mapSize; h++) {
			for (int w = 0; w < mapSize; w++) {
				if (!mViewMaps.mMasks[viewID][h][w]) continue;

				double vd = mViewMaps.mDepths[viewID][h][w];
				vec3d vn = mViewMaps.mNormals[viewID][h][w];
				Eigen::Vector4d p = xformMat * Eigen::Vector4d((double)w, (double)h, vd, 1.0);
				Eigen::Vector4d n = rotateMat * Eigen::Vector4d(vn[0], vn[1], vn[2], 1.0);
				matP.col(pointID) = p.topRows(3);
				matN.col(pointID) = n.topRows(3);

				pointID++;
			}
		}
		matP = matP.cwiseMax(-MeshView::VIEW_RADIUS).cwiseMin(MeshView::VIEW_RADIUS);

		string shapeName = mResultFolder + to_string(viewID) + ".ply";
		if (!MapsData::visualizePoints(shapeName, matP, matN)) return false;
	}
	cout << endl;

	return true;
}

bool OptimizeViews::alignPointCloud() {

	Eigen::Matrix3Xd baseMatP = mViewPointCloudsPosition[mViewOrder[0]];
	Eigen::Matrix3Xd baseMatN = mViewPointCloudsNormal[mViewOrder[0]];

	cout << "Aligning";
	for (int orderID = 1; orderID < (int)mViewOrder.size(); orderID++) {
		int viewID = mViewOrder[orderID];
		//cout << "Aligning view " << viewID << endl;
		cout << ".";

		Eigen::Affine3d xform;
		xform.setIdentity();

		Eigen::Matrix3Xd viewMatP = mViewPointCloudsPosition[viewID];
		Eigen::Matrix3Xd viewMatN = mViewPointCloudsNormal[viewID];

		if (!MatchRigidICP::run(3, viewMatP, viewMatN, baseMatP, baseMatN, xform, true)) return false;

		// transform points
		viewMatP = xform * viewMatP;
		viewMatN = xform.rotation() * viewMatN;
		mViewPointCloudsPosition[viewID] = viewMatP;
		mViewPointCloudsNormal[viewID] = viewMatN;

		// add in points
		Eigen::Matrix3Xd extMatP(3, baseMatP.cols() + viewMatP.cols());
		Eigen::Matrix3Xd extMatN(3, baseMatN.cols() + viewMatN.cols());
		extMatP << baseMatP, viewMatP;
		extMatN << baseMatN, viewMatN;
		baseMatP.swap(extMatP);
		baseMatN.swap(extMatN);

		// add in transformation
		Eigen::Matrix4d alignMat = xform.inverse().matrix();
		Eigen::Matrix4d rotateMat = Eigen::Matrix4d::Identity();
		rotateMat.topLeftCorner(3, 3) = alignMat.topLeftCorner(3, 3);
		mViewMatrices[viewID] *= alignMat;
		mRotateMatrices[viewID] *= rotateMat;
	}
	cout << endl;

	if (!MapsData::visualizePoints(mResultFolder + "align.ply", baseMatP, baseMatN)) return false;

	return true;
}

bool OptimizeViews::wrapPointCloud() {

	int numWrapViews = (int)mViewMaps.mSketches.size();
	int numProcessViews = (int)mViewPoints.size();
	int mapSize = mViewMaps.mMapSize;

	cout << "Wrapping";
	for (int wrapViewID = 0; wrapViewID < numWrapViews; wrapViewID++) {
		cout << "*";
		vector<vector<bool>> &wrapMask = mViewMaps.mMasks[wrapViewID];

		// build Kd tree for mask point
		vector<vec3> maskPoints(0);
		maskPoints.reserve(mapSize*mapSize);
		vector<vec2i> maskIdx(0);
		maskIdx.reserve(mapSize*mapSize);
		for (int row = 0; row < mapSize; row++) {
			for (int col = 0; col < mapSize; col++) {
				if (wrapMask[row][col]) {
					maskPoints.push_back(vec3((float)col, (float)row, 0.0f));
					maskIdx.push_back(vec2i(row, col));
				}
			}
		}
		SKDTree tree;
		SKDTreeData treeData;
		if (!MeshKDTree::buildKdTree(maskPoints, tree, treeData)) return false;

		// compute offset
		vector<vector<vec2i>> wrapOffset(mapSize, vector<vec2i>(mapSize, vec2i(0, 0)));
#pragma omp parallel for
		for (int pixelID = 0; pixelID < mapSize*mapSize; pixelID++) {
			int row = pixelID / mapSize;
			int col = pixelID % mapSize;
			if (wrapMask[row][col]) {
				wrapOffset[row][col] = vec2i(0, 0);
			} else {
				SKDT::NamedPoint queryPoint((float)col, (float)row, 0.0f);
				Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(1);
				tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);
				int maskID = (int)tree.getElements()[queryResult[0].getIndex()].id;
				wrapOffset[row][col] = maskIdx[maskID] - vec2i(row, col);
			}
		}

		// wrap points for each view
		for (int processViewID = 0; processViewID < numProcessViews; processViewID++) {
			cout << ".";

			Eigen::Matrix3Xd &points = mViewPointCloudsPosition[processViewID];
			int numPoints = (int)points.cols();

			Eigen::Matrix4Xd homoPoints(4, numPoints);
			homoPoints << points, Eigen::MatrixXd::Ones(1, numPoints);

			Eigen::Matrix4Xd projPoints = mViewMatrices[wrapViewID] * homoPoints;
#pragma omp parallel for
			for (int pointID = 0; pointID < numPoints; pointID++) {
				int col = (int)projPoints(0, pointID);
				int row = (int)projPoints(1, pointID);
				if (col < 0 || col >= mapSize || row < 0 || row >= mapSize) continue;
				if (!wrapMask[row][col]) {
					vec2i offset = wrapOffset[row][col];
					projPoints(0, pointID) += offset[1];
					projPoints(1, pointID) += offset[0];
				}
			}
			homoPoints = mViewMatrices[wrapViewID].inverse() * projPoints;

			points = homoPoints.topRows(3);
		}
	}
	cout << endl;

	return true;
}

bool OptimizeViews::optimizePointCloud() {

	int numViews = (int)mViewPoints.size();
	int mapSize = mViewMaps.mMapSize;

	vector<vector<vector<bool>>> optimizeMasks(numViews);
	vector<vector<vector<double>>> optimizedDepths(numViews);
	vector<vector<vector<vec3d>>> optimizedNormals(numViews);

	if (!ProjectViews::init(mapSize)) return false;

	cout << "Optimizing";
	for (int viewID = 0; viewID < numViews; viewID++) {
		//cout << "Optimizing point cloud for view " << viewID << endl;
		cout << ".";

		int numNeighbors = (int)(mViewNeighbors[viewID].size());
		vector<vector<vector<bool>>> viewMasks(numNeighbors + 1);
		vector<vector<vector<double>>> viewDepths(numNeighbors + 1);
		vector<vector<vector<vec3d>>> viewNormals(numNeighbors + 1);
		viewMasks[0] = mViewMaps.mMasks[viewID];
		viewDepths[0] = mViewMaps.mDepths[viewID];
		viewNormals[0] = mViewMaps.mNormals[viewID];

		if (false) {
			if (!MapsData::visualizeMask(mResultFolder + "mask-orig.png", viewMasks[0])) return false;
			if (!MapsData::visualizeDepth(mResultFolder + "depth-orig.png", viewDepths[0])) return false;
			if (!MapsData::visualizeNormal(mResultFolder + "normal-orig.png", viewNormals[0])) return false;
		}

		for (int neighborID = 0; neighborID < numNeighbors; neighborID++) {
			int nbViewID = mViewNeighbors[viewID][neighborID];
			//cout << "Projecting " << nbViewID << " -> " << viewID << endl;

			Eigen::Matrix4d viewMat = mViewMatrices[viewID] * mViewMatrices[nbViewID].inverse();
			Eigen::Matrix4d rotMat = mRotateMatrices[viewID] * mRotateMatrices[nbViewID].transpose();

			ProjectViews pv;
			if (!pv.loadMaps(mViewMaps.mMasks[nbViewID], mViewMaps.mDepths[nbViewID], mViewMaps.mNormals[nbViewID])) return false;
			if (!pv.project(viewMat, rotMat, viewMasks[neighborID + 1], viewDepths[neighborID + 1], viewNormals[neighborID + 1])) return false;

			if (false) {
				if (!MapsData::visualizeMask(mResultFolder + "mask-proj.png", viewMasks[neighborID + 1])) return false;
				if (!MapsData::visualizeDepth(mResultFolder + "depth-proj.png", viewDepths[neighborID + 1])) return false;
				if (!MapsData::visualizeNormal(mResultFolder + "normal-proj.png", viewNormals[neighborID + 1])) return false;
				system("pause");
			}
		}

		// optimization
		if (!OptimizeDepths::optimize(
			viewMasks, viewDepths, viewNormals, mViewMaps.mMaskProbs[viewID],
			optimizeMasks[viewID], optimizedDepths[viewID], optimizedNormals[viewID])) return false;
	}
	cout << endl;

	if (!ProjectViews::finish()) return false;

	// output optimized results
	for (int viewID = 0; viewID < numViews; viewID++) {
		mViewMaps.mMasks[viewID].swap(optimizeMasks[viewID]);
		mViewMaps.mDepths[viewID].swap(optimizedDepths[viewID]);
		mViewMaps.mNormals[viewID].swap(optimizedNormals[viewID]);
	}

	return true;
}

bool OptimizeViews::unionPointCloud() {

	cout << "Combining point clouds..." << endl;

	int numViews = (int)mViewPoints.size();

	mPointCloud.positions.clear();
	mPointCloud.normals.clear();
	for (int viewID = 0; viewID < numViews; viewID++) {
		int numPoints = (int)mViewPointCloudsPosition[viewID].cols();
		for (int pointID = 0; pointID < numPoints; pointID++) {
			Eigen::Vector3d p = mViewPointCloudsPosition[viewID].col(pointID);
			Eigen::Vector3d n = mViewPointCloudsNormal[viewID].col(pointID);
			mPointCloud.positions.push_back(vec3d(p[0], p[1], p[2]));
			mPointCloud.normals.push_back(vec3d(n[0], n[1], n[2]));
		}
	}
	mPointCloud.amount = (int)mPointCloud.positions.size();

	return true;
}

bool OptimizeViews::symmetrizePointCloud() {

	cout << "Symmetrizing point clouds..." << endl;

	int numPoints = mPointCloud.amount;

	// flip point cloud horizontally

	TPointSet flippedPointCloud = mPointCloud;
	vec3 center(0.0f, 0.0f, 0.0f);
	for (vec3 pos : mPointCloud.positions) {
		center += pos;
	}
	center *= 1.0f / numPoints;
	for (vec3 &position : flippedPointCloud.positions) {
		position[0] = center[0]*2.0f - position[0];
	}
	for (vec3 &normal : flippedPointCloud.normals) {
		normal[0] = -normal[0];
	}

	// align point cloud

	Eigen::Matrix3Xd matSP(3, numPoints);
	Eigen::Matrix3Xd matSN(3, numPoints);
	Eigen::Matrix3Xd matTP(3, numPoints);
	Eigen::Matrix3Xd matTN(3, numPoints);

	for (int sampleID = 0; sampleID < numPoints; sampleID++) {
		vec3 p = flippedPointCloud.positions[sampleID];
		vec3 n = flippedPointCloud.normals[sampleID];
		matSP.col(sampleID) = Eigen::Vector3d(p[0], p[1], p[2]);
		matSN.col(sampleID) = Eigen::Vector3d(n[0], n[1], n[2]);
	}
	for (int sampleID = 0; sampleID < numPoints; sampleID++) {
		vec3 p = mPointCloud.positions[sampleID];
		vec3 n = mPointCloud.normals[sampleID];
		matTP.col(sampleID) = Eigen::Vector3d(p[0], p[1], p[2]);
		matTN.col(sampleID) = Eigen::Vector3d(n[0], n[1], n[2]);
	}

	Eigen::Affine3d xform;
	xform.setIdentity();
	if (!MatchRigidICP::run(10, matSP, matSN, matTP, matTN, xform)) return false;
	//if (!MatchRigidICP::visualize("symmetrize.ply", matSP, matTP, xform)) return false;

	// append symmetrized points

	Eigen::Matrix3d rotation = xform.rotation();
	for (int sampleID = 0; sampleID < numPoints; sampleID++) {
		vec3 p = flippedPointCloud.positions[sampleID];
		vec3 n = flippedPointCloud.normals[sampleID];
		Eigen::Vector3d pv(p[0], p[1], p[2]);
		Eigen::Vector3d nv(n[0], n[1], n[2]);
		pv = xform * pv;
		nv = rotation * nv;
		mPointCloud.positions.push_back(vec3d(pv[0], pv[1], pv[2]));
		mPointCloud.normals.push_back(vec3d(nv[0], nv[1], nv[2]));
	}
	mPointCloud.amount = (int)mPointCloud.positions.size();

	return true;
}

bool OptimizeViews::trimPointCloud() {

	cout << "Trimming point clouds..." << endl;

	int numPoints = mPointCloud.amount;
	vector<bool> pointFlags(numPoints, true);

	// check sparse points

	SKDTree tree;
	SKDTreeData treeData;
	if (!MeshKDTree::buildKdTree(mPointCloud.positions, tree, treeData)) return false;

	vec3 bsCenter;
	float bsRadius;
	if (!MeshCompute::computeBoundingSphere(mPointCloud, bsCenter, bsRadius)) return false;
	double distBound = bsRadius * 0.05;
	int inspectNeighbors = 10;

	vector<double> pointDists(numPoints, 0.0);
#pragma omp parallel for
	for (int pointID = 0; pointID < numPoints; pointID++) {
		vec3 point = mPointCloud.positions[pointID];
		SKDT::NamedPoint queryPoint(point[0], point[1], point[2]);
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(inspectNeighbors);
		tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult, distBound);
		if (queryResult.size() < inspectNeighbors) {
			pointDists[pointID] = DBL_MAX;
		} else {
			int nbID = (int)tree.getElements()[queryResult[queryResult.size() - 1].getIndex()].id;
			double nbDist = (mPointCloud.positions[nbID] - point).length();
			//double nbDist = 0;
			//for (int queryID = 0; queryID < queryResult.size(); queryID++) {
			//	int nbID = (int)tree.getElements()[queryResult[queryID].getIndex()].id;
			//	nbDist += (mPointCloud.positions[nbID] - point).length();
			//}
			//if (nbDist) nbDist /= queryResult.size();
			//else nbDist = DBL_MAX;
			pointDists[pointID] = nbDist;
		}
	}

	vector<double> tmpDists = pointDists;
	int midPoint = numPoints / 2; // UNDONE: trimming parameters
	nth_element(tmpDists.begin(), tmpDists.begin() + midPoint, tmpDists.end());
	double midDist = tmpDists[midPoint];
	double distThreshold = midDist * 2.0;

#pragma omp parallel for
	for (int pointID = 0; pointID < numPoints; pointID++) {
		if (pointDists[pointID] > distThreshold) {
			pointFlags[pointID] = false;
		}
	}

	// update points

	vector<vec3> newPositions(0), newNormals(0);
	for (int pointID = 0; pointID < numPoints; pointID++) {
		if (pointFlags[pointID]) {
			newPositions.push_back(mPointCloud.positions[pointID]);
			newNormals.push_back(mPointCloud.normals[pointID]);
		}
	}
	mPointCloud.positions.swap(newPositions);
	mPointCloud.normals.swap(newNormals);
	mPointCloud.amount = (int)mPointCloud.positions.size();
	numPoints = mPointCloud.amount;
	if (!MeshIO::savePointSet(mResultFolder + "trim.ply", mPointCloud)) return false;

	return true;
}

bool OptimizeViews::reorientPointCloud() {

	cout << "Reorienting point clouds..." << endl;

	TTriangleMesh envelop;
	MeshEnvelope me(&mPointCloud);
	if (!me.process()) return false;
	if (!me.output(envelop)) return false;
	if (!MeshIO::saveMesh(mResultFolder + "envelop.ply", envelop)) return false;

	TSampleSet envelopSamples;
	SampleSimplePoissonDisk sspd(&envelop);
	if (!sspd.runSampling(10000)) return false;
	if (!sspd.exportSample(envelopSamples)) return false;

	SKDTree envelopTree;
	SKDTreeData envelopTreeData;
	if (!MeshKDTree::buildKdTree(envelopSamples.positions, envelopTree, envelopTreeData)) return false;

#pragma omp parallel for
	for (int pointID = 0; pointID < mPointCloud.amount; pointID++) {
		vec3 point = mPointCloud.positions[pointID];
		vec3 normal = mPointCloud.normals[pointID];
		SKDT::NamedPoint queryPoint(point[0], point[1], point[2]);
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(1);
		envelopTree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);
		int nbID = (int)envelopTree.getElements()[queryResult[0].getIndex()].id;
		if (cml::dot(envelopSamples.normals[nbID], mPointCloud.normals[pointID]) < 0) {
			mPointCloud.normals[pointID] = -mPointCloud.normals[pointID];
		}
	}

	return true;
}

bool OptimizeViews::error(string s) {

	cout << "Error: " << s << endl;
	system("pause");

	return false;
}