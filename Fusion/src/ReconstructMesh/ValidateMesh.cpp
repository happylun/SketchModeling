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


#include "ValidateMesh.h"

#include "RenderMesh.h"

#include "Mesh/MeshIO.h"
#include "Mesh/MeshView.h"
#include "Mesh/MeshVoxelize.h"
#include "Mesh/MeshCompute.h"
#include "Mesh/MeshClean.h"

#include "Sample/SampleSimplePoissonDisk.h"

#include "Match/MatchSimpleICP.h"

#include "Format/PlyExporter.h"

using namespace Monster;

//#define DEBUG_VISUALIZATION

const double ValidateMesh::VOXELIZE_RANGE = 2.5;
const int ValidateMesh::VOXELIZE_RESOLUTION = 128;

bool ValidateMesh::loadViewPoint(string &viewPointFile) {

	return MeshView::loadViewPoints(viewPointFile, mViewPoints, &mViewGroups);
}

bool ValidateMesh::loadGroundTruth(string &meshFile, string &mapFolder) {

	int numMaps = (int)mViewPoints.size();
	if (!MeshIO::loadMesh(meshFile, mGTMesh)) return false;
	if (!MeshClean::cleanUp(mGTMesh)) return false;
	if (!mGTMaps.loadMap(mapFolder, numMaps, "gt")) return false;

	// voxelization

	if (true) {
		double range = VOXELIZE_RANGE;
		int resolution = VOXELIZE_RESOLUTION;

		if (!MeshVoxelize::voxelize(mGTMesh, range, resolution, mGTVoxels)) return false;

#ifdef DEBUG_VISUALIZATION
		if (true) {
			TTriangleMesh voxMesh;
			if (!MeshVoxelize::voxels2mesh(mGTVoxels, range, voxMesh)) return false;
			if (!MeshIO::saveMesh("vox-GT.ply", voxMesh)) return false;
		}
#endif
	}

	return true;
}

bool ValidateMesh::loadPredictMesh(string &meshFile) {

	if (!MeshIO::loadMesh(meshFile, mMesh)) return false;
	if (!MeshClean::cleanUp(mMesh)) return false;
	if (!MeshCompute::recomputeNormals(mMesh)) return false;

	if (!alignMesh()) return false;

	// voxelization

	if (true) {
		double range = VOXELIZE_RANGE;
		int resolution = VOXELIZE_RESOLUTION;

		TTriangleMesh clampMesh = mMesh;
		for (vec3 &p : mMesh.positions) {
			for (int k = 0; k < 3; k++) {
				p[k] = cml::clamp(p[k], -(float)range, (float)range);
			}
		}
		if (!MeshVoxelize::voxelize(clampMesh, range, resolution, mVoxels)) return false;

#ifdef DEBUG_VISUALIZATION
		if (true) {
			TTriangleMesh voxMesh;
			if (!MeshVoxelize::voxels2mesh(mVoxels, range, voxMesh)) return false;
			if (!MeshIO::saveMesh("vox-mesh.ply", voxMesh)) return false;
		}
#endif
	}

	return true;
}

bool ValidateMesh::loadPredictVoxels(string &voxelFile, string outMeshFile) {

	double range = VOXELIZE_RANGE;
	int inResolution = 128;
	int outResolution = VOXELIZE_RESOLUTION;

	vector<vector<vector<bool>>> rawVoxels(inResolution, vector<vector<bool>>(inResolution, vector<bool>(inResolution, false)));

	ifstream file(voxelFile, ios::binary);
	int numValidVoxels = 0;
	for (int x = 0; x < inResolution; x++) {
		for (int y = 0; y < inResolution; y++) {
			for (int z = 0; z < inResolution; z++) {
				unsigned char c;
				file.read((char*)&c, 1);
				rawVoxels[x][y][z] = (c != 0);
				if (c != 0) numValidVoxels++;
			}
		}
	}
	file.close();
	cout << "Number of valid voxels = " << numValidVoxels << endl;

	mVoxels.assign(outResolution, vector<vector<bool>>(outResolution, vector<bool>(outResolution, false)));
	for (int x = 0; x < outResolution; x++) {
		int rx = cml::clamp((int)(x*inResolution / (double)outResolution), 0, inResolution - 1);
		for (int y = 0; y < outResolution; y++) {
			int ry = cml::clamp((int)(y*inResolution / (double)outResolution), 0, inResolution - 1);
			for (int z = 0; z < outResolution; z++) {
				int rz = cml::clamp((int)(z*inResolution / (double)outResolution), 0, inResolution - 1);
				mVoxels[x][y][z] = rawVoxels[rx][ry][rz];
			}
		}
	}

	if (!MeshVoxelize::voxels2mesh(mVoxels, range, mMesh)) return false;

	if (!outMeshFile.empty()) {
		if (!MeshIO::saveMesh(outMeshFile, mMesh)) return false;
	}

	return true;
}

bool ValidateMesh::process() {

	mMetrics.clear();
	if (!validatePoints()) return false;
	if (!validateMaps()) return false;
	if (!validateVoxels()) return false;

	cout << "Metrics:";
	for (double value : mMetrics) cout << "\t" << value;
	cout << endl;

#ifdef DEBUG_VISUALIZATION
	system("pause");
#endif
	
	return true;
}

bool ValidateMesh::alignMesh() {

	cout << "========== Aligning mesh ==========" << endl;

	int numSamples = 50000;
	double sampleRadius = 0.01;

	TSampleSet sampleMesh;
	SampleSimplePoissonDisk sspdMesh(&mMesh);
	if (!sspdMesh.runSampling(numSamples, sampleRadius)) return false;
	if (!sspdMesh.exportSample(sampleMesh)) return false;

	TSampleSet sampleGT;
	SampleSimplePoissonDisk sspdGT(&mGTMesh);
	if (!sspdGT.runSampling(numSamples, sampleRadius)) return false;
	if (!sspdGT.exportSample(sampleGT)) return false;
	
	int numM = sampleMesh.amount;
	int numG = sampleGT.amount;
	
	Eigen::Matrix3Xd matSP(3, numM);
	Eigen::Matrix3Xd matSN(3, numM);
	Eigen::Matrix3Xd matTP(3, numG);
	Eigen::Matrix3Xd matTN(3, numG);

	for (int sampleID = 0; sampleID < numM; sampleID++) {
		vec3 p = sampleMesh.positions[sampleID];
		vec3 n = sampleMesh.normals[sampleID];
		matSP.col(sampleID) = Eigen::Vector3d(p[0], p[1], p[2]);
		matSN.col(sampleID) = Eigen::Vector3d(n[0], n[1], n[2]);
	}
	for (int sampleID = 0; sampleID < numG; sampleID++) {
		vec3 p = sampleGT.positions[sampleID];
		vec3 n = sampleGT.normals[sampleID];
		matTP.col(sampleID) = Eigen::Vector3d(p[0], p[1], p[2]);
		matTN.col(sampleID) = Eigen::Vector3d(n[0], n[1], n[2]);
	}

	Eigen::Affine3d xform;
	xform.setIdentity();
	if (!MatchSimpleICP::run(10, matSP, matSN, matTP, matTN, xform)) return false;
#ifdef DEBUG_VISUALIZATION
	if (!MatchSimpleICP::visualize("align.ply", matSP, matTP, xform)) return false;
#endif

	Eigen::Matrix3d rotation = xform.rotation();
	for (int vertID = 0; vertID < mMesh.amount; vertID++) {
		vec3 p = mMesh.positions[vertID];
		vec3 n = mMesh.normals[vertID];
		Eigen::Vector3d pv(p[0], p[1], p[2]);
		Eigen::Vector3d nv(n[0], n[1], n[2]);
		pv = xform * pv;
		nv = rotation * nv;
		mMesh.positions[vertID] = vec3d(pv[0], pv[1], pv[2]);
		mMesh.normals[vertID] = vec3d(nv[0], nv[1], nv[2]);
	}

	return true;
}

bool ValidateMesh::validatePoints() {

	cout << "========== Validating points ==========" << endl;

	int numSamples = 50000;
	double sampleRadius = 0.01;

	TSampleSet sampleMesh;
	SampleSimplePoissonDisk sspdMesh(&mMesh);
	if (!sspdMesh.runSampling(numSamples, sampleRadius)) return false;
	if (!sspdMesh.exportSample(sampleMesh)) return false;
	//cout << "Num samples = " << sampleMesh.amount << endl;

	TSampleSet sampleGT;
	SampleSimplePoissonDisk sspdGT(&mGTMesh);
	if (!sspdGT.runSampling(numSamples, sampleRadius)) return false;
	if (!sspdGT.exportSample(sampleGT)) return false;
	//cout << "Num samples = " << sampleGT.amount << endl;

	vec3 gtMeshCenter;
	float gtMeshRadius;
	if (!MeshCompute::computeBoundingSphere(mGTMesh, gtMeshCenter, gtMeshRadius)) return false;

	SKDTree tree;
	SKDTreeData treeData;
	if (!MeshKDTree::buildKdTree(sampleGT.positions, tree, treeData)) return false;

	int numMeshPoints = sampleMesh.amount;
	Eigen::MatrixXd pointDists(numMeshPoints, 2);
#pragma omp parallel for
	for (int pointID = 0; pointID < numMeshPoints; pointID++) {
		vec3 point = sampleMesh.positions[pointID];
		vec3 normal = sampleMesh.normals[pointID];

		SKDT::NamedPoint queryPoint(point[0], point[1], point[2]);
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(1);
		tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);

		int gtPointID = (int)tree.getElements()[queryResult[0].getIndex()].id;
		vec3 pointGT = sampleGT.positions[gtPointID];
		vec3 normalGT = sampleGT.normals[gtPointID];

		double deltaP = (point-pointGT).length();
		double deltaN = 1.0 - fabs(cml::dot(normal, normalGT));

		pointDists(pointID, 0) = deltaP;
		pointDists(pointID, 1) = deltaN;
	}


#ifdef DEBUG_VISUALIZATION
	// visualize farthest point

	if (true) {
		int maxPointID;
		pointDists.col(1).maxCoeff(&maxPointID);
		vec3 p1 = sampleMesh.positions[maxPointID];

		SKDT::NamedPoint queryPoint(p1[0], p1[1], p1[2]);
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(1);
		tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);

		int matchPointID = (int)tree.getElements()[queryResult[0].getIndex()].id;
		vec3 p2 = sampleGT.positions[matchPointID];

		vector<vec3> edges = { p1, p2 };

		if (!MeshIO::savePointSet("points-mesh.ply", sampleMesh)) return false;
		if (!MeshIO::savePointSet("points-GT.ply", sampleGT)) return false;
		PlyExporter pe;
		if (!pe.addLine(&edges)) return false;
		if (!pe.output("farthest.ply")) return false;
	}
#endif

	double maxPDist = pointDists.col(0).maxCoeff() / gtMeshRadius;
	double avgPDist = pointDists.col(0).mean() / gtMeshRadius;
	double maxNDist = pointDists.col(1).maxCoeff();
	double avgNDist = pointDists.col(1).mean();
	cout << "Max point distance = " << maxPDist << ", " << maxNDist << endl;
	cout << "Average point distance = " << avgPDist << ", " << avgNDist << endl;

	mMetrics.push_back(maxPDist);
	mMetrics.push_back(maxNDist);
	mMetrics.push_back(avgPDist);
	mMetrics.push_back(avgNDist);

	return true;
}

bool ValidateMesh::validateMaps() {

	cout << "========== Validating maps ==========" << endl;

	int imgSize = mGTMaps.mMapSize;
	if (!RenderMesh::init(imgSize)) return false;

	Eigen::Matrix4d projMat;
	if (!MeshView::buildProjMatrix(projMat)) return false;

	int numViews = (int)mViewPoints.size();
	vec3d avgDelta(0.0, 0.0, 0.0);

	for (int viewID = 0; viewID < numViews; viewID++) {
		Eigen::Matrix4d viewMat;
		if (!MeshView::buildViewMatrix(mViewPoints[viewID], viewMat)) return false;

		Eigen::Matrix4f mvpMat = (projMat * viewMat).cast<float>();
		Eigen::Matrix4f rotMat;
		rotMat.setIdentity();
		rotMat.topLeftCorner(3, 3) = viewMat.topLeftCorner(3, 3).cast<float>();

		vector<vector<bool>> masks;
		vector<vector<double>> depths;
		vector<vector<vec3d>> normals;
		if (!RenderMesh::render(mMesh, mvpMat, rotMat, masks, depths, normals)) return false;

		double totalDeltaD = 0;
		double totalDeltaN = 0;
		int totalCount = 0;
		int countI = 0;
		int countU = 0;
		for (int row = 0; row < imgSize; row++) {
			for (int col = 0; col < imgSize; col++) {
				bool mp = masks[row][col];
				double dp = depths[row][col];
				vec3d np = normals[row][col];
				if (!mp) np = vec3d(0.0, 0.0, 1.0);

				bool mg = mGTMaps.mMasks[viewID][row][col];
				double dg = mGTMaps.mDepths[viewID][row][col];
				vec3d ng = mGTMaps.mNormals[viewID][row][col];
				if (!mg) ng = vec3d(0.0, 0.0, 1.0);

				vec3d delta(0.0, 0.0, 0.0);
				vec3i count(0, 0, 0);
				if (true) {
					double deltaD = fabs(dp - dg);
					double deltaN = 1.0 - cml::dot(cml::normalize(np), cml::normalize(ng));
					totalDeltaD += deltaD;
					totalDeltaN += deltaN;
					totalCount++;
				}
				if (mg & mp) countI++;
				if (mg || mp) countU++;
			}
		}
		if (totalCount) {
			totalDeltaD /= totalCount;
			totalDeltaN /= totalCount;
		}
		double totalDeltaM = 1.0 - countI / (double)countU;
		vec3d viewDelta(totalDeltaM, totalDeltaD, totalDeltaN);
		avgDelta += viewDelta;
		//cout << "View " << viewID << " error = " << viewDelta << endl;
	}
	avgDelta /= numViews;
	cout << "Average error = " << avgDelta << endl;

	if (!RenderMesh::finish()) return false;

	mMetrics.push_back(avgDelta[0]);
	mMetrics.push_back(avgDelta[1]);
	mMetrics.push_back(avgDelta[2]);

	return true;
}

bool ValidateMesh::validateVoxels() {

	cout << "========== Validating voxels ==========" << endl;

	double range = VOXELIZE_RANGE;
	int resolution = VOXELIZE_RESOLUTION;

	int countI = 0; // intersection
	int countUm = 0; // mesh only
	int countUg = 0; // GT only
	for (int x = 0; x < resolution; x++) {
		for (int y = 0; y < resolution; y++) {
			for (int z = 0; z < resolution; z++) {
				bool bm = mVoxels[x][y][z];
				bool bg = mGTVoxels[x][y][z];
				if (bm && bg) countI++;
				if (bm && !bg) countUm++;
				if (!bm && bg) countUg++;
			}
		}
	}
	int countU = countI + countUm + countUg;
	cout << "I = " << countI << ", Um = " << countUm << ", Ug = " << countUg << endl;

	double iou = (countI / (double)countU);
	cout << "Intersection over Union = " << iou << endl;

	mMetrics.push_back(1.0 - iou);

	return true;
}

bool ValidateMesh::output(vector<double> &metrics) {

	metrics = mMetrics;

	return true;
}

bool ValidateMesh::error(string s) {

	cout << "Error: " << s << endl;
	system("pause");

	return false;
}