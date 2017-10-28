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


#include "MatchSimpleICP.h"

#include <vector>
#include <set>

#include "Library/CMLHelper.h"

#include "Format/PlyExporter.h"

using namespace Monster;

bool MatchSimpleICP::initAlignment(
	Eigen::Matrix3Xd &source,
	Eigen::Matrix3Xd &target,
	Eigen::Affine3d &transformation) {

	// align center and scale
	Eigen::MatrixXd matS = transformation*source;
	Eigen::Vector3d centerS = matS.rowwise().mean();
	matS.colwise() -= centerS;
	Eigen::Vector3d centerT = target.rowwise().mean();
	Eigen::MatrixXd matT = target.colwise() - centerT;
	Eigen::Vector3d scaleS = matS.rowwise().norm();
	Eigen::Vector3d scaleT = matT.rowwise().norm();
	Eigen::Vector3d scale = scaleT.array() / scaleS.array();
	scale = (scaleS.array() == 0).select(1.0, scale); // special handling for flat patches
	scale = (scaleT.array() == 0).select(1.0, scale);
	scale = (scale.array().abs() < 0.1).select(1.0, scale);
	scale = (scale.array().abs() > 10).select(1.0, scale);
	transformation.pretranslate(-centerS);
	transformation.prescale(scale);
	transformation.pretranslate(centerT);

	return true;
}

bool MatchSimpleICP::run(
	int iteration,
	Eigen::Matrix3Xd &sourceP,
	Eigen::Matrix3Xd &sourceN,
	Eigen::Matrix3Xd &targetP,
	Eigen::Matrix3Xd &targetN,
	Eigen::Affine3d &transformation,
	bool aligned)
{

	// initialization
	SKDTree tree;
	SKDTreeData treeData;
	if (!buildKDTree(targetP, tree, treeData)) return false;
	if (!aligned && !initAlignment(sourceP, targetP, transformation)) return false;
	Eigen::Affine3d initialTransformation = transformation;

	// ICP iteration
	double lastError = DBL_MAX;
	for (int iterID = 0; iterID<iteration; iterID++) {

		// find nearest neighbors
		Eigen::Matrix3d rotation = transformation.rotation();
		Eigen::Matrix3Xd matXSP = transformation * sourceP;
		Eigen::Matrix3Xd matXSN = rotation * sourceN;
		Eigen::Matrix3Xd matTP, matTN;
		vector<int> slices;
		if (!findNearestNeighbors(tree, matXSP, slices)) return false;
		if (!sliceMatrices(targetP, slices, matTP)) return false;
		if (!sliceMatrices(targetN, slices, matTN)) return false;
		if (!findMatchedNeighbors(matXSP, matXSN, matTP, matTN, slices, false)) return false;
		if (slices.empty()) break; // no matched points
		if (!sliceMatrices(matXSP, slices, matXSP)) return false;
		if (!sliceMatrices(matXSN, slices, matXSN)) return false;
		if (!sliceMatrices(matTP, slices, matTP)) return false;
		if (!sliceMatrices(matTN, slices, matTN)) return false;

		// align matched points
		Eigen::Affine3d newTransformation;
		if (!extractTransformation(matXSP, matTP, newTransformation)) return false;
		transformation = newTransformation * transformation;

		/*
		// visualize each iteration
		if (!visualize("iter-match.ply", matXSP, matTP, newTransformation)) return false;
		if (!visualize("iter-all.ply", sourceP, targetP, transformation)) return false;
		system("pause");
		//*/

		matXSP = transformation * sourceP;
		double currentError;
		if (!error(targetP, matXSP, currentError)) return false;
		if (currentError > lastError*0.99) break; // UNDONE: param ICP convergence threshold
		lastError = currentError;
	}

	if (!transformation.matrix().allFinite()) {
		transformation = initialTransformation;
		if (!initAlignment(sourceP, targetP, transformation)) return false;
	}

	return true;
}

bool MatchSimpleICP::error(
	Eigen::Matrix3Xd &source,
	Eigen::Matrix3Xd &target,
	double &outError)
{
	SKDTree tree;
	SKDTreeData treeData;
	if (!buildKDTree(target, tree, treeData)) return false;

	Eigen::Matrix3Xd matched;
	vector<int> neighbors;
	if (!findNearestNeighbors(tree, source, neighbors)) return false;
	if (!sliceMatrices(target, neighbors, matched)) return false;
	outError = (matched - source).squaredNorm() / source.cols();

	return true;
}

bool MatchSimpleICP::buildKDTree(
	Eigen::Matrix3Xd &points,
	SKDTree &tree,
	SKDTreeData &data)
{

	data.resize(points.cols());
	for (int i = 0; i<(int)points.cols(); i++) {
		data[i] = SKDT::NamedPoint((float)points(0, i), (float)points(1, i), (float)points(2, i), (size_t)i);
	}
	tree.init(data.begin(), data.end());

	return true;
}

bool MatchSimpleICP::findNearestNeighbors(
	SKDTree &tree,
	Eigen::Matrix3Xd &inPoints,
	vector<int> &outIndices)
{

	outIndices.resize(inPoints.cols());
#pragma omp parallel for
	for (int j = 0; j < inPoints.cols(); j++) {
		SKDT::NamedPoint queryPoint((float)inPoints(0, j), (float)inPoints(1, j), (float)inPoints(2, j));
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(1);
		tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);
		if (queryResult.isEmpty()) outIndices[j] = 0; // too far away, whatever...
		else outIndices[j] = (int)tree.getElements()[queryResult[0].getIndex()].id;
	}

	return true;
}

bool MatchSimpleICP::findMatchedNeighbors(
	Eigen::Matrix3Xd &inSourceP,
	Eigen::Matrix3Xd &inSourceN,
	Eigen::Matrix3Xd &inTargetP,
	Eigen::Matrix3Xd &inTargetN,
	vector<int> &outIndices,
	bool aligned)
{
	if (inSourceP.cols() == 0) return true; // empty

	const double rejectDistanceThreshold = 5.0;

	Eigen::ArrayXd vecD = (inSourceP - inTargetP).colwise().norm().array();
	Eigen::ArrayXd vecN = (inSourceN.transpose() * inTargetN).diagonal().array().abs();
	double maxDist;
	if (aligned) {
		// use alpha times bounding box diagonal length as clamping distance (alpha = 0.05?)
		double bbLength = (inSourceP.rowwise().maxCoeff() - inSourceP.rowwise().minCoeff()).norm();
		maxDist = bbLength * 0.05; // UNDONE: param percentage of bounding box side length as filter distance
	} else {
		// use r times median length as clamping distance (r = 5?)
		vector<double> vDist(vecD.data(), vecD.data() + vecD.size());
		nth_element(vDist.begin(), vDist.begin() + vecD.size() / 2, vDist.end());
		maxDist = vDist[vecD.size() / 2] * rejectDistanceThreshold;
	}

	auto filter = vecD < maxDist && vecN > 0.5;
	outIndices.clear();
	outIndices.reserve((int)filter.count());
	for (int j = 0; j < filter.size(); j++) { // don't parallelize
		if (filter(j)) {
			outIndices.push_back(j);
		}
	}

	return true;
}

bool MatchSimpleICP::sliceMatrices(
	Eigen::Matrix3Xd &inMatrix,
	vector<int> &inIndices,
	Eigen::Matrix3Xd &outMatrix)
{
	Eigen::Matrix3Xd tmpMatrix;
	tmpMatrix.resize(inMatrix.rows(), inIndices.size());
	for (int j = 0; j < (int)inIndices.size(); j++) {
		tmpMatrix.col(j) = inMatrix.col(inIndices[j]);
	}
	outMatrix.swap(tmpMatrix);

	return true;
}

bool MatchSimpleICP::extractTransformation(
	Eigen::Matrix3Xd &source,
	Eigen::Matrix3Xd &target,
	Eigen::Affine3d &transformation)
{
	/*
	// classical ICP solution

	Eigen::Vector3d vecSCenter = source.rowwise().mean();
	Eigen::Vector3d vecTCenter = target.rowwise().mean();
	Eigen::Matrix3Xd transSource = source.colwise() - vecSCenter;
	Eigen::Matrix3Xd transTarget = target.colwise() - vecTCenter;
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(transTarget*transSource.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d matRotate = svd.matrixU() * svd.matrixV().transpose();
	Eigen::Vector3d vecTranslate = vecTCenter - matRotate * vecSCenter;
	transformation.setIdentity();
	transformation.prerotate(matRotate);
	transformation.pretranslate(vecTranslate);
	*/

	
	// least square solution of finding optimal affine transformation

	Eigen::Vector3d vecSCenter = source.rowwise().mean();
	Eigen::Vector3d vecTCenter = target.rowwise().mean();
	Eigen::Matrix3Xd transSource = source.colwise() - vecSCenter;
	Eigen::Matrix3Xd transTarget = target.colwise() - vecTCenter;

	Eigen::ColPivHouseholderQR<Eigen::MatrixX3d> solver(transSource.transpose());
	Eigen::Matrix3d affine = solver.solve(transTarget.transpose());
	transformation.setIdentity();
	transformation.linearExt() = affine.transpose();
	transformation.translate(-vecSCenter);
	transformation.pretranslate(vecTCenter);
	

	/*
	// classical ICP solution

	Eigen::Vector3d vecSCenter = source.rowwise().mean();
	Eigen::Vector3d vecTCenter = target.rowwise().mean();
	Eigen::Matrix3Xd transSource = source.colwise() - vecSCenter;
	Eigen::Matrix3Xd transTarget = target.colwise() - vecTCenter;
	Eigen::Vector3d vecSScale = transSource.rowwise().norm();
	Eigen::Vector3d vecTScale = transTarget.rowwise().norm();
	Eigen::Vector3d vecScale = vecTScale.array() / vecSScale.array();
	vecScale = (vecSScale.array() == 0).select(1.0, vecScale); // special handling for flat patches
	vecScale = (vecTScale.array() == 0).select(1.0, vecScale);
	vecScale = (vecScale.array().abs() < 0.1).select(1.0, vecScale);
	vecScale = (vecScale.array().abs() > 10).select(1.0, vecScale);
	Eigen::Matrix3Xd scaleSource = vecScale.asDiagonal() * transSource;
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(transTarget*scaleSource.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d matRotate = svd.matrixU() * svd.matrixV().transpose();
	Eigen::Vector3d vecTranslate = vecTCenter - matRotate * vecScale.asDiagonal() * vecSCenter;
	transformation.setIdentity();
	transformation.prescale(vecScale);
	transformation.prerotate(matRotate);
	transformation.pretranslate(vecTranslate);
	*/
	return true;
}

bool MatchSimpleICP::visualize(
	string filename,
	Eigen::Matrix3Xd &source,
	Eigen::Matrix3Xd &target,
	Eigen::Affine3d &transformation)
{

	vector<vec3> sourcePoints(source.cols());
	vector<vec3> targetPoints(target.cols());

	for (int j = 0; j < source.cols(); j++) {
		sourcePoints[j] = vec3d(source(0, j), source(1, j), source(2, j));
	}
	for (int j = 0; j < target.cols(); j++) {
		targetPoints[j] = vec3d(target(0, j), target(1, j), target(2, j));
	}

	matrix4d mat(
		transformation(0, 0), transformation(0, 1), transformation(0, 2), transformation(0, 3),
		transformation(1, 0), transformation(1, 1), transformation(1, 2), transformation(1, 3),
		transformation(2, 0), transformation(2, 1), transformation(2, 2), transformation(2, 3),
		0.0, 0.0, 0.0, 1.0);

	PlyExporter pe;
	if (!pe.addPoint(&sourcePoints, 0, mat, vec3i(255, 0, 0))) return false;
	if (!pe.addPoint(&targetPoints, 0, cml::identity_4x4(), vec3i(0, 255, 0))) return false;
	if (!pe.output(filename)) return false;

	return true;
}