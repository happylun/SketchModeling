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


#include "Match2DRigidICP.h"

#include <vector>
#include <set>

#include "Library/CMLHelper.h"

#include "Format/PlyExporter.h"

using namespace Monster;

bool Match2DRigidICP::initAlignment(
	Eigen::Matrix2Xd &source,
	Eigen::Matrix2Xd &target,
	Eigen::Affine2d &transformation)
{

	// align center
	Eigen::Vector2d centerS = (transformation*source).rowwise().mean();
	Eigen::Vector2d centerT = target.rowwise().mean();
	transformation.pretranslate(centerT-centerS);

	return true;
}

bool Match2DRigidICP::run(
	int iteration,
	Eigen::Matrix2Xd &source,
	Eigen::Matrix2Xd &target,
	Eigen::Affine2d &transformation,
	bool aligned)
{

	// initialization
	SKDTree tree;
	SKDTreeData treeData;
	if (!buildKDTree(target, tree, treeData)) return false;
	if (!aligned && !initAlignment(source, target, transformation)) return false;
	Eigen::Affine2d initialTransformation = transformation;

	// ICP iteration
	double lastError = DBL_MAX;
	for (int iterID = 0; iterID<iteration; iterID++) {

		// find nearest neighbors
		Eigen::Matrix2d rotation = transformation.rotation();
		Eigen::Matrix2Xd matXS = transformation * source;
		Eigen::Matrix2Xd matT;
		vector<int> slices;
		if (!findNearestNeighbors(tree, matXS, slices)) return false;
		if (!sliceMatrices(target, slices, matT)) return false;
		if (!findMatchedNeighbors(matXS, matT, slices, false)) return false;
		if (slices.empty()) break; // no matched points
		if (!sliceMatrices(matXS, slices, matXS)) return false;
		if (!sliceMatrices(matT, slices, matT)) return false;

		// align matched points
		Eigen::Affine2d newTransformation;
		if (!extractTransformation(matXS, matT, newTransformation)) return false;
		transformation = newTransformation * transformation;

		/*
		// visualize each iteration
		if (!visualize("iter-match.ply", matXS, matT, newTransformation)) return false;
		if (!visualize("iter-all.ply", source, target, transformation)) return false;
		//system("pause");
		//*/

		//matXS = transformation * source;
		//double currentError;
		//if (!error(target, matXS, currentError)) return false;
		//if (currentError > lastError*0.99) break; // UNDONE: param ICP convergence threshold
		//lastError = currentError;
	}

	if (!transformation.matrix().allFinite()) {
		transformation = initialTransformation;
		if (!initAlignment(source, target, transformation)) return false;
	}

	return true;
}

bool Match2DRigidICP::error(
	Eigen::Matrix2Xd &source,
	Eigen::Matrix2Xd &target,
	double &outError)
{
	SKDTree tree;
	SKDTreeData treeData;
	if (!buildKDTree(target, tree, treeData)) return false;

	Eigen::Matrix2Xd matched;
	vector<int> neighbors;
	if (!findNearestNeighbors(tree, source, neighbors)) return false;
	if (!sliceMatrices(target, neighbors, matched)) return false;
	outError = (matched - source).squaredNorm() / source.cols();

	return true;
}

bool Match2DRigidICP::buildKDTree(
	Eigen::Matrix2Xd &points,
	SKDTree &tree,
	SKDTreeData &data)
{

	data.resize(points.cols());
	for (int i = 0; i<(int)points.cols(); i++) {
		data[i] = SKDT::NamedPoint((float)points(0, i), (float)points(1, i), 0.0f, (size_t)i);
	}
	tree.init(data.begin(), data.end());

	return true;
}

bool Match2DRigidICP::findNearestNeighbors(
	SKDTree &tree,
	Eigen::Matrix2Xd &inPoints,
	vector<int> &outIndices)
{

	outIndices.resize(inPoints.cols());
#pragma omp parallel for
	for (int j = 0; j < inPoints.cols(); j++) {
		SKDT::NamedPoint queryPoint((float)inPoints(0, j), (float)inPoints(1, j), 0.0f);
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(1);
		tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);
		if (queryResult.isEmpty()) outIndices[j] = 0; // too far away, whatever...
		else outIndices[j] = (int)tree.getElements()[queryResult[0].getIndex()].id;
	}

	return true;
}

bool Match2DRigidICP::findMatchedNeighbors(
	Eigen::Matrix2Xd &inSource,
	Eigen::Matrix2Xd &inTarget,
	vector<int> &outIndices,
	bool aligned)
{
	if (inSource.cols() == 0) return true; // empty

	const double rejectDistanceThreshold = 5.0;

	Eigen::ArrayXd vecD = (inSource - inTarget).colwise().norm().array();
	double maxDist;
	if (aligned) {
		// use alpha times bounding box diagonal length as clamping distance (alpha = 0.05?)
		double bbLength = (inSource.rowwise().maxCoeff() - inSource.rowwise().minCoeff()).norm();
		maxDist = bbLength * 0.05; // UNDONE: param percentage of bounding box side length as filter distance
	} else {
		// use r times median length as clamping distance (r = 5?)
		vector<double> vDist(vecD.data(), vecD.data() + vecD.size());
		nth_element(vDist.begin(), vDist.begin() + vecD.size() / 2, vDist.end());
		maxDist = vDist[vecD.size() / 2] * rejectDistanceThreshold;
	}

	auto filter = vecD < maxDist;
	outIndices.clear();
	outIndices.reserve((int)filter.count());
	for (int j = 0; j < filter.size(); j++) { // don't parallelize
		if (filter(j)) {
			outIndices.push_back(j);
		}
	}

	return true;
}

bool Match2DRigidICP::sliceMatrices(
	Eigen::Matrix2Xd &inMatrix,
	vector<int> &inIndices,
	Eigen::Matrix2Xd &outMatrix)
{
	Eigen::Matrix2Xd tmpMatrix;
	tmpMatrix.resize(inMatrix.rows(), inIndices.size());
	for (int j = 0; j < (int)inIndices.size(); j++) {
		tmpMatrix.col(j) = inMatrix.col(inIndices[j]);
	}
	outMatrix.swap(tmpMatrix);

	return true;
}

bool Match2DRigidICP::extractTransformation(
	Eigen::Matrix2Xd &source,
	Eigen::Matrix2Xd &target,
	Eigen::Affine2d &transformation)
{
	// classical ICP solution

	Eigen::Vector2d vecSCenter = source.rowwise().mean();
	Eigen::Vector2d vecTCenter = target.rowwise().mean();

	Eigen::Matrix2Xd transSource = source.colwise() - vecSCenter;
	Eigen::Matrix2Xd transTarget = target.colwise() - vecTCenter;
	Eigen::JacobiSVD<Eigen::Matrix2d> svd(transTarget*transSource.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix2d matRotate = svd.matrixU() * svd.matrixV().transpose();
	Eigen::Vector2d vecTranslate = vecTCenter - matRotate * vecSCenter;
	transformation.setIdentity();
	transformation.prerotate(matRotate);
	transformation.pretranslate(vecTranslate);

	//// HACK: no rotation
	//if (true) {
	//	Eigen::Vector2d vecTranslate = vecTCenter - vecSCenter;
	//	transformation.setIdentity();
	//	transformation.pretranslate(vecTranslate);
	//}

	return true;
}

bool Match2DRigidICP::visualize(
	string filename,
	Eigen::Matrix2Xd &source,
	Eigen::Matrix2Xd &target,
	Eigen::Affine2d &transformation)
{

	vector<vec3> sourcePoints(source.cols());
	vector<vec3> targetPoints(target.cols());

	for (int j = 0; j < source.cols(); j++) {
		sourcePoints[j] = vec3d(source(0, j), source(1, j), 0.0);
	}
	for (int j = 0; j < target.cols(); j++) {
		targetPoints[j] = vec3d(target(0, j), target(1, j), 0.0);
	}

	matrix4d mat(
		transformation(0, 0), transformation(0, 1), 0.0, transformation(0, 2),
		transformation(1, 0), transformation(1, 1), 0.0, transformation(1, 2),
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	PlyExporter pe;
	if (!pe.addPoint(&sourcePoints, 0, mat, vec3i(255, 0, 0))) return false;
	if (!pe.addPoint(&targetPoints, 0, cml::identity_4x4(), vec3i(0, 255, 0))) return false;
	if (!pe.output(filename)) return false;

	return true;
}