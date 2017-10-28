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


#pragma once

#include <string>

#include "Library/EigenHelper.h"
#include "Library/TheaKDTreeHelper.h"

using namespace std;

namespace Monster {

	class MatchSimpleICP {

	private:

		MatchSimpleICP() {}
		~MatchSimpleICP() {}

	public:

		// source: 3xNs source point sets
		// target: 3xNt target point sets
		// transformation: initial and output transformation
		static bool initAlignment(
			Eigen::Matrix3Xd &source,
			Eigen::Matrix3Xd &target,
			Eigen::Affine3d &transformation);

		// sourceP: 3xN source point positions
		// sourceN: 3xN source point normals
		// targetP: 3xN target point positions
		// targetN: 3xN target point normals
		// transformation: initial and output transformation
		// alignment: whether `source' and `target' is roughly aligned through `transformation'
		static bool run(
			int iteration,
			Eigen::Matrix3Xd &sourceP,
			Eigen::Matrix3Xd &sourceN,
			Eigen::Matrix3Xd &targetP,
			Eigen::Matrix3Xd &targetN,
			Eigen::Affine3d &transformation,
			bool aligned=false);

		// source: 3xNs source point sets
		// target: 3xNt target point sets
		// error: average squared distance to closest point
		static bool error(
			Eigen::Matrix3Xd &source,
			Eigen::Matrix3Xd &target,
			double &outError);

		// filename: output .ply file
		// source: untransformed source point positions
		// target: target point positions
		// transformation: ICP result
		static bool visualize(
			string filename,
			Eigen::Matrix3Xd &source,
			Eigen::Matrix3Xd &target,
			Eigen::Affine3d &transformation);

	private:

		static bool buildKDTree(
			Eigen::Matrix3Xd &points,
			SKDTree &tree,
			SKDTreeData &data);

		static bool findNearestNeighbors(
			SKDTree &tree,
			Eigen::Matrix3Xd &inPoints,
			vector<int> &outIndices);

		static bool findMatchedNeighbors(
			Eigen::Matrix3Xd &inSourceP,
			Eigen::Matrix3Xd &inSourceN,
			Eigen::Matrix3Xd &inTargetP,
			Eigen::Matrix3Xd &inTargetN,
			vector<int> &outIndices,
			bool aligned = false);

		static bool sliceMatrices(
			Eigen::Matrix3Xd &inMatrix,
			vector<int> &inIndices,
			Eigen::Matrix3Xd &outMatrix);

		static bool extractTransformation(
			Eigen::Matrix3Xd &source,
			Eigen::Matrix3Xd &target,
			Eigen::Affine3d &transformation);
	};
}