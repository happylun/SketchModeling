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

	class Match2DRigidICP {

	private:

		Match2DRigidICP() {}
		~Match2DRigidICP() {}

	public:

		// source: 2xNs source point sets
		// target: 2xNt target point sets
		// transformation: initial and output transformation
		static bool initAlignment(
			Eigen::Matrix2Xd &source,
			Eigen::Matrix2Xd &target,
			Eigen::Affine2d &transformation);

		// source: 2xNs source point positions
		// target: 2xNt target point positions
		// transformation: initial and output transformation
		// alignment: whether `source' and `target' is roughly aligned through `transformation'
		static bool run(
			int iteration,
			Eigen::Matrix2Xd &source,
			Eigen::Matrix2Xd &target,
			Eigen::Affine2d &transformation,
			bool aligned=false);

		// source: 2xNs source point sets
		// target: 2xNt target point sets
		// error: average squared distance to closest point
		static bool error(
			Eigen::Matrix2Xd &source,
			Eigen::Matrix2Xd &target,
			double &outError);

		// filename: output .ply file
		// source: untransformed source point positions
		// target: target point positions
		// transformation: ICP result
		static bool visualize(
			string filename,
			Eigen::Matrix2Xd &source,
			Eigen::Matrix2Xd &target,
			Eigen::Affine2d &transformation);

	private:

		static bool buildKDTree(
			Eigen::Matrix2Xd &points,
			SKDTree &tree,
			SKDTreeData &data);

		static bool findNearestNeighbors(
			SKDTree &tree,
			Eigen::Matrix2Xd &inPoints,
			vector<int> &outIndices);

		static bool findMatchedNeighbors(
			Eigen::Matrix2Xd &inSource,
			Eigen::Matrix2Xd &inTarget,
			vector<int> &outIndices,
			bool aligned = false);

		static bool sliceMatrices(
			Eigen::Matrix2Xd &inMatrix,
			vector<int> &inIndices,
			Eigen::Matrix2Xd &outMatrix);

		static bool extractTransformation(
			Eigen::Matrix2Xd &source,
			Eigen::Matrix2Xd &target,
			Eigen::Affine2d &transformation);
	};
}