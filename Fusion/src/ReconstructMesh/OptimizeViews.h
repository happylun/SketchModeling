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

#include <vector>
#include <string>

#include "Library/CMLHelper.h"
#include "Library/EigenHelper.h"

#include "MapsData.h"

#include "Mesh/MeshKDTree.h"
#include "Type/MonsterTypes.h"

using namespace std;

namespace Monster {

	class OptimizeViews {

	public:

		OptimizeViews() {}
		~OptimizeViews() {}

	public:

		bool init(
			string &sketchViews,
			string &sketchFolder,
			string &mapFolder,
			string &resultFolder,
			string &viewPointFile);
		bool process(bool skipOptimization = false, bool symmetrization = false);

	private:

		bool establishAlignmentOrder();
		bool prunePointCloud();
		bool extractPointCloud();
		bool alignPointCloud();
		bool wrapPointCloud();
		bool optimizePointCloud();

		bool unionPointCloud();
		bool symmetrizePointCloud();
		bool trimPointCloud();
		bool reorientPointCloud();

	private:

		bool error(string s);

	private:

		vector<Eigen::Vector3d> mViewPoints;
		vector<vector<int>> mViewGroups;
		vector<vector<int>> mViewNeighbors;
		vector<int> mViewOrder;
		vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mViewMatrices; // object space -> image space
		vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mRotateMatrices; // object space -> image space

		MapsData mViewMaps;
		vector<Eigen::Matrix3Xd> mViewPointCloudsPosition; // object space
		vector<Eigen::Matrix3Xd> mViewPointCloudsNormal; // object space
		TPointSet mPointCloud; // union of points from all views

		string mResultFolder;
	};

}