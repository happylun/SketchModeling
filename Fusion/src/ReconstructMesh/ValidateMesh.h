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

#include "MapsData.h"

#include "Library/EigenHelper.h"

#include "Type/MonsterTypes.h"

using namespace std;

namespace Monster {

	class ValidateMesh {

	private:

		static const double VOXELIZE_RANGE;
		static const int VOXELIZE_RESOLUTION;

	public:

		ValidateMesh() {}
		~ValidateMesh() {}

	public:

		bool loadViewPoint(string &viewPointFile);
		bool loadGroundTruth(string &meshFile, string &mapFolder);
		bool loadPredictMesh(string &meshFile);
		bool loadPredictVoxels(string &voxelFile, string outMeshFile = "");

		bool process();

		bool output(vector<double> &metrics);

	private:

		bool alignMesh();
		bool validatePoints();
		bool validateMaps();
		bool validateVoxels();

		bool error(string s);

	private:

		TTriangleMesh mMesh;
		vector<vector<vector<bool>>> mVoxels;

		MapsData mGTMaps;
		TTriangleMesh mGTMesh;
		vector<vector<vector<bool>>> mGTVoxels;

		vector<Eigen::Vector3d> mViewPoints;
		vector<vector<int>> mViewGroups;

		vector<double> mMetrics;
	};

}