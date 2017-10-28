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

#include "Mesh/MeshKDTree.h"
#include "Type/MonsterTypes.h"

using namespace std;

namespace Monster {

	class MapsData {

	public:

		MapsData();
		~MapsData();

	public:

		bool loadSketch(string &sketchViews, string &sketchFolder);
		bool loadMap(string &mapFolder, int numMaps, string prefix = "pred");
		bool loadMask(string &maskFolder, int numMaps);

	public:

		static bool parseSketch(string &imageName, vector<vector<double>> &sketch);
		static bool parseDepthNormal(
			string &imageName,
			vector<vector<bool>> &mask,
			vector<vector<double>> &depth,
			vector<vector<vec3d>> &normal);
		static bool parseMask(string &imageName, vector<vector<double>> &mask);
		static bool cleanMask(vector<vector<bool>> &mask);
		static bool computeNormalFromDepth(
			vector<vector<bool>> &mask,
			vector<vector<double>> &depth,
			vector<vector<vec3d>> &normal);

	public:

		static bool visualizePoints(string fileName, Eigen::Matrix3Xd &positions, Eigen::Matrix3Xd &normals);
		static bool visualizeSketch(string fileName, vector<vector<double>> &sketch);
		static bool visualizeMask(string fileName, vector<vector<bool>> &mask);
		static bool visualizeProb(string fileName, vector<vector<double>> &prob);
		static bool visualizeDepth(string fileName, vector<vector<double>> &depth);
		static bool visualizeNormal(string fileName, vector<vector<vec3d>> &normal);

	private:

		static bool error(string s);

	public:

		int mSketchSize;
		int mMapSize;

		vector<vector<vector<double>>> mSketches; // BG: 0.0
		vector<vector<vector<bool>>> mMasks; // BG: false
		vector<vector<vector<double>>> mMaskProbs; // BG: 0.0
		vector<vector<vector<double>>> mDepths; // BG: 1.0
		vector<vector<vector<vec3d>>> mNormals; // BG: (1.0, 1.0, 1.0)
	};

}