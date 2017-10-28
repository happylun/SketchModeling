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
#include <unordered_set>

#include "MapsData.h"

#include "Library/EigenHelper.h"

#include "Type/MonsterTypes.h"

using namespace std;

namespace Monster {

	class OptimizeMesh {

	public:

		OptimizeMesh() {}
		~OptimizeMesh() {}

	public:

		bool init(
			string &sketchViews,
			string &sketchFolder,
			string &mapFolder,
			string &resultFolder,
			string &viewPointFile);
		bool process();

	private:

		bool preProcessMesh();
		bool preProcessSketch();

		bool doStrokeMatching();
		bool doContourMatching();
		bool doDeformation();

		bool extractSketchContour();
		bool extractSketchStroke();
		bool extractMeshContour(int viewID, vector<int> &contourVertices);
		bool findShortestPath(vector<vector<pair<int, double>>> &graph, int start, int finish, vector<int> &path);

		bool error(string s);

	private:

		string mResultFolder;
		string mVisualFolder;

		TTriangleMesh mMesh;
		MapsData mViewMaps;

		vector<Eigen::Vector3d> mViewPoints;
		vector<vector<int>> mViewGroups;
		vector<EigenAA(Eigen::Matrix4d)> mViewMatrices; // object space -> sketch space
		vector<EigenAA(Eigen::Matrix4d)> mRotateMatrices; // object space -> sketch space

		vector<vector<vec2d>> mSketchContours; // pixel coordinate : # of contour pixels : # of views
		vector<vector<vector<vec2d>>> mSketchStrokes; // sample coordinate : # of stroke samples : # of strokes : # of views

		vector<vec3d> mContourHandles; // deformation handle position : # of vertices
		vector<bool> mContourFlags; // whether lying on contour : # of vertices

		vector<bool> mStrokeFlags; // whether lying on stroke : # of vertices

		vector<unordered_set<int>> mMeshGraph;
	};

}