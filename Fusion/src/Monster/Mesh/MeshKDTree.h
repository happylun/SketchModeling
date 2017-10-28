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

#include "Library/TheaKDTreeHelper.h"

#include "Type/MonsterTypes.h"

using namespace std;

namespace Monster {

	class MeshKDTree {

	private:

		// make it non-instantiable
		MeshKDTree() {}
		~MeshKDTree() {}

	public:

		static bool buildKdTree(vector<vec3> &points, SKDTree &tree, SKDTreeData &treeData); // kd tree of points
		static bool buildKdTree(TTriangleMesh &mesh, TKDTree &tree, TKDTreeData &treeData); // kd tree of face triangles
		static bool buildKdTree(TTriangleMesh &mesh, SKDTree &tree, SKDTreeData &treeData); // kd tree of face center points

		static bool distanceToPoints(vector<vec3> &points, SKDTree &tree, vec3 &point, double &distance);
		static bool distanceToMesh(TTriangleMesh &mesh, SKDTree &faceTree, vec3 &point, double &distance);
		static bool checkInsideMesh(TKDTree &tree, vec3 &point, bool &insideFlag, double eps = 1e-5);
	};
}