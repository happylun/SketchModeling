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

#include "Type/MonsterTypes.h"

namespace Monster {

	class MeshCompute {

	private:

		// make it non-instantiable
		MeshCompute() {}
		~MeshCompute() {}

	public:

		static bool recomputeNormals(TTriangleMesh &mesh);
		static bool recomputeNormals(vector<vec3> &points, vector<vec3> &normals, int numNeighbors = 20, double neighborDist = 0.1);
		static bool computeDihedralAngle(vec3 center1, vec3 normal1, vec3 center2, vec3 normal2, double &angle);
		static bool computeAABB(TPointSet &mesh, vec3 &bbMin, vec3 &bbMax);
		static bool computeBoundingSphere(TPointSet &mesh, vec3 &center, float &radius);
		static bool computeFaceArea(TTriangleMesh &mesh, double &area);
		static bool computeMassCenter(TTriangleMesh &mesh, vec3 &center);
	};
}