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

#include "Library/CMLHelper.h"

using namespace std;

namespace Monster {

	struct TPointSet {
		vector<vec3> positions; // sample point position vector : # of points
		vector<vec3> normals;   // sample point normal vector : # of points
		int amount; // number of points
		TPointSet() : amount(0) {}
		TPointSet(const TPointSet &ps) :
			positions(ps.positions),
			normals(ps.normals),
			amount(ps.amount)
		{}
	};

	struct TSampleSet : TPointSet {
		float radius; // minimum distance between sample points
		vector<int> indices; // parent ID (face ID or edge ID) : # of samples
		TSampleSet() {}
		TSampleSet(const TPointSet &ps) :
			TPointSet(ps)
		{}
		TSampleSet(const TSampleSet &ss) :
			TPointSet(ss),
			indices(ss.indices),
			radius(ss.radius)
		{}
	};

	struct TTriangleMesh : TPointSet {
		vector<vec3i> indices; // triangle vertex indices : # of triangles
		TTriangleMesh() {}
		TTriangleMesh(const TTriangleMesh &tm) :
			TPointSet(tm),
			indices(tm.indices)
		{}
	};

	struct TTetrahedralMesh : TPointSet {
		vector<vec4i> indices; // tetrahedron vertex indices : # of tetrahedra
		TTetrahedralMesh() {}
		TTetrahedralMesh(const TTetrahedralMesh &tm) :
			TPointSet(tm),
			indices(tm.indices)
		{}
	};
}