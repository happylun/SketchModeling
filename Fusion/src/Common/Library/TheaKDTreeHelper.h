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

/*
	Added (for VS2015 compatibility):
		TheaKDTree/Vector.hpp#L23:
			#include <algorithm>
	Modified (removed boost library dependency):
		TheaKDTree/Util.hpp
		TheaKDTree/KDTree3.hpp			
*/

#include "TheaKDTree/KDTree3.hpp"

namespace SKDT {
	struct NamedPoint {
		G3D::Vector3 position;
		size_t  id;

		NamedPoint() {}
		NamedPoint(float x, float y, float z) : position(x,y,z) {}
		NamedPoint(float x, float y, float z, const size_t _id) : position(x,y,z), id(_id) {}
	};
}

namespace TKDT {
	struct NamedTriangle {
		G3D::Vector3 v[3];
		size_t id;
		
		NamedTriangle() {}
		NamedTriangle(G3D::Vector3 const & v0_, G3D::Vector3 const & v1_, G3D::Vector3 const & v2_, size_t _id) {
			v[0] = v0_;
			v[1] = v1_;
			v[2] = v2_;
			id = _id;
		}
		NamedTriangle(NamedTriangle const & src) {
			v[0] = src.v[0];
			v[1] = src.v[1];
			v[2] = src.v[2];
			id = src.id;
		}
		
		// Get the i'th vertex. This is the operative function that a vertex triple needs to have.
		G3D::Vector3 const & getVertex(int i) const { return v[i]; }
	};
}

namespace Thea {
	template <>
	struct PointTraits3<SKDT::NamedPoint> {
		static G3D::Vector3 const & getPosition(SKDT::NamedPoint const & np) { return np.position; }
	};

	template <>
	struct IsPoint3<SKDT::NamedPoint> {
		static bool const value = true;
	};
}

typedef Thea::KDTree3<SKDT::NamedPoint> SKDTree;
typedef std::vector<SKDT::NamedPoint> SKDTreeData;

typedef Thea::Triangle3<TKDT::NamedTriangle> TKDTreeElement;
typedef Thea::KDTree3<TKDTreeElement> TKDTree;
typedef std::vector<TKDTreeElement> TKDTreeData;