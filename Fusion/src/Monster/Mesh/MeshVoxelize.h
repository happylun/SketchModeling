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

#include "Library/EigenHelper.h"

namespace Monster {

	class MeshVoxelize {

	private:

		// make it non-instantiable
		MeshVoxelize() {}
		~MeshVoxelize() {}

	public:

		static bool voxelize(
			TTriangleMesh &mesh,
			double range, int resolution,
			vector<vector<vector<bool>>> &voxels);

		static bool fillVoxels(vector<vector<vector<bool>>> &voxels);

		static bool voxels2mesh(
			vector<vector<vector<bool>>> &voxels,
			double range, TTriangleMesh &mesh);
	};
}