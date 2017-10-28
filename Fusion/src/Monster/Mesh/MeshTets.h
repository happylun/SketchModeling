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

	class MeshTets {

	private:

		// make it non-instantiable
		MeshTets() {}
		~MeshTets() {}

	public:

		static bool tetrahedralize(Eigen::MatrixXd &points, TTetrahedralMesh &tets);

		static bool mesh2mat(TTriangleMesh &mesh, Eigen::MatrixXd &matV, Eigen::MatrixXi &matF);
		static bool mat2mesh(TTriangleMesh &mesh, Eigen::MatrixXd &matV, Eigen::MatrixXi &matF);
		static bool tet2mat(TTetrahedralMesh &tet, Eigen::MatrixXd &matV, Eigen::MatrixXi &matT);
		static bool mat2tet(TTetrahedralMesh &tet, Eigen::MatrixXd &matV, Eigen::MatrixXi &matT);
		static bool tet2mesh(TTetrahedralMesh &tet, TTriangleMesh &mesh);
		static bool tet2meshSimple(TTetrahedralMesh &tet, TTriangleMesh &mesh);
		static bool tet2alpha(TTetrahedralMesh &tet, TTriangleMesh &alpha, double radius);
	};
}