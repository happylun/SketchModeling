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

	class MeshClean {

	private:

		// make it non-instantiable
		MeshClean() {}
		~MeshClean() {}

	public:

		static bool removeDuplicateVertices(
			vector<vec3> &inVertices,
			vector<vec3> &outVertices,
			vector<int> &outVertexIndices, // vertex ID in outVertices : # of vertices in inVertices
			double eps = -1);

		static bool removeDuplicateVertices(
			TTriangleMesh &inMesh,
			TTriangleMesh &outMesh,
			vector<int> &outVertexIndices, // vertex ID in outMesh : # of vertices in inMesh
			double eps = -1);

		static bool removeDuplicateFaces(
			TTriangleMesh &inMesh,
			TTriangleMesh &outMesh,
			vector<int> &outFaceIndices, // face ID in outMesh : # of faces in inMesh
			double eps = -1);

		static bool removeDegeneratedFaces(
			TTriangleMesh &inMesh,
			TTriangleMesh &outMesh,
			vector<int> &outFaceIndices, // face ID in outMesh : # of faces in inMesh
			double eps = -1);

		static bool weldMeshFaces(
			TTriangleMesh &inMesh,
			TTriangleMesh &outMesh,
			vector<int> &outFaceIndices, // face ID in outMesh : # of faces in inMesh
			double eps = -1);

		static bool unweldMeshVertices(
			TTriangleMesh &inMesh,
			TTriangleMesh &outMesh);

		static bool cleanUp(TTriangleMesh &mesh);
		static bool cleanUp(TTetrahedralMesh &mesh);
	};
}