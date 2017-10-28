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


#include "MeshTets.h"

#include <set>

#include "Library/libiglHelperBegin.h"
#include "igl/copyleft/tetgen/tetrahedralize.h"
#include "Library/libiglHelperEnd.h"

#include "Mesh/MeshCompute.h"
#include "Mesh/MeshClean.h"

using namespace Monster;

bool MeshTets::tetrahedralize(Eigen::MatrixXd &points, TTetrahedralMesh &tets) {

	if (points.rows() < 4) {
		cout << "Error: not enough points for tetrahedralization" << endl;
		return false;
	}
	if (points.cols() != 3) {
		cout << "Error: cannot apply tetrahedralization on non-3D points" << endl;
		return false;
	}

	Eigen::MatrixXi matNull(0, 0);
	Eigen::MatrixXd matTV;
	Eigen::MatrixXi matTT;
	Eigen::MatrixXi matTF;
	igl::copyleft::tetgen::tetrahedralize(points, matNull, "pcQ", matTV, matTT, matTF);
	if (!mat2tet(tets, matTV, matTT)) return false;

	return true;
}

bool MeshTets::mesh2mat(TTriangleMesh &mesh, Eigen::MatrixXd &matV, Eigen::MatrixXi &matF) {

	matV.resize(mesh.amount, 3);
	for (int vertID = 0; vertID < mesh.amount; vertID++) {
		vec3 &v = mesh.positions[vertID];
		matV.row(vertID) = Eigen::RowVector3d((double)v[0], (double)v[1], (double)v[2]);
	}

	matF.resize(mesh.indices.size(), 3);
	for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
		vec3i &f = mesh.indices[faceID];
		matF.row(faceID) = Eigen::RowVector3i(f[0], f[1], f[2]);
	}

	return true;
}

bool MeshTets::mat2mesh(TTriangleMesh &mesh, Eigen::MatrixXd &matV, Eigen::MatrixXi &matF) {

	mesh.positions.resize((int)matV.rows());
	mesh.amount = (int)mesh.positions.size();

	for (int vertID = 0; vertID < mesh.amount; vertID++) {
		mesh.positions[vertID] = vec3d(matV(vertID, 0), matV(vertID, 1), matV(vertID, 2));
	}

	mesh.indices.resize((int)matF.rows());
	for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
		mesh.indices[faceID] = vec3i(matF(faceID, 0), matF(faceID, 1), matF(faceID, 2));
	}

	if (!MeshCompute::recomputeNormals(mesh)) return false;

	return true;
}

bool MeshTets::tet2mat(TTetrahedralMesh &tet, Eigen::MatrixXd &matV, Eigen::MatrixXi &matT) {

	matV.resize(tet.amount, 3);
	for (int vertID = 0; vertID < tet.amount; vertID++) {
		vec3 &v = tet.positions[vertID];
		matV.row(vertID) = Eigen::RowVector3d((double)v[0], (double)v[1], (double)v[2]);
	}

	matT.resize(tet.indices.size(), 4);
	for (int tetID = 0; tetID < (int)tet.indices.size(); tetID++) {
		vec4i &t = tet.indices[tetID];
		matT.row(tetID) = Eigen::RowVector4i(t[0], t[1], t[2], t[3]);
	}

	return true;
}

bool MeshTets::mat2tet(TTetrahedralMesh &tet, Eigen::MatrixXd &matV, Eigen::MatrixXi &matT) {

	tet.positions.resize((int)matV.rows());
	tet.normals.clear();
	tet.amount = (int)tet.positions.size();

	for (int vertID = 0; vertID < tet.amount; vertID++) {
		tet.positions[vertID] = vec3d(matV(vertID, 0), matV(vertID, 1), matV(vertID, 2));
	}

	tet.indices.resize((int)matT.rows());
	for (int faceID = 0; faceID < (int)tet.indices.size(); faceID++) {
		tet.indices[faceID] = vec4i(matT(faceID, 0), matT(faceID, 1), matT(faceID, 2), matT(faceID, 3));
	}

	return true;
}

bool MeshTets::tet2mesh(TTetrahedralMesh &tet, TTriangleMesh &mesh) {

	int numVerts = tet.amount;
	int numTets = (int)tet.indices.size();

	mesh.positions = tet.positions;
	mesh.normals = tet.normals;
	mesh.amount = numVerts;

	set<vec3i> triangleSet;
	for (int tetID = 0; tetID < numTets; tetID++) {
		auto t = tet.indices[tetID];
		for (int k = 0; k < 4; k++) {
			vec3i key(t[k], t[(k + 1) % 4], t[(k + 2) % 4]);
			if (key[0] > key[1]) swap(key[0], key[1]);
			if (key[0] > key[2]) swap(key[0], key[2]);
			if (key[1] > key[2]) swap(key[1], key[2]);
			triangleSet.insert(key);
		}
	}
	mesh.indices.clear();
	for (vec3i key : triangleSet) {
		mesh.indices.push_back(key);
	}

	return true;
}

bool MeshTets::tet2meshSimple(TTetrahedralMesh &tet, TTriangleMesh &mesh) {

	int numVerts = tet.amount;
	int numTets = (int)tet.indices.size();

	mesh.positions = tet.positions;
	mesh.normals = tet.normals;
	mesh.amount = numVerts;

	mesh.indices.resize(numTets * 4);
	for (int tetID = 0; tetID < numTets; tetID++) {
		auto t = tet.indices[tetID];
		mesh.indices[tetID * 4 + 0] = vec3i(t[0], t[1], t[3]);
		mesh.indices[tetID * 4 + 1] = vec3i(t[0], t[2], t[1]);
		mesh.indices[tetID * 4 + 2] = vec3i(t[3], t[2], t[0]);
		mesh.indices[tetID * 4 + 3] = vec3i(t[1], t[2], t[3]);
	}

	return true;
}

bool MeshTets::tet2alpha(TTetrahedralMesh &tet, TTriangleMesh &alpha, double radius) {

	int numTets = (int)tet.indices.size();

	// compute tetrahedron's circumradius
	// ref: http://mathworld.wolfram.com/Tetrahedron.html

	vector<double> tetRadius(numTets);
#pragma omp parallel for
	for (int tetID = 0; tetID < numTets; tetID++) {
		vec4i idx = tet.indices[tetID];
		vec3 v[4];
		for (int k = 0; k < 4; k++) v[k] = tet.positions[idx[k]];
		vec3d a = v[1] - v[0]; double al = a.length();
		vec3d b = v[2] - v[0]; double bl = b.length();
		vec3d c = v[3] - v[0]; double cl = c.length();
		vec3d ap = v[3] - v[2]; double apl = ap.length();
		vec3d bp = v[3] - v[1]; double bpl = bp.length();
		vec3d cp = v[2] - v[1]; double cpl = cp.length();
		double sa = al*apl;
		double sb = bl*bpl;
		double sc = cl*cpl;
		if (sa < sb) swap(sa, sb);
		if (sa < sc) swap(sa, sc);
		if (sb < sc) swap(sb, sc);
		double delta = sqrt((sa + (sb + sc)) * (sc - (sa - sb)) * (sc + (sa - sb)) * (sa + (sb - sc))) / 4;
		double volume = fabs(cml::dot(a, cml::cross(b, c))) / 6;
		tetRadius[tetID] = delta / (volume * 6);
	}

	// prune tetrahedra

	vector<vec4i> newIndices(0);
	for (int tetID = 0; tetID < numTets; tetID++) {
		if (tetRadius[tetID] < radius) {
			newIndices.push_back(tet.indices[tetID]);
		}
	}
	tet.indices.swap(newIndices);
	if (!MeshClean::cleanUp(tet)) return false;
	if (!tet2mesh(tet, alpha)) return false;

	return true;
}