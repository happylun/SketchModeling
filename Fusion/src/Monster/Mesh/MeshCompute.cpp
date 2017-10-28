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


#include "MeshCompute.h"

#include "MeshKDTree.h"

#include "Library/EigenHelper.h"

using namespace Monster;

bool MeshCompute::recomputeNormals(TTriangleMesh &mesh) {

	int numVertices = mesh.amount;

	vector<vec3d> normals(numVertices, vec3d(0.0, 0.0, 0.0));
	vector<vec3d> binormals(numVertices, vec3d(0.0, 0.0, 0.0));
	vector<vec3d> rawnormals(numVertices, vec3d(0.0, 0.0, 0.0));
	vector<double> weights(numVertices, 0.0);

	for (vec3i faceIdx : mesh.indices) {
		vec3d v[3];
		for (int j = 0; j < 3; j++) v[j] = mesh.positions[faceIdx[j]];
		vec3d bn[3];
		for (int j = 0; j < 3; j++) bn[j] = cml::normalize(v[j] - (v[(j + 1) % 3] + v[(j + 2) % 3]) / 2);

		vec3d vv1 = v[1] - v[0];
		vec3d vv2 = v[2] - v[0];
		double l1 = vv1.length();
		double l2 = vv2.length();
		vec3d n = cml::cross(vv1, vv2);
		double area = n.length() / 2;
		if (n.length_squared()) n.normalize();

		//double w = cml::unsigned_angle(vv1, vv2); // angle weight
		//double w = area; // area weight
		double w = area / cml::sqr(l1*l2); // spherical mesh weight

		for (int j = 0; j < 3; j++) {
			rawnormals[faceIdx[j]] += n;
			normals[faceIdx[j]] += n * w;
			binormals[faceIdx[j]] += bn[j] * w;
			weights[faceIdx[j]] += w;
		}
	}

	bool warnFlag = false;
	mesh.normals.resize(numVertices);
	for (int j = 0; j < numVertices; j++) {
		if (normals[j].length_squared() > 0) {
			mesh.normals[j] = (vec3)cml::normalize(normals[j]);
		} else if (binormals[j].length_squared() > 0) {
			mesh.normals[j] = (vec3)cml::normalize(binormals[j]);
		} else if (rawnormals[j].length_squared() > 0) {
			mesh.normals[j] = (vec3)cml::normalize(rawnormals[j]);
		} else {
			if (!warnFlag) {
				//cout << "Warning: detected zero area faces or unreferenced vertices" << endl;
				warnFlag = true;
			}
			mesh.normals[j] = vec3(0.0f, 1.0f, 0.0f);
		}
	}

	return true;
}

bool MeshCompute::recomputeNormals(vector<vec3> &points, vector<vec3> &normals, int numNeighbors, double neighborDist) {

	SKDTree tree;
	SKDTreeData treeData;
	if (!MeshKDTree::buildKdTree(points, tree, treeData)) return false;

	vec3 bbMin(FLT_MAX, FLT_MAX, FLT_MAX);
	vec3 bbMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (vec3 v : points) {
		bbMin.minimize(v);
		bbMax.maximize(v);
	}
	double nnDist = (bbMax - bbMin).length() * neighborDist;

	normals.resize(points.size());
#pragma omp parallel for
	for (int pointID = 0; pointID < (int)points.size(); pointID++) {
		vec3 point = points[pointID];
		SKDT::NamedPoint queryPoint(point[0], point[1], point[2]);
		Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(numNeighbors);
		tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult, nnDist);
		if (queryResult.size() < 6) {
			queryResult = Thea::BoundedSortedArray<SKDTree::Neighbor>(numNeighbors);
			tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);
		}
		
		Eigen::MatrixX3d patchMat(queryResult.size(), 3);
		for (int queryID = 0; queryID < queryResult.size(); queryID++) {
			int neighborID = (int)tree.getElements()[queryResult[queryID].getIndex()].id;
			Eigen::Vector3d neighborPoint(vec3d(points[neighborID]).data());
			patchMat.row(queryID) = neighborPoint;
		}
		patchMat.rowwise() -= patchMat.colwise().mean();
		Eigen::JacobiSVD< Eigen::MatrixXd > svd(patchMat, Eigen::ComputeThinV);
		Eigen::Vector3d normal = svd.matrixV().col(2).normalized();
		if (normal[2] < 0) normal = -normal;
		normals[pointID] = cml::normalize(vec3d(normal[0], normal[1], normal[2]));
	}

	return true;
}

bool MeshCompute::computeDihedralAngle(vec3 center1, vec3 normal1, vec3 center2, vec3 normal2, double &angle) {

	double dihedralAngle = cml::constantsd::pi() - cml::acos_safe((double)cml::dot(normal1, normal2));

	vec3 centerDir = center1 - center2;
	float cosAngle1 = cml::dot(centerDir, normal2); // unnormalized
	float cosAngle2 = cml::dot(-centerDir, normal1); // unnormalized
	bool flag1 = (cosAngle1 >= 0);
	bool flag2 = (cosAngle2 >= 0);

	if (flag1 && flag2) {
		// concave
	} else if (!flag1 && !flag2) {
		// convex
		dihedralAngle = cml::constantsd::two_pi() - dihedralAngle;
	} else {
		if (cml::dot(normal1, normal2) > 0.9f) {
			// prevent numerical error on flat plane
		} else {
			// face orientation incompatible
			dihedralAngle = -cml::constantsd::pi();
		}
	}

	angle = dihedralAngle;

	return true;
}

bool MeshCompute::computeAABB(TPointSet &mesh, vec3 &bbMin, vec3 &bbMax) {

	if (mesh.amount == 0) {
		// empty shape
		bbMin = vec3(0.0f, 0.0f, 0.0f);
		bbMax = vec3(0.0f, 0.0f, 0.0f);
		return true;
	}

	bbMin = vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	bbMax = vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (vec3 v : mesh.positions) {
		bbMin.minimize(v);
		bbMax.maximize(v);
	}

	return true;
}

bool MeshCompute::computeBoundingSphere(TPointSet &mesh, vec3 &center, float &radius) {

	vec3 bbMin, bbMax;
	if (!computeAABB(mesh, bbMin, bbMax)) return false;

	center = (bbMax + bbMin) / 2;
	radius = 0.0f;
	for (vec3 point : mesh.positions) {
		float dist = (point - center).length();
		radius = max(radius, dist);
	}

	return true;
}

bool MeshCompute::computeFaceArea(TTriangleMesh &mesh, double &area) {

	area = 0;
	for (vec3i &faceIdx : mesh.indices) {
		vec3 facePos[3];
		for (int k = 0; k < 3; k++) facePos[k] = mesh.positions[faceIdx[k]];
		area += cml::cross(facePos[1] - facePos[0], facePos[2] - facePos[0]).length() * 0.5;
	}

	return true;
}

bool MeshCompute::computeMassCenter(TTriangleMesh &mesh, vec3 &center) {

	// compute face center and area

	int numFaces = (int)mesh.indices.size();
	vector<vec3> faceCenters(numFaces);
	vector<float> faceAreas(numFaces);
#pragma omp parallel for
	for (int faceID = 0; faceID < numFaces; faceID++) {
		vec3i idx = mesh.indices[faceID];
		vec3 v[3];
		for (int k = 0; k < 3; k++) v[k] = mesh.positions[idx[k]];

		faceCenters[faceID] = (v[0] + v[1] + v[2]) / 3;
		faceAreas[faceID] = cml::cross(v[1] - v[0], v[2] - v[0]).length() / 2;
	}

	// compute mass center

	vec3d massCenter(0.0, 0.0, 0.0);
	double mass = 0;
	for (int faceID = 0; faceID < numFaces; faceID++) {
		massCenter += faceCenters[faceID] * faceAreas[faceID];
		mass += faceAreas[faceID];
	}
	massCenter /= mass;

	center = massCenter;

	return true;
}