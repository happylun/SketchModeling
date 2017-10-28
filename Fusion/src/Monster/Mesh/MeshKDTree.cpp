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


#include "MeshKDTree.h"

using namespace Monster;

bool MeshKDTree::buildKdTree(vector<vec3> &samples, SKDTree &tree, SKDTreeData &treeData) {

	treeData.resize(samples.size());
	for (size_t i = 0; i < samples.size(); i++) {
		vec3 &v = samples[i];
		treeData[i] = SKDT::NamedPoint(v[0], v[1], v[2], i);
	}
	tree.init(treeData.begin(), treeData.end());

	return true;
}

bool MeshKDTree::buildKdTree(TTriangleMesh &mesh, TKDTree &tree, TKDTreeData &treeData) {

	treeData.resize(mesh.indices.size());

#pragma omp parallel for
	for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
		vec3i idx = mesh.indices[faceID];
		G3D::Vector3 v0(mesh.positions[idx[0]].data());
		G3D::Vector3 v1(mesh.positions[idx[1]].data());
		G3D::Vector3 v2(mesh.positions[idx[2]].data());
		treeData[faceID].set(TKDT::NamedTriangle(v0, v1, v2, faceID));
	}

	tree.init(treeData.begin(), treeData.end());

	return true;
}

bool MeshKDTree::buildKdTree(TTriangleMesh &mesh, SKDTree &tree, SKDTreeData &treeData) {

	vector<vec3> faceCenters(mesh.indices.size());
	for (int faceID = 0; faceID < (int)mesh.indices.size(); faceID++) {
		vec3i idx = mesh.indices[faceID];
		faceCenters[faceID] = (mesh.positions[idx[0]] + mesh.positions[idx[1]] + mesh.positions[idx[2]]) / 3;
	}

	return buildKdTree(faceCenters, tree, treeData);
}

bool MeshKDTree::distanceToPoints(vector<vec3> &points, SKDTree &tree, vec3 &point, double &distance) {

	// better to parallelize outside this function

	SKDT::NamedPoint queryPoint(point[0], point[1], point[2]);
	Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(1);
	tree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);

	int pointID = (int)tree.getElements()[queryResult[0].getIndex()].id;
	distance = (points[pointID] - point).length();

	return true;
}

bool MeshKDTree::distanceToMesh(TTriangleMesh &mesh, SKDTree &faceTree, vec3 &point, double &distance) {

	// better to parallelize outside this function

	int numQueryFace = 10;

	SKDT::NamedPoint queryPoint(point[0], point[1], point[2]);
	Thea::BoundedSortedArray<SKDTree::Neighbor> queryResult(numQueryFace);
	faceTree.kClosestElements<Thea::MetricL2>(queryPoint, queryResult);

	int minFace;
	int minType;
	vec3 minPoint;

	double minDistance = DBL_MAX;
	vec3d p = point;
	for (int qID = 0; qID < queryResult.size(); qID++) {
		int faceID = (int)faceTree.getElements()[queryResult[qID].getIndex()].id;
		vec3i idx = mesh.indices[faceID];
		vec3d v[3];
		for (int k = 0; k < 3; k++) v[k] = mesh.positions[idx[k]];
		vec3d n = cml::normalize(cml::cross(v[1] - v[0], v[2] - v[0]));
		double distFace = cml::dot(p - v[0], n);
		vec3d projP = p - n*distFace;
		bool insideFace = true;
		for (int k = 0; k < 3; k++) {
			if (cml::dot(n, cml::cross(projP - v[k], projP - v[(k + 1) % 3])) < 0) {
				insideFace = false;
				break;
			}
		}
		if (insideFace) {
			distFace = fabs(distFace);
			if (distFace < minDistance) {
				minDistance = distFace;
				minFace = faceID;
				minType = 0;
				minPoint = projP;
			}

		} else {
			bool insideEdge = false;
			for (int k = 0; k < 3; k++) {
				vec3d &v1 = v[k];
				vec3d &v2 = v[(k + 1) % 3];
				vec3d edge = v2 - v1;
				double sign1 = cml::dot(p - v1, edge);
				double sign2 = cml::dot(p - v2, edge);
				if (sign1 > 0 && sign2 < 0) {
					insideEdge = true;
					vec3 edgeP = v1 + edge * sign1 / edge.length_squared();
					double distEdge = (p - edgeP).length();
					if (distEdge < minDistance) {
						minDistance = distEdge;
						minFace = faceID;
						minType = 1;
						minPoint = edgeP;
					}
				}
			}
			if (!insideEdge) {
				for (int k = 0; k < 3; k++) {
					double distVertex = (p - v[k]).length();
					if (distVertex < minDistance) {
						minDistance = distVertex;
						minFace = faceID;
						minType = 2;
						minPoint = v[k];
					}
				}
			}
		}
	}

	distance = minDistance;

	return true;
}

bool MeshKDTree::checkInsideMesh(TKDTree &tree, vec3 &point, bool &insideFlag, double eps) {

	int numQueryRay = 5;

	int insideCount = 0;
	for (int qID = 0; qID < numQueryRay; qID++) {

		vec3d rayOrigin = point;

		double r1 = cml::random_unit();
		double r2 = cml::random_unit();
		vec3d rayDir(
			2.0 * cos(cml::constantsd::two_pi()*r1) * cml::sqrt_safe(r2*(1 - r2)),
			2.0 * sin(cml::constantsd::two_pi()*r1) * cml::sqrt_safe(r2*(1 - r2)),
			1.0 - 2.0*r2); // random direction on unit sphere

		int hitCount = 0;
		while (true) {
			Thea::Ray3 ray(G3D::Vector3(rayOrigin.data()), G3D::Vector3(rayDir.data()));
			double dist = tree.rayIntersectionTime(ray);
			if (dist < 0) break;
			hitCount++;
			rayOrigin += rayDir * (dist + eps);
		}

		insideCount += hitCount % 2;
	}

	insideFlag = (insideCount * 2 > numQueryRay);

	return true;
}
