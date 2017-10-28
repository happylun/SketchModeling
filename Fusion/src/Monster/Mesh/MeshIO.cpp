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


#include "MeshIO.h"

#include <iostream>

#include "Format/PlyLoader.h"
#include "Format/PlyExporter.h"

using namespace Monster;

bool MeshIO::saveMesh(string fileName, TTriangleMesh &mesh, bool ascii) {

	string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "ply") {
		PlyExporter pe;
		if (!pe.addMesh(&mesh.indices, &mesh.positions, mesh.normals.empty() ? 0 : &mesh.normals)) return false;
		if (!pe.output(fileName, ascii)) return false;
	} else {
		cout << "unsupported mesh format : " << ext << endl;
		return false;
	}

	return true;
}

bool MeshIO::loadMesh(string fileName, TTriangleMesh &mesh) {

	string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "ply") {
		if (!PlyLoader::loadMesh(fileName, &mesh.positions, &mesh.normals, &mesh.indices)) return false;
		mesh.amount = (int)mesh.positions.size();
	} else {
		cout << "Unable to load mesh with extension \"" << ext << "\"" << endl;
		return false;
	}

	return true;
}

bool MeshIO::savePointSet(string fileName, TPointSet &points, bool ascii) {

	string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "ply") {
		PlyExporter pe;
		if (!pe.addPoint(&points.positions, points.normals.empty() ? 0 : &points.normals)) return false;
		if (!pe.output(fileName, ascii)) return false;
	} else {
		cout << "unsupported point set format : " << ext << endl;
		return false;
	}

	return true;
}

bool MeshIO::loadPointSet(string fileName, TPointSet &points) {

	string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "ply") {
		if (!PlyLoader::loadMesh(fileName, &points.positions, &points.normals)) return false;
		points.amount = (int)points.positions.size();
	} else {
		cout << "Unable to load point set with extension \"" << ext << "\"" << endl;
		return false;
	}

	return true;
}


vec3i MeshIO::colorMapping(int index) {

	static vector<vec3i> colorMap(0);
	if (colorMap.size() == 0) {
		// random permutation of HSV color map
		// 0: red
		// 1: cyan
		// 2: green
		// 3: purple
		// 4: yellow
		// 5: blue
		// 6: dark green
		// 7: pink
		// 8: light cyan
		// 9: dark blue
		colorMap.push_back(vec3i(255, 0, 0));      // 0
		colorMap.push_back(vec3i(0, 128, 128));    // 1
		colorMap.push_back(vec3i(0, 255, 0));      // 2
		colorMap.push_back(vec3i(128, 0, 128));    // 3
		colorMap.push_back(vec3i(255, 255, 0));    // 4
		colorMap.push_back(vec3i(0, 0, 255));      // 5
		colorMap.push_back(vec3i(0, 128, 0));      // 6
		colorMap.push_back(vec3i(255, 0, 255));    // 7
		colorMap.push_back(vec3i(0, 255, 255));    // 8
		colorMap.push_back(vec3i(0, 0, 128));      // 9
		colorMap.push_back(vec3i(128, 0, 0));      // 10
		colorMap.push_back(vec3i(176, 255, 0));    // 11
		colorMap.push_back(vec3i(128, 128, 0));    // 12
		colorMap.push_back(vec3i(111, 0, 255));    // 13
		colorMap.push_back(vec3i(0, 120, 255));    // 14
		colorMap.push_back(vec3i(0, 255, 184));    // 15
		colorMap.push_back(vec3i(255, 0, 141));    // 16
		colorMap.push_back(vec3i(255, 103, 0));    // 17
		colorMap.push_back(vec3i(0, 107, 255));    // 18
		colorMap.push_back(vec3i(255, 231, 0));    // 19
		colorMap.push_back(vec3i(60, 0, 255));     // 20
		colorMap.push_back(vec3i(21, 255, 0));
		colorMap.push_back(vec3i(21, 0, 255));
		colorMap.push_back(vec3i(124, 255, 0));
		colorMap.push_back(vec3i(73, 255, 0));
		colorMap.push_back(vec3i(0, 255, 107));
		colorMap.push_back(vec3i(255, 0, 77));
		colorMap.push_back(vec3i(137, 255, 0));
		colorMap.push_back(vec3i(0, 255, 43));
		colorMap.push_back(vec3i(255, 0, 26));
		colorMap.push_back(vec3i(0, 255, 159));
		colorMap.push_back(vec3i(0, 236, 255));
		colorMap.push_back(vec3i(0, 210, 255));
		colorMap.push_back(vec3i(34, 0, 255));
		colorMap.push_back(vec3i(201, 255, 0));
		colorMap.push_back(vec3i(9, 255, 0));
		colorMap.push_back(vec3i(86, 0, 255));
		colorMap.push_back(vec3i(255, 13, 0));
		colorMap.push_back(vec3i(201, 0, 255));
		colorMap.push_back(vec3i(240, 0, 255));
		colorMap.push_back(vec3i(255, 0, 51));
		colorMap.push_back(vec3i(0, 255, 197));
		colorMap.push_back(vec3i(0, 255, 56));
		colorMap.push_back(vec3i(189, 255, 0));
		colorMap.push_back(vec3i(176, 0, 255));
		colorMap.push_back(vec3i(255, 90, 0));
		colorMap.push_back(vec3i(0, 133, 255));
		colorMap.push_back(vec3i(255, 0, 90));
		colorMap.push_back(vec3i(34, 255, 0));
		colorMap.push_back(vec3i(255, 0, 154));
		colorMap.push_back(vec3i(214, 255, 0));
		colorMap.push_back(vec3i(189, 0, 255));
		colorMap.push_back(vec3i(0, 30, 255));
		colorMap.push_back(vec3i(99, 0, 255));
		colorMap.push_back(vec3i(255, 64, 0));
		colorMap.push_back(vec3i(255, 0, 206));
		colorMap.push_back(vec3i(255, 0, 39));
		colorMap.push_back(vec3i(240, 255, 0));
		colorMap.push_back(vec3i(255, 0, 0));
		colorMap.push_back(vec3i(0, 4, 255));
		colorMap.push_back(vec3i(60, 255, 0));
		colorMap.push_back(vec3i(0, 69, 255));
		colorMap.push_back(vec3i(255, 116, 0));
		colorMap.push_back(vec3i(255, 0, 13));
		colorMap.push_back(vec3i(86, 255, 0));
		colorMap.push_back(vec3i(253, 0, 255));
		colorMap.push_back(vec3i(0, 223, 255));
		colorMap.push_back(vec3i(255, 180, 0));
		colorMap.push_back(vec3i(255, 77, 0));
		colorMap.push_back(vec3i(255, 0, 219));
		colorMap.push_back(vec3i(99, 255, 0));
		colorMap.push_back(vec3i(0, 255, 171));
		colorMap.push_back(vec3i(0, 255, 236));
		colorMap.push_back(vec3i(0, 255, 223));
		colorMap.push_back(vec3i(255, 39, 0));
		colorMap.push_back(vec3i(0, 255, 133));
		colorMap.push_back(vec3i(227, 0, 255));
		colorMap.push_back(vec3i(0, 255, 69));
		colorMap.push_back(vec3i(0, 159, 255));
		colorMap.push_back(vec3i(255, 219, 0));
		colorMap.push_back(vec3i(255, 0, 231));
		colorMap.push_back(vec3i(73, 0, 255));
		colorMap.push_back(vec3i(0, 184, 255));
		colorMap.push_back(vec3i(0, 255, 81));
		colorMap.push_back(vec3i(0, 146, 255));
		colorMap.push_back(vec3i(255, 0, 129));
		colorMap.push_back(vec3i(255, 0, 193));
		colorMap.push_back(vec3i(0, 94, 255));
		colorMap.push_back(vec3i(0, 56, 255));
		colorMap.push_back(vec3i(0, 171, 255));
		colorMap.push_back(vec3i(111, 255, 0));
		colorMap.push_back(vec3i(0, 255, 210));
		colorMap.push_back(vec3i(255, 167, 0));
		colorMap.push_back(vec3i(0, 255, 30));
		colorMap.push_back(vec3i(0, 255, 249));
		colorMap.push_back(vec3i(137, 0, 255));
		colorMap.push_back(vec3i(255, 206, 0));
		colorMap.push_back(vec3i(255, 193, 0));
		colorMap.push_back(vec3i(163, 255, 0));
		colorMap.push_back(vec3i(0, 255, 146));
		colorMap.push_back(vec3i(163, 0, 255));
		colorMap.push_back(vec3i(255, 26, 0));
		colorMap.push_back(vec3i(0, 255, 17));
		colorMap.push_back(vec3i(0, 197, 255));
		colorMap.push_back(vec3i(150, 255, 0));
		colorMap.push_back(vec3i(0, 43, 255));
		colorMap.push_back(vec3i(255, 129, 0));
		colorMap.push_back(vec3i(255, 0, 64));
		colorMap.push_back(vec3i(255, 141, 0));
		colorMap.push_back(vec3i(47, 0, 255));
		colorMap.push_back(vec3i(255, 154, 0));
	}

	if (index < 0) return vec3i(0, 0, 0);
	if (index < (int)colorMap.size()) return colorMap[index];

	return vec3i(cml::random_integer(0, 255), cml::random_integer(0, 255), cml::random_integer(0, 255));
}

vec3i MeshIO::colorMappingDefinite(int index) {

	return vec3i(index%256, (index/256)%256, (index/256)/256);
}