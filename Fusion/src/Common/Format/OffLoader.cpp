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


#include "OffLoader.h"

#include <fstream>
#include <sstream>

bool OffLoader::loadMesh(string fileName,
	vector<vec3i> *indices, vector<vec3> *vertices,
	vector<vec3i> *vertexColors, vector<vec3i> *faceColors)
{

	if(!indices) return false;
	if(!vertices) return false;

	ifstream file(fileName);
	if (!file.is_open()) {
		cout << "Cannot open file " << fileName << endl;
		return false;
	}

	string line;
	file >> line; // header

	int numVertex, numFace, numEdge;
	file >> numVertex >> numFace >> numEdge;
	getline(file, line); // move to next line

	vertices->resize(numVertex);
	bool outputVertexColor = (vertexColors != 0);
	if (outputVertexColor) vertexColors->clear();
	for (int k = 0; k < numVertex; k++) {
		string line;
		getline(file, line);
		stringstream ss(line);

		vec3 &p = (*vertices)[k];
		ss >> p[0] >> p[1] >> p[2];
		
		if (outputVertexColor) {
			vec3i color(0, 0, 0);
			float r, g, b, a;
			ss >> r >> g >> b >> a;
			if (!ss.fail()) {
				color = vec3i((int)(r * 255), (int)(g * 255), (int)(b * 255));
			}
			vertexColors->push_back(color);
		}
	}

	indices->clear();
	bool outputFaceColor = (faceColors != 0);
	if (outputFaceColor) faceColors->clear();
	for (int k = 0; k < numFace; k++) {
		string line;
		getline(file, line);
		stringstream ss(line);

		int n;
		ss >> n;
		vector<int> v(n);
		for (int i = 0; i < n; i++) ss >> v[i];

		vec3i color(0, 0, 0);
		if (outputFaceColor) {
			float r, g, b, a;
			ss >> r >> g >> b >> a;
			if (!ss.fail()) {
				color = vec3i((int)(r * 255), (int)(g * 255), (int)(b * 255));
			}
		}

		for (int i = 1; i < n - 1; i++) {
			vec3i idx(v[0], v[i], v[i + 1]);
			indices->push_back(idx);
			if (outputFaceColor) {
				faceColors->push_back(color);
			}
		}
	}

	file.close();

	return true;
}
