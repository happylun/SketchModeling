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

#include <string>
#include <vector>

#include "Library/CMLHelper.h"

using namespace std;

class PlyExporter {

public:

	PlyExporter()  { clearUp(); }
	~PlyExporter() { clearUp(); }

public:

	void clearUp();

	bool addMesh(string fileName, vec3 offset = vec3(0.f, 0.f, 0.f), vec3i color = vec3i(0, 0, 0));
	bool addMesh(string fileName, matrix transform = cml::identity_4x4(), vec3i color = vec3i(0, 0, 0));
	bool addMesh(vector<vec3i> *indices, vector<vec3> *vertices, vector<vec3> *normals, vec3 offset, vec3i color = vec3i(0, 0, 0));
	bool addMesh(vector<vec3i> *indices, vector<vec3> *vertices, vector<vec3> *normals = 0, matrix transform = cml::identity_4x4(), vec3i color = vec3i(0, 0, 0));
	bool addMesh(vector<vec3i> *indices, vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *colors, vec3 offset = vec3(0.0f, 0.0f, 0.0f));
	bool addMesh(vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *indices, vec3i faceColors = vec3i(0, 0, 0), vec3 offset = vec3(0.0f, 0.0f, 0.0f));
	bool addMesh(vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *indices, vector<vec3i> *faceColors, vec3 offset = vec3(0.0f, 0.0f, 0.0f));

	bool addPoint(string fileName, vec3 offset = vec3(0.f,0.f,0.f), vec3i color = vec3i(0,0,0));
	bool addPoint(string fileName, matrix transform = cml::identity_4x4(), vec3i color = vec3i(0,0,0));
	bool addPoint(vector<vec3> *vertices, vector<vec3> *normals, vec3 offset, vec3i color = vec3i(0,0,0));
	bool addPoint(vector<vec3> *vertices, vector<vec3> *normals=0, matrix transform = cml::identity_4x4(), vec3i color = vec3i(0,0,0));
	bool addPoint(vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *colors, vec3 offset = vec3(0.f,0.f,0.f));

	bool addLine(vector<vec2i> *indices, vector<vec3> *vertices);
	bool addLine(vector<vec3> *pointPairs);
	bool addLine(vector<vec3> *points1, vector<vec3> *points2);

	bool addComment(string comment);

	bool output(string fileName, bool ascii=false);

public:

	template<class T> static bool exportSample(string fileName, vector<T> *samples, vector<string> *properties, vector<string> *comments = 0);

public:

	vector<vec3> mVertices;
	vector<vec3> mNormals;
	vector<vec3i> mColors;
	vector<vec3i> mFaceIndices;
	vector<vec2i> mEdgeIndices;
	vector<vec3i> mFaceColors;
	vector<string> mComments;

	bool hasNormal;
	bool hasColor;
	bool hasFaceColor;
	
};

#define PLY_EXPORTER_TEMPLATE_DEFINITION

// definition of template member function needs to be placed along with declaration
#include "PlyExporterTemplate.h"