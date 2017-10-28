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

class PlyLoader {

private:

	PlyLoader() {}
	~PlyLoader() {}

private:

	struct TEntry {

		static const int TYPE_CHAR = 0;
		static const int TYPE_UCHAR = 1;
		static const int TYPE_SHORT = 2;
		static const int TYPE_USHORT = 3;
		static const int TYPE_INT = 4;
		static const int TYPE_UINT = 5;
		static const int TYPE_FLOAT = 6;
		static const int TYPE_DOUBLE = 7;
		static const int TYPE_LIST = -1;

		static const int PROP_X = 0;
		static const int PROP_Y = 1;
		static const int PROP_Z = 2;
		static const int PROP_NX = 3;
		static const int PROP_NY = 4;
		static const int PROP_NZ = 5;
		static const int PROP_VI = -1;

		vector<vec2i> fields;
		TEntry() : fields(0) {}
	};

	struct THeader {
		int numVertices;
		int numFaces;
		TEntry vertexEntries;
		TEntry faceEntries;
		vector<string> comments;
		bool asciiFlag;
		THeader() : numVertices(0), numFaces(0), comments(0), asciiFlag(false) {}
	};

	struct TPrimitive {
		vec3 position;
		vec3 normal;
		vector<int> indices;
		TPrimitive() : position(0.0f, 0.0f, 0.0f), normal(0.0f, 0.0f, 0.0f), indices(0) {}
	};

public:

	static bool loadMesh(string fileName, vector<vec3> *vertices, vector<vec3> *normals = 0, vector<vec3i> *indices = 0, vector<string> *comments = 0);

	template<class T>
	static bool loadSample(string fileName, vector<T> *samples, vector<string> *comments = 0);

private:

	static bool loadHeader(istream &fileStream, THeader &header);
	static bool loadPrimitive(istream &fileStream, bool asciiFlag, TEntry &entry, TPrimitive &data);

	static bool parseType(string token, int &type);
	static bool parseProp(string token, int &prop);

	template<class T1, class T2>
	static bool loadRawData(istream &stream, T2 &data, bool ascii);

	template<class T>
	static bool loadTypedData(istream &stream, T &data, int type, bool ascii);
};

#define PLY_LOADER_TEMPLATE_DEFINITION

// definition of template member function needs to be placed along with declaration
#include "PlyLoaderTemplate.h"