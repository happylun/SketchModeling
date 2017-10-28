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


#include "PlyLoader.h"

#include <fstream>
#include <sstream>

#include "Utility/StringUtil.h"

bool PlyLoader::loadMesh(string fileName, vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *indices, vector<string> *comments) {

	if(!vertices) return false;

	vertices->clear();
	if(normals) normals->clear();
	if(indices) indices->clear();

	ifstream file;

	file.open(fileName, ios::binary);
	if(!file) {
		cout << "File " << fileName << " does not exist!" << endl;
		return false;
	}

	THeader header;
	if (!loadHeader(file, header)) return false;
	if (comments) *comments = header.comments;
	
	for (int vertexID = 0; vertexID < header.numVertices; vertexID++) {
		TPrimitive data;
		if (!loadPrimitive(file, header.asciiFlag, header.vertexEntries, data)) return false;
		vertices->push_back(data.position);
		if (normals) normals->push_back(data.normal);
	}

	if (indices) {
		for (int faceID = 0; faceID < header.numFaces; faceID++) {
			TPrimitive data;
			if (!loadPrimitive(file, header.asciiFlag, header.faceEntries, data)) return false;
			int n = (int)data.indices.size();
			for (int k = 1; k < n - 1; k++) {
				indices->push_back(vec3i(data.indices[0], data.indices[k], data.indices[k + 1]));
			}
		}
	}

	file.close();

	return true;
}

bool PlyLoader::loadHeader(istream &fileStream, THeader &header) {

	string line;
	bool vertexProp = false;
	bool faceProp = false;
	while (true) {
		getline(fileStream, line, '\n');
		string rawLine = line;
		line = StringUtil::toLower(line);
		if (line.find("format") != string::npos) {
			stringstream ss(line);
			string s;
			ss >> s; ss >> s;
			if (s == "binary_little_endian") {
				header.asciiFlag = false;
			} else if (s == "ascii") {
				header.asciiFlag = true;
			} else {
				cout << "Error: unknown format " << s << endl;
				return false;
			}
		}
		if (line.find("comment") != string::npos) {
			header.comments.push_back(rawLine);
		}
		if (line.find("element") != string::npos) {
			stringstream ss(line);
			string s;
			ss >> s; ss >> s;
			if (s == "vertex") {
				vertexProp = true;
				faceProp = false;
				ss >> header.numVertices;
			} else if (s == "face") {
				faceProp = true;
				vertexProp = false;
				ss >> header.numFaces;
			} else {
				cout << "Error: unknown element " << s << endl;
				return false;
			}
		}
		if (line.find("property") != string::npos) {
			if (!vertexProp && !faceProp) {
				cout << "Error: element type is not specified" << endl;
				return false;
			}
			stringstream ss(line);
			string s;
			ss >> s; ss >> s;
			vec2i field, extraField;
			if (!parseType(s, field[0])) return false;
			if (field[0] == TEntry::TYPE_LIST) {
				ss >> s; if (!parseType(s, extraField[0])) return false;
				ss >> s; if (!parseType(s, extraField[1])) return false;
			}
			ss >> s; if (!parseProp(s, field[1])) return false;

			if (vertexProp) {
				header.vertexEntries.fields.push_back(field);
				if (field[0] == TEntry::TYPE_LIST) {
					header.vertexEntries.fields.push_back(extraField);
				}
			}
			
			if (faceProp) {
				header.faceEntries.fields.push_back(field);
				if (field[0] == TEntry::TYPE_LIST) {
					header.faceEntries.fields.push_back(extraField);
				}
			}
		}
		if (line.find("end_header") != string::npos) {
			break;
		}
	}

	return true;
}

bool PlyLoader::loadPrimitive(istream &fileStream, bool asciiFlag, TEntry &entry, TPrimitive &data) {

	int numFields = (int)entry.fields.size();

	int fieldID = 0;
	while (fieldID < numFields) {
		vec2i field = entry.fields[fieldID];
		if (field[0] != TEntry::TYPE_LIST) {
			// single entry
			float fieldData;
			if (!loadTypedData(fileStream, fieldData, field[0], asciiFlag)) return false;

			switch (field[1]) {
			case TEntry::PROP_X:
				data.position[0] = fieldData; break;
			case TEntry::PROP_Y:
				data.position[1] = fieldData; break;
			case TEntry::PROP_Z:
				data.position[2] = fieldData; break;
			case TEntry::PROP_NX:
				data.normal[0] = fieldData; break;
			case TEntry::PROP_NY:
				data.normal[1] = fieldData; break;
			case TEntry::PROP_NZ:
				data.normal[2] = fieldData; break;
			}
		} else {
			// list
			vec2i extraField = entry.fields[++fieldID];
			int listSize;
			if (!loadTypedData(fileStream, listSize, extraField[0], asciiFlag)) return false;
			vector<int> listData(listSize);
			for (int id = 0; id < listSize; id++) {
				if (!loadTypedData(fileStream, listData[id], extraField[1], asciiFlag)) return false;
			}

			switch (field[1]) {
			case TEntry::PROP_VI:
				data.indices = listData; break;
			}
		}
		fieldID++;
	}

	return true;
}

bool PlyLoader::parseType(string token, int &type) {

	if (token == "char") {
		type = TEntry::TYPE_CHAR;
	} else if (token == "uchar") {
		type = TEntry::TYPE_UCHAR;
	} else if (token == "short") {
		type = TEntry::TYPE_SHORT;
	} else if (token == "ushort") {
		type = TEntry::TYPE_USHORT;
	} else if (token == "int") {
		type = TEntry::TYPE_INT;
	} else if (token == "uint") {
		type = TEntry::TYPE_UINT;
	} else if (token == "float") {
		type = TEntry::TYPE_FLOAT;
	} else if (token == "double") {
		type = TEntry::TYPE_DOUBLE;
	} else if (token == "list") {
		type = TEntry::TYPE_LIST;
	} else {
		cout << "Error: unknown type " << token << endl;
		return false;
	}

	return true;
}

bool PlyLoader::parseProp(string token, int &prop) {

	if (token == "x") {
		prop = TEntry::PROP_X;
	} else if (token == "y") {
		prop = TEntry::PROP_Y;
	} else if (token == "z") {
		prop = TEntry::PROP_Z;
	} else if (token == "nx") {
		prop = TEntry::PROP_NX;
	} else if (token == "ny") {
		prop = TEntry::PROP_NY;
	} else if (token == "nz") {
		prop = TEntry::PROP_NZ;
	} else if (token == "vertex_index") {
		prop = TEntry::PROP_VI;
	} else if (token == "vertex_indices") {
		prop = TEntry::PROP_VI;
	} else {
		cout << "Error: unknown property " << token << endl;
		return false;
	}

	return true;
}