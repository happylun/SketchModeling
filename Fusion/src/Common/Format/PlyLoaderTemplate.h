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

// definition of template member function

// DO NOT INCLUDE THIS FILE EXCEPT IN FILE "PlyLoader.h"

#ifdef PLY_LOADER_TEMPLATE_DEFINITION

template<class T>
bool PlyLoader::loadSample(string fileName, vector<T> *samples, vector<string> *comments) {

	if(!samples) return false;

	int numSamples = 0;

	samples->clear();

	ifstream file;

	file.open(fileName, ios::binary);
	if(!file) {
		cout << "File " << fileName << " does not exist!" << endl;
		return false;
	}

	string line;
	while(true) {
		getline(file, line, '\n');
		if( comments && line.find("comment") != string::npos) {
			comments->push_back(line);
		}
		if( line.find("element vertex") != string::npos) {
			stringstream ss(line);
			string s;
			ss >> s; ss >> s;
			ss >> numSamples;
		}
		if( line.find("end_header") != string::npos) {
			break;
		}
	}

	for(int i=0; i<numSamples; i++) {
		T sample;
		file.read((char*)&sample, sizeof(sample));
		samples->push_back(sample);
	}
	file.close();

	return true;
}

template<class T1, class T2>
bool PlyLoader::loadRawData(istream &stream, T2 &data, bool ascii) {

	T1 value;
	if (ascii) {
		stream >> value;
	} else {
		stream.read((char*)&value, sizeof(value));
	}
	if (stream.fail()) {
		cout << "Error: fail to load raw data" << endl;
	}
	data = (T2)value;

	return true;
}

template<class T>
bool PlyLoader::loadTypedData(istream &stream, T &data, int type, bool ascii) {

	switch (type) {
	case TEntry::TYPE_CHAR:
		return loadRawData<char>(stream, data, ascii);
	case TEntry::TYPE_UCHAR:
		return loadRawData<unsigned char>(stream, data, ascii);
	case TEntry::TYPE_SHORT:
		return loadRawData<short>(stream, data, ascii);
	case TEntry::TYPE_USHORT:
		return loadRawData<unsigned short>(stream, data, ascii);
	case TEntry::TYPE_INT:
		return loadRawData<int>(stream, data, ascii);
	case TEntry::TYPE_UINT:
		return loadRawData<unsigned int>(stream, data, ascii);
	case TEntry::TYPE_FLOAT:
		return loadRawData<float>(stream, data, ascii);
	case TEntry::TYPE_DOUBLE:
		return loadRawData<double>(stream, data, ascii);
	default:
		cout << "Error: unknown type " << type << endl;
	}
	return false;
}

#endif