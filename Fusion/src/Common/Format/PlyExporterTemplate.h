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

// DO NOT INCLUDE THIS FILE EXCEPT IN FILE "PlyExporter.h"

#include <fstream>

#ifdef PLY_EXPORTER_TEMPLATE_DEFINITION

template<class T> bool PlyExporter::exportSample(string fileName, vector<T> *samples, vector<string> *properties, vector<string> *comments) {

	if(!samples) return false;
	if (!properties) return false;

	ofstream outFile;

	outFile.open(fileName);
	if(!outFile) {
		cout << "Cannot write data to file " << fileName << "!" << endl;
		return false;
	}

	outFile << "ply" << endl;
	outFile << "format binary_little_endian 1.0" << endl;

	if (comments) {
		for (string comment : (*comments)) {
			outFile << "comment " << comment << endl;
		}
	}
	outFile << "element vertex " << ((int)(samples->size())) << endl;

	for (string property : (*properties)) {
		outFile << "property " << property << endl;
	}

	outFile << "end_header" << endl;

	outFile.close();
	outFile.open(fileName, ios::app | ios::binary);

	for (T &sample : (*samples)) {
		outFile.write((char*)&sample, sizeof(sample));
	}
	outFile.close();

	return true;
}

#endif