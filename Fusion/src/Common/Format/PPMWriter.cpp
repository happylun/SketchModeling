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


#include "PPMWriter.h"

#include <iostream>
#include <fstream>

bool PPMWriter::writeImage(string fileName, vector<unsigned char> &image, int width, int height, int channel) {

	int bufferSize = (int)image.size();
	if (bufferSize != width*height*channel) {
		cout << "Error: incorrect image size:" << endl;
		cout << width << " x " << height << " x " << channel << " != " << bufferSize << endl;
		return false;
	}
	if (channel != 1 && channel != 3) {
		cout << "Error: cannot write PPM image with " << channel << " channels" << endl;
		return false;
	}

	ofstream file(fileName);
	file << "P6" << endl;
	file << width << " " << height << " " << 255 << " ";
	file.close();
	file.open(fileName, ios::binary | ios::app);
	if (channel == 1) {
		for (auto c : image) {
			file.write((const char*)(&c), 1);
			file.write((const char*)(&c), 1);
			file.write((const char*)(&c), 1);
		}
	} else if (channel == 3) {
		for (auto c : image) {
			file.write((const char*)(&c), 1);
		}
	}
	file.close();

	return true;
}