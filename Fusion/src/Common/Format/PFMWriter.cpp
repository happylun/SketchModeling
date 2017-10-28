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


#include "PFMWriter.h"

#include <iostream>
#include <fstream>

bool PFMWriter::flipBuffer(vector<float> &inBuffer, vector<float> &outBuffer, int width, int height, int channel) {

	// flip buffer upside down

	if ((int)inBuffer.size() != width*height*channel) return false;

	outBuffer.resize(inBuffer.size());
	for (int row = 0; row < height; row++) {
		auto iterFirst = inBuffer.begin() + row*width*channel;
		auto iterLast = iterFirst + width*channel;
		auto iterDest = outBuffer.begin() + (height - row - 1)*width*channel;
		copy(iterFirst, iterLast, iterDest);
	}
	return true;
}

bool PFMWriter::writeImage(string fileName, vector<float> &image, int width, int height, int channel) {

	// ref: http://www.pauldebevec.com/Research/HDR/PFM/
	// ref: http://netpbm.sourceforge.net/doc/pfm.html

	int bufferSize = (int)image.size();
	if (bufferSize != width*height*channel) {
		cout << "Error: incorrect image size:" << endl;
		cout << width << " x " << height << " x " << channel << " != " << bufferSize << endl;
		return false;
	}
	if (channel != 1 && channel != 3) {
		cout << "Error: cannot write PFM image with " << channel << " channels" << endl;
		return false;
	}

	// PFM format will flip the buffer upside down by nature...
	vector<float> buffer;
	if (!flipBuffer(image, buffer, width, height, channel)) return false;

	ofstream file(fileName, ios::binary);
	file << (channel == 1 ? "Pf" : "PF") << char(10); // identifier line
	file << to_string(width) << " " << to_string(height) << char(10); // dimension line
	file << to_string(-1.0) << char(10); // scale factor / endianness
	for (auto c : buffer) {
		file.write((const char*)(&c), sizeof(c));
	}
	file.close();

	return true;
}