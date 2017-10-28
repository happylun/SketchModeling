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


#include "PFMReader.h"

#include <iostream>
#include <fstream>

bool PFMReader::error(string s) {
	if (!s.empty()) cout << "Error: " << s << endl;
	system("pause");
	return false;
}

bool PFMReader::flipBuffer(vector<float> &inBuffer, vector<float> &outBuffer, int width, int height, int channel) {

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

bool PFMReader::loadImage(string fileName, vector<float> &image, int &width, int &height, int &channel) {

	ifstream file(fileName, ios::binary);
	if (!file.is_open()) return error("cannot open PFM file");
	char c;

	file.read(&c, 1);
	if (c != 'P') return error("incorrect pfm header");
	file.read(&c, 1);
	if (c == 'f') channel = 1;
	else if (c == 'F') channel = 3;
	else return error("incorrect pfm header");
	file.read(&c, 1);
	if (c != char(10)) return error("incorrect pfm header");

	string szW = "";
	while (true) {
		file.read(&c, 1);
		if (file.fail()) return error("incorrect pfm header");
		if (c == ' ') break;
		szW += c;
	}
	width = stoi(szW);

	string szH = "";
	while (true) {
		file.read(&c, 1);
		if (file.fail()) return error("incorrect pfm header");
		if (c == char(10)) break;
		szH += c;
	}
	height = stoi(szH);

	while (true) {
		file.read(&c, 1);
		if (file.fail()) return error("incorrect pfm header");
		if (c == char(10)) break;
	}

	vector<float> buffer(width*height*channel);
	file.read((char*)(buffer.data()), buffer.size()*sizeof(float));
	if (file.fail()) return error("incorrect pfm content");

	file.close();

	if (!flipBuffer(buffer, image, width, height, channel)) return false;

	return true;
}

bool PFMReader::loadMonoImage(string fileName, Eigen::MatrixXf &image) {

	vector<float> rawData;
	int width, height, channel;
	if (!loadImage(fileName, rawData, width, height, channel)) return false;
	if (channel != 1) return error("incorrect number of channels for mono pfm");

	image = Eigen::Map<Eigen::MatrixXf>(rawData.data(), width, height).transpose();

	return true;
}

bool PFMReader::loadColorImage(string fileName, vector<vector<vec3>> &image) {

	vector<float> rawData;
	int width, height, channel;
	if (!loadImage(fileName, rawData, width, height, channel)) return false;
	if (channel != 3) return error("incorrect number of channels for color pfm");

	image.resize(height);
	int ptr = 0;
	for (int row = 0; row < height; row++) {
		image[row].resize(width);
		for (int col = 0; col < width; col++) {
			image[row][col] = vec3(rawData[ptr], rawData[ptr + 1], rawData[ptr + 2]);
			ptr += 3;
		}
	}

	return true;
}