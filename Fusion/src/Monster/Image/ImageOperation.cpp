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


#include "ImageOperation.h"

#include <iostream>

using namespace Monster;

bool ImageOperation::operate(
	vector<unsigned char> &inImage,
	vector<unsigned char> &outImage,
	int width, int height, int channel, int kernel,
	function<unsigned char(unsigned char, unsigned char)> func)
{
	if (width * height * channel != (int)inImage.size()) {
		cout << "Error: incorrect image size" << endl;
		return false;
	}

	// slow implementation...

	vector<int> kernelU(0), kernelV(0), kernelO(0);
	int kernelLB = -(kernel - 1) / 2;
	int kernelUB = kernel / 2;
	for (int v = kernelLB; v <= kernelUB; v++) {
		for (int u = kernelLB; u <= kernelUB; u++) {
			kernelU.push_back(u);
			kernelV.push_back(v);
			kernelO.push_back((v * width + u) * channel);
		}
	}
	int kernelSize = (int)kernelO.size();

	vector<unsigned char> buffer = inImage;
	int ptr = 0;
	for (int h = 0; h < height; h++) {
		for (int w = 0; w < width; w++) {
			for (int k = 0; k < kernelSize; k++) {
				int v = h + kernelV[k];
				int u = w + kernelU[k];
				if (v<0 || v>width || u<0 || u>height) continue;
				int kptr = ptr + kernelO[k];
				for (int c = 0; c < channel; c++) {
					buffer[ptr + c] = func(buffer[ptr + c], inImage[kptr + c]);
				}
			}
			ptr += channel;
		}
	}

	outImage.swap(buffer);

	return true;
}

bool ImageOperation::dilate(
	vector<unsigned char> &inImage,
	vector<unsigned char> &outImage,
	int width, int height, int channel, int kernel)
{
	auto func = [](unsigned char a, unsigned char b) {return a > b ? a : b; };
	return operate(inImage, outImage, width, height, channel, kernel, func);
}

bool ImageOperation::erode(
	vector<unsigned char> &inImage,
	vector<unsigned char> &outImage,
	int width, int height, int channel, int kernel)
{
	auto func = [](unsigned char a, unsigned char b) {return a < b ? a : b; };
	return operate(inImage, outImage, width, height, channel, kernel, func);
}