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


#include "ImageIO.h"

#include <iostream>

#include "Format/PPMWriter.h"
#include "Format/PFMWriter.h"
#include "Format/ImageWriter.h"
#include "Format/ImageReader.h"

using namespace Monster;

bool ImageIO::writeImage(
	string fileName, vector<unsigned char> &image,
	int width, int height, int channel)
{
	string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "ppm") {
		if (!PPMWriter::writeImage(fileName, image, width, height, channel)) return false;
	} else {
		if (!ImageWriter::writeImage(fileName, image, width, height, channel)) return false;
	}

	return true;
}

bool ImageIO::writeImage(
	string fileName, vector<unsigned short> &image,
	int width, int height, int channel)
{
	
	return ImageWriter::writeImage(fileName, image, width, height, channel);
}

bool ImageIO::writeImage(
	string fileName, vector<float> &image,
	int width, int height, int channel)
{
	string ext = fileName.substr(fileName.find_last_of(".") + 1);

	if (ext == "pfm") {
		if (!PFMWriter::writeImage(fileName, image, width, height, channel)) return false;
	} else {
		cout << "Error: unknown file type " << ext << endl;
		return false;
	}

	return true;
}

bool ImageIO::readImage(
	string fileName, vector<unsigned char> &image,
	int &width, int &height, int &channel)
{
	return ImageReader::readImage(fileName, image, width, height, channel);
}

bool ImageIO::readImage(
	string fileName, vector<unsigned short> &image,
	int &width, int &height, int &channel)
{
	return ImageReader::readImage(fileName, image, width, height, channel);
}
