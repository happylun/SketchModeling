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
#include "Library/EigenHelper.h"

using namespace std;

class PFMReader {

private:

	PFMReader() {}
	~PFMReader() {}

public:

	static bool loadImage(string fileName, vector<float> &image, int &width, int &height, int &channel);

	static bool loadMonoImage(string fileName, Eigen::MatrixXf &image);
	static bool loadColorImage(string fileName, vector<vector<vec3>> &image);

	static bool flipBuffer(vector<float> &inBuffer, vector<float> &outBuffer, int width, int height, int channel);

private:

	static bool error(string s);
};