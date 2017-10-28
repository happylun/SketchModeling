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

#include <vector>
#include <string>

#include "Library/CMLHelper.h"

using namespace std;

namespace Monster {

	class ExtractContour {

	private:

		ExtractContour() {}
		~ExtractContour() {}

	public:

		static bool extract(vector<vector<double>> &sketch, vector<vec2d> &contour, string visualFolder = "");

	private:

		static bool extractOutline(vector<vector<bool>> &sketch, vector<vec2i> &outline);
		static bool dilateMask(vector<vector<bool>> &mask);
		static bool erodeMask(vector<vector<bool>> &mask);
		static bool connectContourChains(vector<vec2i> &chain, vector<vector<bool>> &sketch);
		static bool sampleContour(vector<vec2i> &contourPixels, vector<vec2d> &contourSamples);
		static bool computeBezierPoint(vector<vec2d> &controlPoints, double parameter, vec2d &curvePoint);

		static bool visualizeChain(string fileName, vector<vec2i> &chain, vec2i imgSizes);
		static bool visualizeContour(string fileName, vector<vec2d> &contour);

	private:

		static string mVisualFolder;
	};

}