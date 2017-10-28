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

	class ExtractStroke {

	private:

		ExtractStroke() {}
		~ExtractStroke() {}

	public:

		static bool extract(vector<vector<double>> &sketch, vector<vector<vec2d>> &strokes, string visualFolder = "");

	private:

		static bool extractSkeleton(vector<vector<double>> &sketch, vector<vector<bool>> &skeleton);
		static bool extractComponents(vector<vector<bool>> &sketch, vector<vector<vec2i>> &components);
		static bool chainStroke(vector<vec2i> &component, vector<vector<vec2i>> &chains);
		static bool sampleStroke(vector<vector<vec2i>> &rawStrokes, vector<vector<vec2d>> &sampledStrokes);
		static bool computeBezierPoint(vector<vec2d> &controlPoints, double parameter, vec2d &curvePoint);

	private:

		static string mVisualFolder;
	};

}