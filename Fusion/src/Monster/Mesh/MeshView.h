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

#include "Library/CMLHelper.h"
#include "Library/EigenHelper.h"

using namespace std;

namespace Monster {

	class MeshView {

	private:

		// make it non-instantiable
		MeshView() {}
		~MeshView() {}

	public:

		static double VIEW_RADIUS;

	public:

		static bool loadViewPoints(string fileName, vector<Eigen::Vector3d> &viewPoints, vector<vector<int>> *ptrViewGroups = 0);
		static bool buildViewMatrix(Eigen::Vector3d &inViewPoint, Eigen::Matrix4d &outViewMat);
		static bool buildProjMatrix(Eigen::Matrix4d &outMat,
									double l = -VIEW_RADIUS, double r = VIEW_RADIUS,
									double b = -VIEW_RADIUS, double t = VIEW_RADIUS,
									double n = 0.1, double f = VIEW_RADIUS*2.0);
		static bool buildImageMatrix(Eigen::Matrix4d &outMat, int imageWidth, int imageHeight, double shift = -0.5);

	private:

		static bool error(string s);

	};
}