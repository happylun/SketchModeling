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
#include "Library/EigenHelper.h"

#include "Type/MonsterTypes.h"

using namespace std;

namespace Monster {

	class RenderMesh {

	private:

		RenderMesh() {}
		~RenderMesh() {}

	public:

		static bool init(int imageSize);
		static bool finish();

		static bool render(
			TTriangleMesh &mesh,
			Eigen::Matrix4f &viewMatrix,
			Eigen::Matrix4f &rotateMatrix,
			vector<vector<bool>> &masks,
			vector<vector<double>> &depths,
			vector<vector<vec3d>> &normals);

	private:

		static bool initGL(int imageSize);
		static bool initShader();
		static bool checkGLError(int checkPoint = 0);
		static bool error(string s);

	private:

		static unsigned int mTexID;
		static unsigned int mFBOID;
		static unsigned int mRBID;
		static unsigned int mShaderID;

		static int mImageSize;
	};

}