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

#include "Library/TheaKDTreeHelper.h"

#include "Type/MonsterTypes.h"

using namespace std;

namespace Monster {

	class MeshEnvelope {

	private:

		static int PARAM_GRID_DIMENSION;
		static int PARAM_FITTING_ITERATION;
		static int PARAM_SUBDIVISION;

	public:

		MeshEnvelope(TPointSet *samples);
		~MeshEnvelope();

	public:

		bool process();
		bool output(TTriangleMesh &mesh);

	private:

		bool constructEnvelope();
		bool fitEnvelope();

		bool fitMatching();
		bool fitDeformation();
		bool fitRegularization();

		int hashGrid(vec3i pos, vec3i dim);
		vec3i dehashGrid(int pos, vec3i dim);

		bool error(string info);

	private:

		TPointSet *mpSamples;

		SKDTree mSampleTree;
		SKDTreeData mSampleTreeData;

		TTriangleMesh mEnvelopeMesh;

		vector<bool> mFittingMatchingFlag;
		vector<vec3> mFittingMatchingPosition;
	};
}