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


#include <iostream>
#include <fstream>

#include "OptimizeViews.h"
#include "OptimizeMesh.h"

#include "Utility/TimerUtility.h"
#include "Utility/FileUtil.h"

using namespace std;
using namespace Monster;

int error(string s) {
	if(!s.empty()) cout << "Error: " << s << endl;
	system("pause");
	return -1;
}

int main(int argc, char** argv) {

	Eigen::initParallel();

	/*
	1 FS
	./CharacterDraw/hires/m1/
	./CharacterDraw/output/images/m1/
	./CharacterDraw/output/reconstruct/m1/
	./CharacterDraw/view/view.off
	*/
	if (argc < 7) {
		cout << "Usage: " << argv[0] << endl;
		cout << "\t stage(1:fusion; 2:fine-tune)" << endl;
		cout << "\t sketch_views(F/S/T)" << endl;
		cout << "\t sketch_folder map_folder" << endl;
		cout << "\t result_folder view_point_file" << endl;
		cout << "\t [--skip_optimization] [--symmetrization]" << endl;
		return -1;
	}
	int stage = stoi(argv[1]);
	string sketchViews = argv[2];
	string sketchFolder = argv[3];
	string mapFolder = argv[4];
	string resultFolder = argv[5];
	string viewPointFile = argv[6];
	bool bSkipOptimization = false;
	bool bSymmetrization = false;
	for (int k = 7; k < argc; k++) {
		if (string(argv[k]) == "--skip_optimization") bSkipOptimization = true;
		if (string(argv[k]) == "--symmetrization") bSymmetrization = true;
	}

	if (!FileUtil::makedir(resultFolder)) return error("");

	auto timer = TimerUtility::tic();

	if (stage == 1) {
		OptimizeViews ov;
		if (!ov.init(sketchViews, sketchFolder, mapFolder, resultFolder, viewPointFile)) return error("");
		if (!ov.process(bSkipOptimization, bSymmetrization)) return error("");
	} else if (stage == 2) {
		OptimizeMesh om;
		if (!om.init(sketchViews, sketchFolder, mapFolder, resultFolder, viewPointFile)) return error("");
		if (!om.process()) return error("");
	}

	cout << "Total time: " << TimerUtility::toString(TimerUtility::toc(timer)) << endl;

	//system("pause");

	return 0;
}
