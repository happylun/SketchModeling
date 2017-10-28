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


#include "FileUtil.h"

#include <fstream>
#include <vector>

#include <direct.h>

bool FileUtil::makedir(string path) {

	vector<string> tree;
	string folder = path;
	string::size_type pos = folder.find_last_of("/\\");
	while (pos != string::npos) {
		folder = folder.substr(0, pos);
		tree.push_back(folder);
		pos = folder.find_last_of("/\\");
	}

	for (auto it = tree.rbegin(); it != tree.rend(); it++) {
		_mkdir(it->c_str());
	}

	return true;
}

bool FileUtil::existsfile(string file) {

	ifstream f(file);
	if (!f.is_open()) return false;
	f.close();

	return true;
}

bool FileUtil::copyfile(string from, string to) {

	// ref: http://stackoverflow.com/a/10195497

	if (!existsfile(from)) return false;
	if (!makedir(to)) return false;

	ifstream src(from, ios::binary);
	ofstream dst(to, ios::binary);
	dst << src.rdbuf();

	return true;
}