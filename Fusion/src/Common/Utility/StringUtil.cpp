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


#include "StringUtil.h"

#include <sstream>
#include <algorithm>
#include <functional>
#include <cctype>


void StringUtil::split(string line, char delim, vector<string> &tokens) {

	tokens.clear();
	stringstream ss(line);
	string item;
	while (getline(ss, item, delim)) tokens.push_back(trim(item));
}

string StringUtil::trim(string s) {

	// ref: http://stackoverflow.com/a/217605

	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
	return s;
}

string StringUtil::simplify(string s) {

	// ref: http://stackoverflow.com/a/5561835

	s.erase(std::unique(s.begin(), s.end(), [](char l, char r){return (l == r) && (l == ' '); }), s.end());
	return s;
}

string StringUtil::toLower(string s) {

	// ref: http://stackoverflow.com/a/313990/1056666

	string o = s;
	std::transform(s.begin(), s.end(), o.begin(), ::tolower);
	return o;
}

string StringUtil::toUpper(string s) {

	string o = s;
	std::transform(s.begin(), s.end(), o.begin(), ::toupper);
	return o;
}