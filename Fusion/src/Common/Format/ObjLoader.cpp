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


#include "ObjLoader.h"

#include <fstream>
#include <sstream>
#include <map>

#include "Utility/StringUtil.h"

bool ObjLoader::loadMesh(string fileName,
	vector<vec3i> *indices, vector<vec3> *vertices,
	vector<int> *labels, vector<string> *labelNames)
{

	if(!indices) return false;
	if(!vertices) return false;

	bool outputLabel = (labels && labelNames);

	ifstream file(fileName);
	if (!file.is_open()) {
		cout << "Cannot open file " << fileName << endl;
		return false;
	}

	vertices->clear();
	indices->clear();

	if (outputLabel) {
		labels->clear();
		labelNames->clear();
	}
	map<string, int> labelMap;
	int numLabels = 0;
	int currentLabel = 0;

	while (!file.eof()) {
		string line;
		getline(file, line);
		if (file.fail()) break;
		line = StringUtil::trim(StringUtil::simplify(line));
		if (line.empty() || line[0] == '#') continue;

		vector<string> tokens;
		StringUtil::split(line, ' ', tokens);

		if (tokens[0] == "v") {
			if ((int)tokens.size() < 4) {
				cout << "Error: incorrect V line \"" << line << "\"" << endl;
				return false;
			}
			vec3 p;
			for (int k = 0; k < 3; k++) {
				stringstream ss(tokens[k + 1]);
				ss >> p[k];
			}
			vertices->push_back(p);
		} else if (tokens[0] == "f") {
			if ((int)tokens.size() < 4) {
				cout << "Error: incorrect F line \"" << line << "\"" << endl;
				return false;
			}
			vector<int> vi;
			for (int k = 1; k < (int)tokens.size(); k++) {
				stringstream ss(tokens[k]);
				int id; ss >> id;
				vi.push_back(id - 1); // 0-indexed
			}
			for (int k = 1; k < (int)vi.size() - 1; k++) {
				indices->push_back(vec3i(vi[0], vi[k], vi[k + 1]));
				if (outputLabel) labels->push_back(currentLabel);
			}
		} else if (tokens[0] == "g") {
			if (!outputLabel) continue;
			if ((int)tokens.size() < 2) {
				cout << "Error: incorrect G line \"" << line << "\"" << endl;
				return false;
			}

			if (!labels->empty() && labelNames->empty()) {
				string defaultName = "noname";
				labelNames->push_back(defaultName);
				labelMap[defaultName] = numLabels;
				numLabels++;
			}
			
			string name = tokens[1];
			if (labelMap.find(name) == labelMap.end()) {
				labelNames->push_back(name);
				labelMap[name] = numLabels;
				currentLabel = numLabels;
				numLabels++;
			} else {
				currentLabel = labelMap[name];
			}
		}
	}

	file.close();

	return true;
}
