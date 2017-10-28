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


#include "SampleIO.h"

#include "Format/PlyExporter.h"
#include "Format/PlyLoader.h"

using namespace Monster;

bool SampleIO::saveSample(string fileName, TSampleSet &samples) {

	vector<TPlySample> outSamples(samples.amount);
	for (int sampleID = 0; sampleID < samples.amount; sampleID++) {
		TPlySample &sample = outSamples[sampleID];
		sample.p = samples.positions[sampleID];
		sample.n = samples.normals[sampleID];
		sample.f = samples.indices[sampleID];
	}

	vector<string> outProperties;
	outProperties.push_back("float x");
	outProperties.push_back("float y");
	outProperties.push_back("float z");
	outProperties.push_back("float nx");
	outProperties.push_back("float ny");
	outProperties.push_back("float nz");
	outProperties.push_back("uint faceID");

	stringstream ss; ss << "SAMPLE_RADIUS " << samples.radius;
	vector<string> outComments;
	outComments.push_back(ss.str());

	if (!PlyExporter::exportSample(fileName, &outSamples, &outProperties, &outComments)) return false;

	return true;
}

bool SampleIO::loadSample(string fileName, TSampleSet &samples) {

	vector<TPlySample> inSamples;
	vector<string> comments;

	if (!PlyLoader::loadSample(fileName, &inSamples, &comments)) return false;
	samples.positions.clear();
	samples.normals.clear();
	samples.indices.clear();
	for (TPlySample &sample : inSamples) {
		samples.positions.push_back(sample.p);
		samples.normals.push_back(sample.n);
		samples.indices.push_back(sample.f);
	}
	samples.amount = (int)samples.positions.size();

	samples.radius = 0;
	for (auto line : comments) {
		string::size_type pos = line.find("SAMPLE_RADIUS");
		if (pos != string::npos) {
			stringstream ss(line.substr(pos));
			string s; ss >> s;
			ss >> samples.radius;
			break;
		}
	}
	if (samples.radius == 0) {
		cout << "Error: no sample radius information" << endl;
		return false;
	}

	return true;
}
