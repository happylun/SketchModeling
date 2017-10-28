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


#include "SampleRandomness.h"

using namespace Monster;

double SampleRandomness::getPreciseRandomNumber() {

	long long rng = rand()*(RAND_MAX + 1) + rand();
	return double(rng) / ((RAND_MAX + 1)*(RAND_MAX + 1));
}

int SampleRandomness::samplingCDF(vector<double> &cdf) {

	int n = (int)cdf.size();

	double rng = SampleRandomness::getPreciseRandomNumber();
	// binary search
	int p1 = -1;
	int p2 = n - 1;
	while (p1 + 1 < p2) {
		int pm = (p1 + p2) / 2;
		if (cdf[pm] < rng) {
			p1 = pm;
		} else {
			p2 = pm;
		}
	}

	int sample = p2;
	if (rng > 0) {
		// choose first valid sample (leftmost among samples with the same CDF)
		while (sample > 0 && cdf[sample] == cdf[sample-1]) sample--;
	} else {
		// choose first valid sample (first with non-zero CDF)
		while (sample < n-1 && cdf[sample] == 0) sample++;
	}

	return sample;
}