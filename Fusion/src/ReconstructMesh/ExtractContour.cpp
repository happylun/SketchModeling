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


#include "ExtractContour.h"

#include <set>
#include <queue>

#include "MapsData.h"

#include "Mesh/MeshIO.h"

using namespace Monster;

#define DEBUG_VISUALIZATION

string ExtractContour::mVisualFolder = "";

bool ExtractContour::extract(vector<vector<double>> &sketch, vector<vec2d> &contour, string visualFolder) {

	int imgHeight = (int)sketch.size();
	int imgWidth = (int)sketch[0].size();

	mVisualFolder = visualFolder;

	// mark bool flag and find start point

	vector<vector<bool>> boolSketch(imgHeight, vector<bool>(imgWidth, false));
	vector<vector<bool>> inspected(imgHeight, vector<bool>(imgWidth, false));
	vec2i startPoint(-1,-1);
	for (int row = 0; row < imgHeight; row++) {
		for (int col = 0; col < imgWidth; col++) {
			boolSketch[row][col] = (sketch[row][col] > 0.0); // UNDONE: param contour sketch threshold
			if (boolSketch[row][col] && startPoint[0] < 0) {
				startPoint = vec2i(row, col);
				inspected[row][col] = true;
			}
		}
	}

#ifdef DEBUG_VISUALIZATION
	// visualize contour
	if (true) {
		vector<vector<double>> contoursSketch(imgHeight, vector<double>(imgWidth, 0.0));
		for (int row = 0; row < imgHeight; row++) {
			for (int col = 0; col < imgWidth; col++) {
				if (boolSketch[row][col]) contoursSketch[row][col] = 1.0;
			}
		}
		if (!MapsData::visualizeSketch(mVisualFolder + "contour-sketch.png", contoursSketch)) return false;
	}
#endif

	vector<vec2i> contourPixels;
	if (!extractOutline(boolSketch, contourPixels)) return false;

#ifdef DEBUG_VISUALIZATION
	if (!visualizeChain(mVisualFolder + "unconnected.png", contourPixels, vec2i(imgHeight, imgWidth))) return false;
#endif

	if (!connectContourChains(contourPixels, boolSketch)) return false;

#ifdef DEBUG_VISUALIZATION
	if (!visualizeChain(mVisualFolder + "contour.png", contourPixels, vec2i(imgHeight, imgWidth))) return false;
	//system("pause");
#endif

	if (!sampleContour(contourPixels, contour)) return false;

#ifdef DEBUG_VISUALIZATION
	if (!visualizeContour(mVisualFolder + "contour.ply", contour)) return false;
#endif

	return true;
}

bool ExtractContour::extractOutline(vector<vector<bool>> &sketch, vector<vec2i> &outline) {

	int imgHeight = (int)sketch.size();
	int imgWidth = (int)sketch[0].size();

	vector<vec2i> offset4 = { vec2i(1, 0), vec2i(0, 1), vec2i(-1, 0), vec2i(0, -1) };
	vector<vec2i> offset8 = {
		vec2i(1, 1), vec2i(1, 0), vec2i(0, 1), vec2i(1, -1),
		vec2i(-1, 1), vec2i(-1, 0), vec2i(0, -1), vec2i(-1, -1)
	};

	// close operation

	vector<vector<bool>> mask = sketch;
	int numOperations = imgHeight / 128;
	for (int k = 0; k < numOperations; k++) {
		if (!dilateMask(mask)) return false;
	}
#ifdef DEBUG_VISUALIZATION
	//if (!MapsData::visualizeMask(mVisualFolder + "dilate.png", mask)) return false;
#endif
	for (int k = 0; k < numOperations; k++) {
		if (!erodeMask(mask)) return false;
	}
#ifdef DEBUG_VISUALIZATION
	//if (!MapsData::visualizeMask(mVisualFolder + "erode.png", mask)) return false;
#endif

	// mark background

	vector<vector<int>> labels(imgHeight, vector<int>(imgWidth, -1));
	if (true) {
		labels[0][0] = 0;
		vector<vec2i> queue(1, vec2i(0, 0));
		int head = 0;
		while (head < (int)queue.size()) {
			vec2i curPos = queue[head];
			for (vec2i offset : offset4) {
				vec2i nextPos = curPos + offset;
				if (nextPos[0] < 0 || nextPos[0] >= imgHeight || nextPos[1] < 0 || nextPos[1] >= imgWidth) continue;
				if (mask[nextPos[0]][nextPos[1]]) continue;
				if (labels[nextPos[0]][nextPos[1]] == 0) continue;
				queue.push_back(nextPos);
				labels[nextPos[0]][nextPos[1]] = 0;
			}
			head++;
		}
	}

	// mark components

	int mainComponent = -1;
	int mainComponentSize = 0;
	if (true) {
		int numComponents = 0;
		for (int row = 0; row < imgHeight; row++) {
			for (int col = 0; col < imgWidth; col++) {
				if (labels[row][col] >= 0) continue;
				numComponents++;

				labels[row][col] = numComponents;
				vector<vec2i> queue(1, vec2i(row, col));
				int head = 0;
				while (head < (int)queue.size()) {
					vec2i curPos = queue[head];
					for (vec2i offset : offset4) {
						vec2i nextPos = curPos + offset;
						if (nextPos[0] < 0 || nextPos[0] >= imgHeight || nextPos[1] < 0 || nextPos[1] >= imgWidth) continue;
						if (labels[nextPos[0]][nextPos[1]] >= 0) continue;
						queue.push_back(nextPos);
						labels[nextPos[0]][nextPos[1]] = numComponents;
					}
					head++;
				}

				if ((int)queue.size() > mainComponentSize) {
					mainComponentSize = (int)queue.size();
					mainComponent = numComponents;
				}
			}
		}
	}

	// mark contour

	vector<vector<bool>> flags(imgHeight, vector<bool>(imgWidth, false));
	vec2i startPoint(-1, -1);
	for (int row = 0; row < imgHeight; row++) {
		for (int col = 0; col < imgWidth; col++) {
			if (labels[row][col] != mainComponent) continue;
			bool valid = false;
			for (vec2i offset : offset4) {
				int nbRow = cml::clamp(offset[0] + row, 0, imgHeight - 1);
				int nbCol = cml::clamp(offset[1] + col, 0, imgWidth - 1);
				if ((nbRow == row && nbCol == col) || labels[nbRow][nbCol] == 0) {
					valid = true;
					break;
				}
			}
			if (valid) {
				flags[row][col] = true;
				if (startPoint[0] < 0) startPoint = vec2i(row, col);
			}
		}
	}
#ifdef DEBUG_VISUALIZATION
	if (!MapsData::visualizeMask(mVisualFolder + "points.png", flags)) return false;
#endif

	// chain contour

	vector<vec2i> chain(0);
	if (true) {
		vector<vec2i> queue(1, startPoint);
		flags[startPoint[0]][startPoint[1]] = false;
		while (!queue.empty()) {
			vec2i curPos = queue.back();
			bool hasNext = false;
			for (vec2i offset : offset8) {
				vec2i nextPos = curPos + offset;
				if (nextPos[0] < 0 || nextPos[0] >= imgHeight || nextPos[1] < 0 || nextPos[1] >= imgWidth) continue;
				if (!flags[nextPos[0]][nextPos[1]]) continue;
				flags[nextPos[0]][nextPos[1]] = false;
				queue.push_back(nextPos);
				hasNext = true;
				break;
			}
			if (!hasNext) {
				if (queue.size() > chain.size()) {
					chain = queue;
				}
				queue.pop_back();
			}
		}
	}
#ifdef DEBUG_VISUALIZATION
	cout << "Chain length = " << chain.size() << endl;
	cout << "Area = " << mainComponentSize << endl;
	cout << "Ratio = " << (cml::sqr(double(chain.size())) / mainComponentSize) << endl;
#endif

	// output outline

	outline.clear();
	for (vec2i point : chain) {
		if (sketch[point[0]][point[1]]) outline.push_back(point);
	}

	return true;
}

bool ExtractContour::dilateMask(vector<vector<bool>> &mask) {

	int height = (int)mask.size();
	int width = (int)mask[0].size();

	vector<vector<bool>> result = mask;
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			if (mask[row][col]) {
				if (row > 0) result[row - 1][col] = true;
				if (row + 1 < height) result[row + 1][col] = true;
				if (col > 0) result[row][col - 1] = true;
				if (col + 1 < width) result[row][col + 1] = true;
				if (row > 0 && col > 0) result[row - 1][col - 1] = true;
				if (row > 0 && col + 1 < width) result[row - 1][col + 1] = true;
				if (row + 1 < height && col > 0) result[row + 1][col - 1] = true;
				if (row + 1 < height && col + 1 < width) result[row + 1][col + 1] = true;
			}
		}
	}
	mask.swap(result);

	return true;
}

bool ExtractContour::erodeMask(vector<vector<bool>> &mask) {

	int height = (int)mask.size();
	int width = (int)mask[0].size();

	vector<vector<bool>> result = mask;
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			if (mask[row][col]) {
				bool valid = true;
				if (row > 0 && !mask[row - 1][col]) valid = false;
				if (row + 1 < height && !mask[row + 1][col]) valid = false;
				if (col > 0 && !mask[row][col - 1]) valid = false;
				if (col + 1 < width && !mask[row][col + 1]) valid = false;
				result[row][col] = valid;
			}
		}
	}
	mask.swap(result);

	return true;

	return true;
}

bool ExtractContour::connectContourChains(vector<vec2i> &chain, vector<vector<bool>> &sketch) {

	int imgHeight = (int)sketch.size();
	int imgWidth = (int)sketch[0].size();

	vector<vector<double>> distance(imgHeight, vector<double>(imgWidth, DBL_MAX));
	vector<vector<vec2i>> origin(imgHeight, vector<vec2i>(imgWidth, vec2i(-1,-1)));

	static vector<vec2i> inspectOffsets = { // 8-connect offsets in clockwise order
		vec2i(-1, -1), vec2i(-1, 0), vec2i(-1, 1), vec2i(0, 1),
		vec2i(1, 1), vec2i(1, 0), vec2i(1, -1), vec2i(0, -1) };

	vector<vec2i> newChain(0);
	int chainSize = (int)chain.size();
	for (int id = 0; id < chainSize; id++) {
		vec2i source = chain[id];
		vec2i target = chain[(id + 1) % chainSize];
		distance[source[0]][source[1]] = 0;
		origin[source[0]][source[1]] = source;
		auto cmp = [](pair<double, vec2i> &lhs, pair<double, vec2i> &rhs){return lhs.first > rhs.first; };
		priority_queue < pair<double, vec2i>, vector<pair<double, vec2i>>,
			std::function<bool(pair<double, vec2i> &, pair<double, vec2i> &)> > queue(cmp);
		queue.push(make_pair(0, source));
		set<vec2i> touched;
		touched.insert(source);

		// find shortest path

		while (true) {
			auto top = queue.top();
			double expandDist = top.first;
			vec2i expand = top.second;
			queue.pop();
			if (distance[expand[0]][expand[1]] < expandDist) continue;
			if (expand == target) break;
			for (vec2i offset : inspectOffsets) {
				vec2i next = expand + offset;
				if (next[0] < 0 || next[0] >= imgHeight || next[1] < 0 || next[1] >= imgWidth) continue;
				double d = vec2d(offset).length() * ((sketch[expand[0]][expand[1]] && sketch[next[0]][next[1]]) ? 0.3 : 1.0); // UNDONE: param sketch distance
				d += expandDist;
				if (d < distance[next[0]][next[1]]) {
					distance[next[0]][next[1]] = d;
					origin[next[0]][next[1]] = expand;
					queue.push(make_pair(d, next));
					touched.insert(next);
				}
			}
		}

		// update new chain

		vec2i track = target;
		vector<vec2i> newSeg(0);
		while (true) {
			track = origin[track[0]][track[1]];
			if (sketch[track[0]][track[1]]) newSeg.push_back(track);
			if (track == source) break;
		}
		newChain.insert(newChain.end(), newSeg.rbegin(), newSeg.rend());

		// restore distance & origin

		for (vec2i idx : touched) {
			distance[idx[0]][idx[1]] = DBL_MAX;
			origin[idx[0]][idx[1]] = vec2i(-1, -1);
		}
	}

	chain.swap(newChain);

	return true;
}

bool ExtractContour::sampleContour(vector<vec2i> &contourPixels, vector<vec2d> &contourSamples) {

	const double gap = 1.0; // in pixel unit

	int numPixels = (int)contourPixels.size();
	vector<double> pixelNextDist(numPixels, 0);
	for (int pixelID = 0; pixelID < numPixels-1; pixelID++) {
		pixelNextDist[pixelID] = (vec2d(contourPixels[pixelID]) - vec2d(contourPixels[pixelID + 1])).length();
	}
	pixelNextDist[numPixels - 1] = DBL_MAX;

	contourSamples.clear();
	int start = 0;
	double pixelDistThreshold = 2.0;
	for (int finish = 0; finish < numPixels; finish++) {
		if (pixelNextDist[finish] > pixelDistThreshold || finish-start > 100) {

			int numControlPoints = finish - start + 1;
			if (numControlPoints < 4) continue;
			vector<vec2d> controlPoints(numControlPoints);
			double curveLen = 0;
			for (int controlID = 0; controlID < numControlPoints - 1; controlID++) {
				int pixelID = start + controlID;
				controlPoints[controlID] = contourPixels[pixelID];
				curveLen += pixelNextDist[pixelID];
			}
			controlPoints[numControlPoints - 1] = contourPixels[finish];

			int numSegments = (int)(curveLen / gap);
			vector<vec2d> samples(numSegments + 1);
			for (int pointID = 0; pointID <= numSegments; pointID++) {
				double param = pointID / (double)numSegments;
				vec2d point;
				if (!computeBezierPoint(controlPoints, param, point)) return false;
				samples[pointID] = point;
			}

			contourSamples.insert(contourSamples.end(), samples.begin(), samples.end());

			start = finish;
			if (pixelNextDist[finish] > pixelDistThreshold) start++;
		}
	}

	return true;
}

bool ExtractContour::computeBezierPoint(vector<vec2d> &controlPoints, double parameter, vec2d &curvePoint) {

	int numPoints = (int)controlPoints.size();
	vector<vec2d> interPoints = controlPoints;
	for (int iterID = 0; iterID < numPoints - 1; iterID++) {
		for (int pointID = 0; pointID < numPoints - iterID - 1; pointID++) {
			interPoints[pointID] = interPoints[pointID] * (1.0 - parameter) + interPoints[pointID + 1] * parameter;
		}
	}
	curvePoint = interPoints[0];

	return true;
}

bool ExtractContour::visualizeChain(string fileName, vector<vec2i> &chain, vec2i imgSizes) {

	vector<vector<double>> chainsSketch(imgSizes[0], vector<double>(imgSizes[1], 0.0));
	int numChainPoint = (int)chain.size();
	for (int chainID = 0; chainID < numChainPoint; chainID++) {
		vec2i point = chain[chainID];
		chainsSketch[point[0]][point[1]] = (chainID % 128 + 128) / 255.0;
	}
	if (!MapsData::visualizeSketch(fileName, chainsSketch)) return false;

	return true;
}

bool ExtractContour::visualizeContour(string fileName, vector<vec2d> &contour) {

	int numPoints = (int)contour.size();

	TPointSet points;
	points.positions.resize(numPoints);
	points.normals.resize(numPoints);
	for (int pointID = 0; pointID < numPoints; pointID++) {
		points.positions[pointID] = vec3d(contour[pointID][1], -contour[pointID][0], 0.0);
		points.normals[pointID] = vec3(0.0f, 0.0f, 1.0f);
	}
	points.amount = numPoints;

	if (!MeshIO::savePointSet(fileName, points)) return false;

	return true;
}