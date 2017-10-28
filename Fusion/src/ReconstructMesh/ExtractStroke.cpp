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


#include "ExtractStroke.h"

#include <set>

#include "MapsData.h"
#include "Image/ImageIO.h"
#include "Mesh/MeshIO.h"
#include "Format/PlyExporter.h"

using namespace Monster;

#define DEBUG_VISUALIZATION

string ExtractStroke::mVisualFolder = "";

bool ExtractStroke::extract(vector<vector<double>> &sketch, vector<vector<vec2d>> &strokes, string visualFolder) {

	int imgHeight = (int)sketch.size();
	int imgWidth = (int)sketch[0].size();

	mVisualFolder = visualFolder;

	vector<vector<bool>> skeleton;
	if (!extractSkeleton(sketch, skeleton)) return false;

	vector<vector<vec2i>> components;
	if (!extractComponents(skeleton, components)) return false;

	vector<vector<vec2i>> rawStrokes;
	for (auto &component : components) {
		vector<vector<vec2i>> chains;
		if (!chainStroke(component, chains)) return false;
		rawStrokes.insert(rawStrokes.end(), chains.begin(), chains.end());
	}

#ifdef DEBUG_VISUALIZATION
	// visualize strokes
	if (true) {
		vector<unsigned char> strokeImage(imgHeight*imgWidth * 3, 0);
		for (int strokeID = 0; strokeID < (int)rawStrokes.size(); strokeID++) {
			vec3i color = MeshIO::colorMapping(strokeID);
			for (vec2i pos : rawStrokes[strokeID]) {
				int pixelID = pos[0] * imgWidth + pos[1];
				for (int k = 0; k < 3; k++) {
					strokeImage[pixelID * 3 + k] = color[k];
				}
			}
			if (true) {
				vec2i pos = rawStrokes[strokeID][0];
				int pixelID = pos[0] * imgWidth + pos[1];
				for (int k = 0; k < 3; k++) {
					strokeImage[pixelID * 3 + k] = 255;
				}
			}
		}
		if (!ImageIO::writeImage(mVisualFolder + "strokes.png", strokeImage, imgWidth, imgHeight, 3)) return false;
	}
#endif

	if (!sampleStroke(rawStrokes, strokes)) return false;

#ifdef DEBUG_VISUALIZATION
	// visualize sampled strokes
	if (true) {
		vector<vec3> pointPairs;
		for (auto &chain : strokes) {
			int numPoints = (int)chain.size();
			for (int pointID = 0; pointID < numPoints-1; pointID++) {
				vec2 start = chain[pointID];
				vec2 finish = chain[pointID+1];
				pointPairs.push_back(vec3(start[1], -start[0], 0.0f));
				pointPairs.push_back(vec3(finish[1], -finish[0], 0.0f));
			}
		}
		PlyExporter pe;
		if (!pe.addLine(&pointPairs)) return false;
		if (!pe.output(mVisualFolder + "smoothed.ply")) return false;
	}
#endif

#ifdef DEBUG_VISUALIZATION
	//system("pause");
#endif

	return true;
}

bool ExtractStroke::extractSkeleton(vector<vector<double>> &sketch, vector<vector<bool>> &skeleton) {

	int imgHeight = (int)sketch.size();
	int imgWidth = (int)sketch[0].size();

	// find threshold

	double sketchThreshold = 0.5; // UNDONE: param sketch stroke threshold
	if (true) {
		vector<double> values(0);
		values.reserve(imgHeight*imgWidth);
		for (int row = 0; row < imgHeight; row++) {
			for (int col = 0; col < imgWidth; col++) {
				double value = sketch[row][col];
				if (value > 0.1) values.push_back(value);
			}
		}
		if (!values.empty()) {
			int n = (int)values.size();
			int m = (int)(n*0.8); // UNDONE: sketch stroke thresholding percentile
			nth_element(values.begin(), values.begin() + m, values.end());
			sketchThreshold = values[m];
		}
		cout << "Sketch stroke threshold = " << sketchThreshold << endl;
	}

	// mark bool flag

	vector<vector<bool>> boolSketch(imgHeight, vector<bool>(imgWidth, false));
	for (int row = 0; row < imgHeight; row++) {
		for (int col = 0; col < imgWidth; col++) {
			boolSketch[row][col] = (sketch[row][col] >= sketchThreshold); // UNDONE: param stroke sketch threshold
		}
	}

#ifdef DEBUG_VISUALIZATION
	// visualize stroke

	if (true) {
		vector<vector<double>> strokeSketch(imgHeight, vector<double>(imgWidth, 0.0));
		for (int row = 0; row < imgHeight; row++) {
			for (int col = 0; col < imgWidth; col++) {
				if (boolSketch[row][col]) strokeSketch[row][col] = 1.0;
			}
		}
		if (!MapsData::visualizeSketch(mVisualFolder + "sketch.png", strokeSketch)) return false;
	}
#endif

	// compute number of neighbors

	static vector<vec2i> inspectOffsets = { // 8-connect offsets in clockwise order
		vec2i(-1, -1), vec2i(-1, 0), vec2i(-1, 1), vec2i(0, 1),
		vec2i(1, 1), vec2i(1, 0), vec2i(1, -1), vec2i(0, -1) };
	static int numOffsets = (int)inspectOffsets.size();

	vector<vector<int>> neighborCounts(imgHeight, vector<int>(imgWidth, 0));
	for (int row = 0; row < imgHeight; row++) {
		for (int col = 0; col < imgWidth; col++) {
			if (!boolSketch[row][col]) continue;
			for (vec2i offset : inspectOffsets) {
				int nbr = row + offset[0];
				int nbc = col + offset[1];
				if (nbr < 0 || nbr >= imgHeight || nbc < 0 || nbc >= imgWidth) continue;
				if (boolSketch[nbr][nbc]) {
					neighborCounts[row][col]++;
					if (nbr == row || nbc == col) neighborCounts[row][col]++;
				}
			}
		}
	}

	// shrink sketch

	//vector<vec2i> blockOffsets = { vec2i(0, 0), vec2i(1, 0), vec2i(0, 1), vec2i(1, 1) };
	//int numBlockOffsets = (int)blockOffsets.size();
	//for (int row = 0; row < imgHeight-1; row++) {
	//	for (int col = 0; col < imgWidth-1; col++) {
	//		bool canShrink = true;
	//		for (vec2i offset : blockOffsets) {
	//			if (!boolSketch[row + offset[0]][col + offset[1]]) {
	//				canShrink = false;
	//				break;
	//			}
	//		}
	//		if (!canShrink) continue;
	//		vector<vec2i> blockCoords(numBlockOffsets);
	//		vector<int> blockNeighborCounts(numBlockOffsets);
	//		for (int blockID = 0; blockID < numBlockOffsets; blockID++) {
	//			vec2i offset = blockOffsets[blockID];
	//			blockCoords[blockID] = vec2i(row + offset[0], col + offset[1]);
	//			blockNeighborCounts[blockID] = neighborCounts[row + offset[0]][col + offset[1]];
	//		}
	//		for (int i = 0; i < numBlockOffsets - 1; i++) {
	//			for (int j = i + 1; j < numBlockOffsets; j++) {
	//				if (blockNeighborCounts[i] < blockNeighborCounts[j]) {
	//					swap(blockNeighborCounts[i], blockNeighborCounts[j]);
	//					swap(blockCoords[i], blockCoords[j]);
	//				}
	//			}
	//		}
	//		boolSketch[blockCoords[2][0]][blockCoords[2][1]] = false;
	//		boolSketch[blockCoords[3][0]][blockCoords[3][1]] = false;
	//	}
	//}

#ifdef DEBUG_VISUALIZATION
	// visualize skeleton

	if (true) {
		vector<vector<double>> strokeSketch(imgHeight, vector<double>(imgWidth, 0.0));
		for (int row = 0; row < imgHeight; row++) {
			for (int col = 0; col < imgWidth; col++) {
				if (boolSketch[row][col]) strokeSketch[row][col] = 1.0;
			}
		}
		if (!MapsData::visualizeSketch(mVisualFolder + "skeleton.png", strokeSketch)) return false;
	}
#endif

	skeleton.swap(boolSketch);

	return true;
}

bool ExtractStroke::extractComponents(vector<vector<bool>> &sketch, vector<vector<vec2i>> &components) {

	int imgHeight = (int)sketch.size();
	int imgWidth = (int)sketch[0].size();

	static vector<vec2i> inspectOffsets = { // 8-connect offsets in clockwise order
		vec2i(-1, -1), vec2i(-1, 0), vec2i(-1, 1), vec2i(0, 1),
		vec2i(1, 1), vec2i(1, 0), vec2i(1, -1), vec2i(0, -1) };

	// find connected components (BFS)

	vector<vector<int>> componentMap(imgHeight, vector<int>(imgWidth, -1));
	components.clear();
	for (int row = 0; row < imgHeight; row++) {
		for (int col = 0; col < imgWidth; col++) {
			if (!sketch[row][col] || componentMap[row][col] >= 0) continue;
			int compID = (int)components.size();
			componentMap[row][col] = compID;
			vector<vec2i> queue(1, vec2i(row, col));
			int head = 0;
			while (head < (int)queue.size()) {
				vec2i curPos = queue[head];
				for (vec2i offset : inspectOffsets) {
					vec2i nbPos = curPos + offset;
					if (nbPos[0] < 0 || nbPos[0] >= imgHeight || nbPos[1] < 0 || nbPos[1] >= imgWidth) continue;
					if (!sketch[nbPos[0]][nbPos[1]]) continue;
					if (componentMap[nbPos[0]][nbPos[1]] >= 0) continue;
					queue.push_back(nbPos);
					componentMap[nbPos[0]][nbPos[1]] = compID;
				}
				head++;
			}
			components.push_back(queue);
		}
	}
	int numComponents = (int)components.size();
	
	// compute component center

	vector<vec2i> compPosSums(numComponents);
	vector<int> compSizes(numComponents);
	for (int compID = 0; compID < numComponents; compID++) {
		vec2i posSum(0, 0);
		for (vec2i pos : components[compID]) {
			posSum += pos;
		}
		compPosSums[compID] = posSum;
		compSizes[compID] = (int)components[compID].size();
	}

	// chain components

	for (int row = 1; row < imgHeight-1; row++) {
		for (int col = 1; col < imgWidth-1; col++) {
			if (componentMap[row][col] >= 0) continue;
			set<int> nbComps;
			for (vec2i offset : inspectOffsets) {
				int compID = componentMap[row + offset[0]][col + offset[1]];
				if (compID >= 0) nbComps.insert(compID);
			}
			if ((int)nbComps.size() != 2) continue;
			vector<int> nbList(nbComps.begin(), nbComps.end());
			int compID1 = nbList[0];
			int compID2 = nbList[1];
			vec2d p0 = vec2i(row, col);
			vec2d p1 = vec2d(compPosSums[nbList[0]]) / compSizes[nbList[0]];
			vec2d p2 = vec2d(compPosSums[nbList[1]]) / compSizes[nbList[1]];
			vec2d v1 = cml::normalize(p0 - p1);
			vec2d v2 = cml::normalize(p2 - p0);
			if (cml::dot(v1, v2) > 0.5) { // UNDONE: param component chaining angle threshold
				// merge components
				
				auto &comp1 = components[compID1];
				auto &comp2 = components[compID2];
				componentMap[row][col] = compID1;
				for (vec2i pos : comp2) {
					componentMap[pos[0]][pos[1]] = compID1;
				}
				comp1.push_back(vec2i(row, col));
				comp1.insert(comp1.end(), comp2.begin(), comp2.end());
				compPosSums[compID1] += compPosSums[compID2] + vec2i(row, col);
				compSizes[compID1] += compSizes[compID2] + 1;

				numComponents--;
				if (compID2 != numComponents) {
					comp2.swap(components[numComponents]);
					compPosSums[compID2] = compPosSums[numComponents];
					compSizes[compID2] = compSizes[numComponents];
					for (vec2i pos : components[compID2]) {
						componentMap[pos[0]][pos[1]] = compID2;
					}
				}
			}

		}
	}
	components.resize(numComponents);

	// prune small components

	for (int compID = 0; compID < numComponents; compID++) {
		if ((int)components[compID].size() < 5) { // UNDONE: param minimum component size
			for (vec2i pos : components[compID]) {
				componentMap[pos[0]][pos[1]] = -1;
			}
			numComponents--;
			if (compID != numComponents) {
				components[compID].swap(components[numComponents]);
				for (vec2i pos : components[compID]) {
					componentMap[pos[0]][pos[1]] = compID;
				}
			}
			compID--;
		}
	}
	components.resize(numComponents);
	
#ifdef DEBUG_VISUALIZATION
	// visualize components

	if (true) {
		vector<unsigned char> componentImage(imgHeight*imgWidth * 3, 0);
		vector<vec3i> colorMap(numComponents);
		for (int compID = 0; compID < numComponents; compID++) {
			colorMap[compID] = MeshIO::colorMapping(compID);
		}
		for (int row = 0; row < imgHeight; row++) {
			for (int col = 0; col < imgWidth; col++) {
				int compID = componentMap[row][col];
				if (compID < 0) continue;
				int pixelID = row*imgWidth + col;
				vec3i color = colorMap[compID];
				for (int k = 0; k < 3; k++) {
					componentImage[pixelID * 3 + k] = color[k];
				}
			}
		}
		if (!ImageIO::writeImage(mVisualFolder + "components.png", componentImage, imgWidth, imgHeight, 3)) return false;
	}
#endif

	return true;
}

bool ExtractStroke::chainStroke(vector<vec2i> &component, vector<vector<vec2i>> &chains) {

	int numPoints = (int)component.size();

	// build graph

	vector<set<int>> graph(numPoints);
	if (true) {

		static vector<vec2i> inspectOffsets = { // 8-connect offsets in clockwise order
			vec2i(-1, -1), vec2i(-1, 0), vec2i(-1, 1), vec2i(0, 1),
			vec2i(1, 1), vec2i(1, 0), vec2i(1, -1), vec2i(0, -1) };

		vec2i minUV(INT_MAX, INT_MAX);
		vec2i maxUV(-INT_MAX, -INT_MAX);
		for (vec2i pos : component) {
			minUV.minimize(pos);
			maxUV.maximize(pos);
		}
		int blockHeight = maxUV[0] - minUV[0] + 3;
		int blockWidth = maxUV[1] - minUV[1] + 3;
		vector<vector<int>> block(blockHeight, vector<int>(blockWidth, -1));
		for (int pointID = 0; pointID < numPoints; pointID++) {
			vec2i pos = component[pointID];
			vec2i uv = pos - minUV + vec2i(1, 1);
			block[uv[0]][uv[1]] = pointID;
		}

		for (int pointID = 0; pointID < numPoints; pointID++) {
			vec2i pos = component[pointID];
			vec2i uv = pos - minUV + vec2i(1, 1);
			for (vec2i offset : inspectOffsets) {
				vec2i nbUV = uv + offset;
				int nbID = block[nbUV[0]][nbUV[1]];
				if (nbID >= 0) graph[pointID].insert(nbID);
			}
		}
	}
	
	// determine chaining order

	vec2d componentCenter(0.0, 0.0);
	for (vec2i point : component) {
		componentCenter += vec2d(point);
	}
	componentCenter *= (1.0 / numPoints);

	//cout << minDists.maxCoeff() << " < " << numPoints << endl;
	vector<int> pointOrder(numPoints);
	vector<double> pointDist(numPoints);
	for (int pointID = 0; pointID < numPoints; pointID++) {
		pointOrder[pointID] = pointID;
		pointDist[pointID] = (vec2d(component[pointID]) - componentCenter).length();
	}
	sort(pointOrder.begin(), pointOrder.end(),
		 [&pointDist](int lhs, int rhs){return pointDist[lhs] > pointDist[rhs]; });

	// start chaining

	chains.clear();
	vector<bool> visitedFlags(numPoints, false);
	for (int orderID = 0; orderID < numPoints; orderID++) {
		int pointID = pointOrder[orderID];
		if (visitedFlags[pointID]) continue;
		visitedFlags[pointID] = true;
		vector<vec2i> chain(1, component[pointID]);
		int curID = pointID;
		while (true) {
			int nextID = -1;
			int nextNbs = -1;
			for (int nbID : graph[curID]) {
				if (visitedFlags[nbID]) continue;
				int nbs = (int)graph[nbID].size();
				if (nbs > nextNbs) {
					nextNbs = nbs;
					nextID = nbID;
				}
			}
			if (nextID < 0) break;
			chain.push_back(component[nextID]);
			//visitedFlags[nextID] = true;
			for (int nbID : graph[curID]) visitedFlags[nbID] = true;
			curID = nextID;
		}
		if ((int)chain.size() >= 5) { // UNDONE: minimum chain length
			chains.push_back(chain);
		}
	}

	return true;
}


bool ExtractStroke::sampleStroke(vector<vector<vec2i>> &rawStrokes, vector<vector<vec2d>> &sampledStrokes) {

	int numStrokes = (int)rawStrokes.size();
	sampledStrokes.resize(numStrokes);

	double gap = 0.01;
	for (int strokeID = 0; strokeID < numStrokes; strokeID++) {
		vector<vec2i> &chain = rawStrokes[strokeID];
		int numPixels = (int)chain.size();
		vector<vec2d> controlPoints(numPixels);
		controlPoints[0] = chain[0];
		double curveLen = 0;
		for (int pixelID = 1; pixelID < numPixels; pixelID++) {
			controlPoints[pixelID] = chain[pixelID];
			curveLen += (controlPoints[pixelID] - controlPoints[pixelID - 1]).length();
		}
		int numSegments = (int)(curveLen / gap);
		vector<vec2d> &stroke = sampledStrokes[strokeID];
		stroke.resize(numSegments + 1);
		for (int pointID = 0; pointID <= numSegments; pointID++) {
			double param = pointID / (double)numSegments;
			vec2d point;
			if (!computeBezierPoint(controlPoints, param, point)) return false;
			stroke[pointID] = point;
		}
	}

	return true;
}

bool ExtractStroke::computeBezierPoint(vector<vec2d> &controlPoints, double parameter, vec2d &curvePoint) {

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