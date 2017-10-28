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


#include "OptimizeDepths.h"

#include "Library/CMLHelper.h"
#include "Library/EigenHelper.h"

#include "Mesh/MeshView.h"

//#define DEBUG_OUTPUT

using namespace Monster;

bool OptimizeDepths::optimize(
	vector<vector<vector<bool>>> &masks,
	vector<vector<vector<double>>> &depths,
	vector<vector<vector<vec3d>>> &normals,
	vector<vector<double>> &maskProbs,
	vector<vector<bool>> &outMasks,
	vector<vector<double>> &outDepths,
	vector<vector<vec3d>> &outNormals)
{
	const double lambda = 0.3; // UNDONE: PARAM lambda parameter for energy terms
	const double alpha = 3.0; // UNDONE: param scale factor for original view

	int numViews = (int)masks.size();
	int imgSize = (int)masks[0].size();

	// build valid pixel index

	vector<vec2i> pixelIndex(0);
	vector<vector<int>> pixelMap(imgSize, vector<int>(imgSize, -1));
	pixelIndex.reserve(imgSize*imgSize);
	for (int row = 0; row < imgSize; row++) {
		for (int col = 0; col < imgSize; col++) {
			bool valid = masks[0][row][col];
			if (false) {
				// HACK: fill holes from other views (too much...)
				int validCount = 0;
				for (int viewID = 0; viewID < numViews; viewID++) {
					if (masks[viewID][row][col]) {
						validCount++;
					}
				}
				valid = validCount >= 2;
			}
			if (valid) {
				vec2i index(row, col);
				pixelMap[row][col] = (int)pixelIndex.size();
				pixelIndex.push_back(vec2i(row, col));
			}
		}
	}
	int numPixels = (int)pixelIndex.size();

	// find neighbors for discrete partial derivative

#ifdef DEBUG_OUTPUT
	cout << "finding neighbors" << endl;
#endif

	// (pixel ID, pixel weight) : # views : # valid pixels
	vector<vector<vector<vec2i>>> pixelNeighborsDZDx(numPixels, vector<vector<vec2i>>(numViews, vector<vec2i>(0)));
	vector<vector<vector<vec2i>>> pixelNeighborsDZDy(numPixels, vector<vector<vec2i>>(numViews, vector<vec2i>(0)));

	vector<vec2i> neighborOffsets(0);
	for (int ro = -1; ro <= 1; ro++) {
		for (int co = -1; co <= 1; co++) {
			neighborOffsets.push_back(vec2i(ro, co));
		}
	}
	// NOTE: depth direction is opposite to Z axis, dz should be negated
	vector<int> neighborWeightsDZDx = { 1, 0, -1, 4, 0, -4, 1, 0, -1 };
	vector<int> neighborWeightsDZDy = { -1, -4, -1, 0, 0, 0, 1, 4, 1 };

#pragma omp parallel for
	for (int pixelID = 0; pixelID < numPixels; pixelID++) {
		int row = pixelIndex[pixelID][0];
		int col = pixelIndex[pixelID][1];

		for (int viewID = 0; viewID < numViews; viewID++) {
			vector<vec2i> neighborsX(0);
			vector<vec2i> neighborsY(0);
			vector<double> neighborGapsX(0);
			vector<double> neighborGapsY(0);
			double minGapX = DBL_MAX;
			double minGapY = DBL_MAX;
			for (int offsetID = 0; offsetID < (int)neighborOffsets.size(); offsetID++) {
				int nbRow = row + neighborOffsets[offsetID][0];
				int nbCol = col + neighborOffsets[offsetID][1];
				if (nbRow < 0 || nbRow >= imgSize) continue;
				if (nbCol < 0 || nbCol >= imgSize) continue;
				if (!masks[viewID][nbRow][nbCol]) continue;

				int neighborID = pixelMap[nbRow][nbCol];
				if (neighborID < 0) continue;
				double gap = fabs(depths[viewID][nbRow][nbCol] - depths[viewID][row][col]);
				if (neighborWeightsDZDx[offsetID]) {
					neighborsX.push_back(vec2i(neighborID, neighborWeightsDZDx[offsetID]));
					neighborGapsX.push_back(gap);
					minGapX = min(minGapX, gap);
				}
				if (neighborWeightsDZDy[offsetID]) {
					neighborsY.push_back(vec2i(neighborID, neighborWeightsDZDy[offsetID]));
					neighborGapsY.push_back(gap);
					minGapY = min(minGapY, gap);
				}
			}
			double maxGapX = minGapX * 10.0; // UNDONE: param depth discontinuity threshold
			double maxGapY = minGapY * 10.0;
			int neighborWeightsX = 0;
			int neighborWeightsY = 0;
			for (int id = 0; id < (int)neighborsX.size(); id++) {
				if (neighborGapsX[id] < maxGapX) {
					pixelNeighborsDZDx[pixelID][viewID].push_back(neighborsX[id]);
					neighborWeightsX += neighborsX[id][1];
				}
			}
			if (neighborWeightsX) {
				pixelNeighborsDZDx[pixelID][viewID].push_back(vec2i(pixelID, -neighborWeightsX));
			}
			for (int id = 0; id < (int)neighborsY.size(); id++) {
				if (neighborGapsY[id] < maxGapY) {
					pixelNeighborsDZDy[pixelID][viewID].push_back(neighborsY[id]);
					neighborWeightsY += neighborsY[id][1];
				}
			}
			if (neighborWeightsY) {
				pixelNeighborsDZDy[pixelID][viewID].push_back(vec2i(pixelID, -neighborWeightsY));
			}
		}
	}

	if (true) {
		int countX = 0;
		int countY = 0;
		int countTX = 0;
		int countTY = 0;
		for (int pixelID = 0; pixelID < numPixels; pixelID++) {
			for (int viewID = 0; viewID < numViews; viewID++) {
				int x = (int)pixelNeighborsDZDx[pixelID][viewID].size();
				int y = (int)pixelNeighborsDZDy[pixelID][viewID].size();
				if (x) {
					countX += x;
					countTX++;
				}
				if (y) {
					countY += y;
					countTY++;
				}
			}
		}
#ifdef DEBUG_OUTPUT
		cout << "Avg neighbors: " << (countX / (double)countTX) << ", " << (countY / (double)countTY) << endl;
#endif
	}

	// build sparse linear system

#ifdef DEBUG_OUTPUT
	cout << "building sparse linear system" << endl;
#endif

	double kappa = MeshView::VIEW_RADIUS * 2 / imgSize; // pixel gap

	vector<Eigen::Triplet<double, int>> triplets(0);
	vector<double> values(0);

	int eqnID = 0;
	for (int pixelID = 0; pixelID < numPixels; pixelID++) {
		int row = pixelIndex[pixelID][0];
		int col = pixelIndex[pixelID][1];

		double probWeight = maskProbs[row][col];

		for (int viewID = 0; viewID < numViews; viewID++) {
			if (!masks[viewID][row][col]) continue;
			double Zuv = depths[viewID][row][col];
			vec3d Nuv = normals[viewID][row][col];
			double eqnWeight = viewID == 0 ? alpha : 1.0;
			eqnWeight *= probWeight;

			// lambda * Z = lambda * Zuv
			if (true) {
				double lhs = lambda * eqnWeight;
				double rhs = lambda * Zuv * eqnWeight;
				triplets.push_back(Eigen::Triplet<double, int>(eqnID, pixelID, lhs));
				values.push_back(rhs);
				eqnID++;
			}

			// (1-lambda) * (dZdx * Nuv.z) = (1-lambda) * (-kappa * Nuv.x)
			if (true) {
				int totalWeight = 0;
				for (vec2i neighbor : pixelNeighborsDZDx[pixelID][viewID]) {
					if(neighbor[0] != pixelID) totalWeight += abs(neighbor[1]);
				}
				if (totalWeight) {
					for (vec2i neighbor : pixelNeighborsDZDx[pixelID][viewID]) {
						int neighborID = neighbor[0];
						double lhs = (1 - lambda) * Nuv[2] * (neighbor[1] / (double)totalWeight) * eqnWeight;
						triplets.push_back(Eigen::Triplet<double, int>(eqnID, neighborID, lhs));
					}
					double rhs = (1 - lambda) * (-kappa * Nuv[0]) * eqnWeight;
					values.push_back(rhs);
					eqnID++;
				}
			}

			// (1-lambda) * (dZdy * Nuv.z) = (1-lambda) * (-kappa * Nuv.y)
			if (true) {
				int totalWeight = 0;
				for (vec2i neighbor : pixelNeighborsDZDy[pixelID][viewID]) {
					totalWeight += abs(neighbor[1]);
				}
				if (totalWeight) {
					for (vec2i neighbor : pixelNeighborsDZDy[pixelID][viewID]) {
						int neighborID = neighbor[0];
						double lhs = (1 - lambda) * Nuv[2] * (neighbor[1] / (double)totalWeight) * eqnWeight;
						triplets.push_back(Eigen::Triplet<double, int>(eqnID, neighborID, lhs));
					}
					double rhs = (1 - lambda) * (-kappa * Nuv[1]) * eqnWeight;
					values.push_back(rhs);
					eqnID++;
				}
			}
		}
	}
	int numEquations = eqnID;

#ifdef DEBUG_OUTPUT
	cout << "# eqns = " << numEquations << endl;
	cout << "# pixels = " << numPixels << endl;
#endif

	Eigen::SparseMatrix<double> matA(numEquations, numPixels);
	matA.setFromTriplets(triplets.begin(), triplets.end());
	Eigen::VectorXd matB = Eigen::Map<Eigen::VectorXd>(values.data(), values.size());

	// get initial guess

	Eigen::VectorXd matG(numPixels);
	for (int pixelID = 0; pixelID < numPixels; pixelID++) {
		int row = pixelIndex[pixelID][0];
		int col = pixelIndex[pixelID][1];
		matG[pixelID] = depths[0][row][col];
	}

	// solve sparse linear system

#ifdef DEBUG_OUTPUT
	cout << "solving sparse linear system" << endl;
#endif

	Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<double>> solver;
	//cout << "\t initializing solver" << endl;
	solver.compute(matA);
	if (solver.info() != Eigen::Success) return error("compute matA for solver");
	//cout << "\t solving" << endl;
	Eigen::VectorXd matX = solver.solveWithGuess(matB, matG);
	//cout << "\t # iters = " << solver.iterations() << endl;
	//cout << "\t error = " << solver.error() << endl;
	if (solver.info() != Eigen::Success) error("solver error - " + to_string(solver.info()));
	//cout << "\t done." << endl;

	//cout << "Residual error = " << (matA*matX - matB).norm() / matB.norm() << endl;
	//cout << "Before: error = " << (matA*matG - matB).norm() << endl;
	//cout << "After: error = " << (matA*matX - matB).norm() << endl;


	// output results

	outMasks.assign(imgSize, vector<bool>(imgSize, false));
	outDepths.assign(imgSize, vector<double>(imgSize, 1.0));
	outNormals.assign(imgSize, vector<vec3d>(imgSize, vec3d(1.0, 1.0, 1.0)));
	for (int pixelID = 0; pixelID < numPixels; pixelID++) {
		int row = pixelIndex[pixelID][0];
		int col = pixelIndex[pixelID][1];
		outMasks[row][col] = true;
		outDepths[row][col] = matX[pixelID];
		if (masks[0][row][col]) {
			outNormals[row][col] = normals[0][row][col];
		} else {
			vec3d accumNormal(0.0, 0.0, 0.0);
			for (int viewID = 0; viewID < numViews; viewID++) {
				if (masks[viewID][row][col]) {
					accumNormal += normals[viewID][row][col];
				}
			}
			if(accumNormal.length_squared()) accumNormal.normalize();
			else accumNormal = vec3d(0.0, 0.0, 1.0);
			outNormals[row][col] = accumNormal;
		}
	}

	return true;
}

bool OptimizeDepths::error(string s) {

	cout << "Error: " << s << endl;
	system("pause");

	return false;
}