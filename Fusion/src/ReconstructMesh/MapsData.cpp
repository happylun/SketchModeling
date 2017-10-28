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


#include "MapsData.h"

#include <unordered_set>

#include "ProjectViews.h"
#include "OptimizeDepths.h"

#include "Mesh/MeshView.h"
#include "Mesh/MeshIO.h"
#include "Mesh/MeshCompute.h"

#include "Image/ImageIO.h"

#include "Match/MatchRigidICP.h"

#include "Sample/SampleSimplePoissonDisk.h"

#include "Utility/FileUtil.h"

using namespace Monster;

MapsData::MapsData() :
	mSketches(0), mMasks(0), mMaskProbs(0), mDepths(0), mNormals(0),
	mSketchSize(-1), mMapSize(-1)
{

}

MapsData::~MapsData() {

}

bool MapsData::loadSketch(string &sketchViews, string &sketchFolder) {

	if (!sketchFolder.empty()) {
		vector<string> sketchNames;
		int numSketches = sketchViews.length();
		string styleID = "1";
		if (!FileUtil::existsfile(sketchFolder + "sketch-S-1.png")) styleID = "0";
		for (int viewID = 0; viewID < numSketches; viewID++) {
			string viewSketchName = sketchFolder + "sketch-" + sketchViews[viewID] + "-" + styleID + ".png";
			if (FileUtil::existsfile(viewSketchName)) sketchNames.push_back(viewSketchName);
		}

		mSketches.resize(numSketches);
		for (int sketchID = 0; sketchID < numSketches; sketchID++) {
			if (!parseSketch(sketchNames[sketchID], mSketches[sketchID])) return false;
		}
		if (numSketches) {
			if (mSketchSize >= 0) {
				if (mSketchSize != (int)mSketches[0].size()) return error("inconsistent sketch size");
			} else {
				mSketchSize = (int)mSketches[0].size();
			}
		}
	}

	return true;
}

bool MapsData::loadMap(string &mapFolder, int numMaps, string prefix) {

	vector<string> imageNames;
	for (int viewID = 0; viewID < numMaps; viewID++) {
		string suffix = prefix + "-dn14--" + to_string(viewID) + ".png";
		string name = mapFolder + suffix;
		imageNames.push_back(name);
	}

	int numViews = (int)imageNames.size();

	mMasks.resize(numViews);
	mDepths.resize(numViews);
	mNormals.resize(numViews);

	cout << "Loading data";
	for (int viewID = 0; viewID < numViews; viewID++) {

		//cout << "==== Loading view " << viewID << " ====" << endl;
		cout << ".";
		if (!parseDepthNormal(imageNames[viewID], mMasks[viewID], mDepths[viewID], mNormals[viewID])) return false;
		//if (!cleanMask(mMasks[viewID])) return false;
	}
	cout << endl;

	if (numViews) {
		if (mMapSize >= 0) {
			if (mMapSize != (int)mMasks[0].size()) return error("inconsistent map size");
		} else {
			mMapSize = (int)mMasks[0].size();
		}
	}

	return true;
}

bool MapsData::loadMask(string &dn14Folder, int numMaps) {

	vector<string> imageNames;
	for (int viewID = 0; viewID < numMaps; viewID++) {
		string suffix = "mask-dn14--" + to_string(viewID) + ".png";
		string name = dn14Folder + suffix;
		imageNames.push_back(name);
	}

	int numViews = (int)imageNames.size();

	mMaskProbs.resize(numViews);

	cout << "Loading data";
	for (int viewID = 0; viewID < numViews; viewID++) {

		//cout << "==== Loading view " << viewID << " ====" << endl;
		cout << ".";
		if (!parseMask(imageNames[viewID], mMaskProbs[viewID])) return false;
	}
	cout << endl;

	if (numViews) {
		if (mMapSize >= 0) {
			if (mMapSize != (int)mMaskProbs[0].size()) return error("inconsistent map size");
		} else {
			mMapSize = (int)mMaskProbs[0].size();
		}
	}

	return true;
}

bool MapsData::parseSketch(string &imageName, vector<vector<double>> &sketch) {

	vector<unsigned char> buffer;
	int width, height, channel;
	if (!ImageIO::readImage(imageName, buffer, width, height, channel)) return false;

	sketch.assign(height, vector<double>(width, 0.0));
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			double intensity = 0;
			for (int ch = 0; ch < channel; ch++) {
				unsigned char color = buffer[(row*width + col)*channel + ch];
				intensity += 1.0 - color / (double)(UCHAR_MAX);
			}
			intensity /= channel;
			if (intensity > 0.1) { // UNDONE: sketch thresholding parameter
				sketch[row][col] = intensity;
			}
		}
	}

	return true;
}

bool MapsData::parseDepthNormal(
	string &imageName,
	vector<vector<bool>> &mask,
	vector<vector<double>> &depth,
	vector<vector<vec3d>> &normal)
{
	vector<unsigned short> buffer;
	int width, height, channel;
	if (!ImageIO::readImage(imageName, buffer, width, height, channel)) return false;

	mask.assign(height, vector<bool>(width));
	depth.assign(height, vector<double>(width));
	normal.assign(height, vector<vec3d>(width));

	for (int pixelID = 0; pixelID < height*width; pixelID++) {
		int h = pixelID / width;
		int w = pixelID % width;

		bool mk = true;
		double d = 1.0;
		vec3d n(1.0, 1.0, 1.0);
		if (channel == 1) {
			unsigned short c = buffer[pixelID];
			d = c / (double)32768 - 1.0;
			n = vec3d(1.0, 1.0, 1.0);
		} else if (channel == 4) {
			unsigned short r = buffer[pixelID * 4];
			unsigned short g = buffer[pixelID * 4 + 1];
			unsigned short b = buffer[pixelID * 4 + 2];
			unsigned short a = buffer[pixelID * 4 + 3];

			double x = r / (double)32768 - 1.0;
			double y = g / (double)32768 - 1.0;
			double z = b / (double)32768 - 1.0;
			d = a / (double)32768 - 1.0;
			n = vec3d(x, y, z);

			double nl = n.length();
			if (nl == 0 || nl > 1.5) {  // UNDONE: param normal pruning threshold
				mk = false;
			} else {
				n /= nl;
			}
		} else {
			return error("unsupported number of channels");
		}

		if (d >= 0.9) mk = false; // UNDONE: param depth pruning threshold
		if (!mk) {
			d = 1.0;
			n = vec3d(1.0, 1.0, 1.0);
		}

		mask[h][w] = mk; // T/F
		depth[h][w] = d; // [-1,1]
		normal[h][w] = n; // [-1,1]^3
	}

	if (channel == 1) {
		if (!computeNormalFromDepth(mask, depth, normal)) return false;
		//if (!visualizeNormal("normal.png", normal)) return false;
		//system("pause");
	}

	return true;
}

bool MapsData::parseMask(string &imageName, vector<vector<double>> &mask) {

	vector<unsigned short> buffer;
	int width, height, channel;
	if (!ImageIO::readImage(imageName, buffer, width, height, channel)) return false;

	mask.assign(height, vector<double>(width));

	for (int pixelID = 0; pixelID < height*width; pixelID++) {
		int h = pixelID / width;
		int w = pixelID % width;
		unsigned short v = buffer[pixelID];

		double p = v / (double)65536;

		mask[h][w] = p; // [0,1]
	}

	return true;
}

bool MapsData::cleanMask(vector<vector<bool>> &mask) {

	int height = (int)mask.size();
	int width = (int)mask[0].size();
	vector<vector<bool>> newMask(height, vector<bool>(width, true));

	// find first empty pixel

	vector<vec2i> queue(0);
	int count = 0;
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			if (!mask[row][col]) {
				count++;
				if (queue.empty()) queue.push_back(vec2i(row, col));
			}
		}
	}
	if (queue.empty()) return true;

	// BFS

	int head = 0;
	newMask[queue[0][0]][queue[0][1]] = false;
	while (head < (int)queue.size()) {
		vec2i curPos = queue[head];
		if (curPos[0] < height - 1 && newMask[curPos[0] + 1][curPos[1]] && !mask[curPos[0] + 1][curPos[1]]) {
			queue.push_back(vec2i(curPos[0] + 1, curPos[1]));
			newMask[curPos[0] + 1][curPos[1]] = false;
		}
		if (curPos[1] < width - 1 && newMask[curPos[0]][curPos[1] + 1] && !mask[curPos[0]][curPos[1] + 1]) {
			queue.push_back(vec2i(curPos[0], curPos[1] + 1));
			newMask[curPos[0]][curPos[1] + 1] = false;
		}
		if (curPos[0] > 0 && newMask[curPos[0] - 1][curPos[1]] && !mask[curPos[0] - 1][curPos[1]]) {
			queue.push_back(vec2i(curPos[0] - 1, curPos[1]));
			newMask[curPos[0] - 1][curPos[1]] = false;
		}
		if (curPos[1] > 0 && newMask[curPos[0]][curPos[1] - 1] && !mask[curPos[0]][curPos[1] - 1]) {
			queue.push_back(vec2i(curPos[0], curPos[1] - 1));
			newMask[curPos[0]][curPos[1] - 1] = false;
		}

		head++;
	}

	//cout << "Pruned " << (count - (int)queue.size()) << " / " << count << " holes" << endl;

	mask.swap(newMask);

	return true;
}

bool MapsData::computeNormalFromDepth(
	vector<vector<bool>> &mask,
	vector<vector<double>> &depth,
	vector<vector<vec3d>> &normal)
{
	int height = (int)mask.size();
	int width = (int)mask[0].size();

	Eigen::Matrix4d projMat, imgMat;
	if (!MeshView::buildProjMatrix(projMat)) return false;
	if (!MeshView::buildImageMatrix(imgMat, width, height)) return false;
	Eigen::Matrix4d forwardMat = imgMat * projMat;
	Eigen::Matrix4d backwardMat = forwardMat.inverse();

	Eigen::MatrixXi index = Eigen::MatrixXi::Ones(height, width) * -1;

	TPointSet points;
	points.positions.clear();
	points.positions.reserve(height*width);
	for (int rowID = 0; rowID < height; rowID++) {
		for (int colID = 0; colID < width; colID++) {
			if (!mask[rowID][colID]) continue;
			Eigen::Vector4d point((double)colID, (double)rowID, depth[rowID][colID], 1.0);
			Eigen::Vector4d reprojPoint = backwardMat * point;

			index(rowID, colID) = (int)points.positions.size();
			points.positions.push_back(vec3d(reprojPoint[0], reprojPoint[1], reprojPoint[2]));
		}
	}
	points.amount = (int)points.positions.size();
	if (!MeshCompute::recomputeNormals(points.positions, points.normals, 10, 0.1)) return false;

	normal.resize(height);
	for (auto &row : normal) row.resize(width, vec3d(1.0, 1.0, 1.0));
	for (int rowID = 0; rowID < height; rowID++) {
		for (int colID = 0; colID < width; colID++) {
			int idx = index(rowID, colID);
			if (idx < 0) continue;
			normal[rowID][colID] = points.normals[idx];
		}
	}

	return true;
}

bool MapsData::visualizePoints(string fileName, Eigen::Matrix3Xd &positions, Eigen::Matrix3Xd &normals) {

	TPointSet ps;
	ps.positions.resize((int)positions.cols());
	ps.normals.resize((int)normals.cols());
	for (int pointID = 0; pointID < (int)positions.cols(); pointID++) {
		Eigen::Vector3d p = positions.col(pointID);
		ps.positions[pointID] = vec3d(p[0], p[1], p[2]);
	}
	for (int pointID = 0; pointID < (int)normals.cols(); pointID++) {
		Eigen::Vector3d n = normals.col(pointID);
		ps.normals[pointID] = vec3d(n[0], n[1], n[2]);
	}
	ps.amount = (int)ps.positions.size();
	if (!MeshIO::savePointSet(fileName, ps)) return false;

	return true;
}

bool MapsData::visualizeSketch(string fileName, vector<vector<double>> &sketch) {

	int height = (int)sketch.size();
	int width = (int)sketch[0].size();
	vector<unsigned char> buffer(height*width, 0);
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			int color = cml::clamp((int)(sketch[row][col] * 255), 0, 255);
			buffer[row*width + col] = color;
		}
	}
	if (!ImageIO::writeImage(fileName, buffer, width, height, 1)) return false;

	return true;
}

bool MapsData::visualizeMask(string fileName, vector<vector<bool>> &mask) {

	int height = (int)mask.size();
	int width = (int)mask[0].size();
	vector<unsigned char> buffer(height*width, 0);
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			bool mk = mask[row][col];
			if (mk) buffer[row*width + col] = 255;
		}
	}
	if (!ImageIO::writeImage(fileName, buffer, width, height, 1)) return false;

	return true;
}

bool MapsData::visualizeProb(string fileName, vector<vector<double>> &prob) {

	int height = (int)prob.size();
	int width = (int)prob[0].size();
	vector<unsigned char> buffer(height*width, 0);
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			double p = prob[row][col];
			buffer[row*width + col] = (unsigned char)(p * 255);
		}
	}
	if (!ImageIO::writeImage(fileName, buffer, width, height, 1)) return false;

	return true;
}

bool MapsData::visualizeDepth(string fileName, vector<vector<double>> &depth) {

	int height = (int)depth.size();
	int width = (int)depth[0].size();
	vector<unsigned char> buffer(height*width, 0);
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			double d = depth[row][col];
			buffer[row*width + col] = (unsigned char)((d*0.5 + 0.5) * 255);
		}
	}
	if (!ImageIO::writeImage(fileName, buffer, width, height, 1)) return false;

	return true;
}

bool MapsData::visualizeNormal(string fileName, vector<vector<vec3d>> &normal) {

	int height = (int)normal.size();
	int width = (int)normal[0].size();
	vector<unsigned char> buffer(height*width * 3, 0);
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			vec3d n = normal[row][col];
			for (int k = 0; k < 3; k++) {
				buffer[(row*width + col) * 3 + k] = (unsigned char)((n[k] * 0.5 + 0.5) * 255);
			}
		}
	}
	if (!ImageIO::writeImage(fileName, buffer, width, height, 3)) return false;

	return true;
}

bool MapsData::error(string s) {

	cout << "Error: " << s << endl;
	system("pause");

	return false;
}