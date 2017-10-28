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


#include "MeshVoxelize.h"

#include "Sample/SampleSimplePoissonDisk.h"

#include "Mesh/MeshClean.h"
#include "Mesh/MeshIO.h"

using namespace Monster;

bool MeshVoxelize::voxelize(
	TTriangleMesh &mesh,
	double range, int resolution,
	vector<vector<vector<bool>>> &voxels)
{
	// grid range: [-range, range]^3
	// grid resolution: resolution^3

	voxels.assign(resolution, vector<vector<bool>>(resolution, vector<bool>(resolution, false)));
	double gap = range*2.0 / resolution;

	// Poisson sampling

	int maxNumSamples = 500000;
	double sampleRadius = gap * 0.2;
	TSampleSet samples;
	SampleSimplePoissonDisk sspd(&mesh);
	if (!sspd.runSampling(maxNumSamples, sampleRadius)) return false;
	if (!sspd.exportSample(samples)) return false;
	//if (!MeshIO::savePointSet("sample.ply", samples)) return false;

	// mark voxel occupancy

#pragma omp parallel for
	for (int sampleID = 0; sampleID < samples.amount; sampleID++) {
		vec3 point = samples.positions[sampleID];
		vec3i index;
		for (int k = 0; k < 3; k++) {
			index[k] = cml::clamp((int)((point[k] + range) / gap), 0, resolution - 1);
		}
		voxels[index[0]][index[1]][index[2]] = true;
	}

	//if (true) {
	//	TTriangleMesh mesh;
	//	if (!voxels2mesh(voxels, range, mesh)) return false;
	//	if (!MeshIO::saveMesh("unfill.ply", mesh)) return false;
	//}

	// fill internal voxels

	if (!fillVoxels(voxels)) return false;

	//if (true) {
	//	TTriangleMesh mesh;
	//	if (!voxels2mesh(voxels, range, mesh)) return false;
	//	if (!MeshIO::saveMesh("fill.ply", mesh)) return false;
	//}
	//system("pause");

	return true;
}

bool MeshVoxelize::fillVoxels(vector<vector<vector<bool>>> &voxels) {

	int numX = (int)voxels.size();
	int numY = (int)voxels[0].size();
	int numZ = (int)voxels[0][0].size();

	vector<vector<vector<bool>>> filledVoxels(numX, vector<vector<bool>>(numY, vector<bool>(numZ, true)));
	vector<vec3i> inspectOffsets = {
		vec3i(-1, 0, 0), vec3i(1, 0, 0), vec3i(0, -1, 0), vec3i(0, 1, 0), vec3i(0, 0, -1), vec3i(0, 0, 1)
	};
	vector<vec3i> queue(1, vec3i(0, 0, 0)); // NOTE: make sure that this corner point is always empty
	filledVoxels[0][0][0] = false;
	int head = 0;
	while (head < (int)queue.size()) {
		vec3i curIdx = queue[head];
		for (vec3i offset : inspectOffsets) {
			vec3i nbIdx = curIdx + offset;
			if (nbIdx[0] < 0 || nbIdx[0] >= numX ||
				nbIdx[1] < 0 || nbIdx[1] >= numY ||
				nbIdx[2] < 0 || nbIdx[2] >= numZ) continue;
			if (voxels[nbIdx[0]][nbIdx[1]][nbIdx[2]]) continue;
			if (filledVoxels[nbIdx[0]][nbIdx[1]][nbIdx[2]]) {
				filledVoxels[nbIdx[0]][nbIdx[1]][nbIdx[2]] = false;
				queue.push_back(nbIdx);
			}
		}
		head++;
	}
	voxels.swap(filledVoxels);

	return true;
}

bool MeshVoxelize::voxels2mesh(
	vector<vector<vector<bool>>> &voxels,
	double range, TTriangleMesh &mesh)
{

	// compute voxel metrics

	int numX = (int)voxels.size();
	int numY = (int)voxels[0].size();
	int numZ = (int)voxels[0][0].size();
	double gapX = range*2.0 / numX;
	double gapY = range*2.0 / numY;
	double gapZ = range*2.0 / numZ;

	// compute all vertex positions

	mesh.positions.clear();
	for (int x = 0; x <= numX; x++) {
		for (int y = 0; y <= numY; y++) {
			for (int z = 0; z <= numZ; z++) {
				vec3 p = vec3d(gapX*x - range, gapY*y - range, gapZ*z - range);
				mesh.positions.push_back(p);
			}
		}
	}
	mesh.amount = (int)mesh.positions.size();
	mesh.normals.assign(mesh.amount, vec3(0.0f, 0.0f, 0.0f));

	// extract all voxel faces

	vector<vec3i> boxIndices = {
		vec3i(0, 1, 2), vec3i(2, 1, 3), vec3i(0, 4, 5), vec3i(0, 5, 1),
		vec3i(0, 2, 6), vec3i(0, 6, 4), vec3i(5, 6, 7), vec3i(4, 6, 5),
		vec3i(2, 3, 7), vec3i(2, 7, 6), vec3i(1, 5, 7), vec3i(1, 7, 3)
	};
	mesh.indices.clear();
	for (int x = 0; x < numX; x++) {
		for (int y = 0; y < numY; y++) {
			for (int z = 0; z < numZ; z++) {
				if (!voxels[x][y][z]) continue;
				vector<int> indices(0);
				for (int xx = x; xx < x + 2; xx++) {
					for (int yy = y; yy < y + 2; yy++) {
						for (int zz = z; zz < z + 2; zz++) {
							int index = (xx*(numY + 1) + yy)*(numZ + 1) + zz;
							indices.push_back(index);
						}
					}
				}
				for (vec3i bi : boxIndices) {
					vec3i faceIdx(indices[bi[0]], indices[bi[1]], indices[bi[2]]);
					mesh.indices.push_back(faceIdx);
				}
			}
		}
	}

	// clean up mesh

	map<vec3i, int> faceCount;
	for (vec3i idx : mesh.indices) {
		vec3i key1 = idx;
		while (key1[0] > key1[1] || key1[0] > key1[2]) {
			key1 = vec3i(key1[1], key1[2], key1[0]);
		}
		vec3i key2 = vec3i(key1[0], key1[2], key1[1]);
		auto &it1 = faceCount.find(key1);
		auto &it2 = faceCount.find(key2);
		if (it1 != faceCount.end()) {
			it1->second++;
		} else if (it2 != faceCount.end()) {
			it2->second++;
		} else {
			faceCount[key1] = 1;
		}
	}
	mesh.indices.clear();
	for (auto it : faceCount) {
		if (it.second == 1) {
			mesh.indices.push_back(it.first);
		}
	}
	if (!MeshClean::cleanUp(mesh)) return false;

	return true;
}