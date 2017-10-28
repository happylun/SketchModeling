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


#include "PlyExporter.h"

#include <fstream>
#include <sstream>

#include "PlyLoader.h"

void PlyExporter::clearUp() {

	mVertices.clear();
	mNormals.clear();
	mColors.clear();
	mFaceIndices.clear();
	mEdgeIndices.clear();
	mFaceColors.clear();
	mComments.clear();

	hasNormal = false;
	hasColor = false;
	hasFaceColor = false;
}

bool PlyExporter::addMesh(string fileName, vec3 offset, vec3i color) {

	vector<vec3> v, n;
	vector<vec3i> vi;

	PlyLoader::loadMesh(fileName, &v, &n, &vi);

	if( !addMesh(&vi, &v, &n, offset, color) ) return false;

	return true;
}


bool PlyExporter::addMesh(string fileName, matrix transform, vec3i color) {

	vector<vec3> v, n;
	vector<vec3i> vi;

	PlyLoader::loadMesh(fileName, &v, &n, &vi);

	if( !addMesh(&vi, &v, &n, transform, color) ) return false;

	return true;
}

bool PlyExporter::addMesh(vector<vec3i> *indices, vector<vec3> *vertices, vector<vec3> *normals, vec3 offset, vec3i color) {

	matrix mat = cml::identity_4x4();
	cml::matrix_translation(mat, offset);

	return addMesh(indices, vertices, normals, mat, color);
}

bool PlyExporter::addMesh(vector<vec3i> *indices, vector<vec3> *vertices, vector<vec3> *normals, matrix transform, vec3i color) {

	int elementOffset = (int)mVertices.size();

	// add vertices

	for(auto v : *vertices) {
		v = cml::transform_point_4D(transform, v);
		mVertices.push_back( v );
	}

	// add normals

	if(normals) {
		if(vertices->size() != normals->size()) return false;

		matrix normalTransform = transform;
		cml::matrix_set_translation(normalTransform, vec3(0.0f,0.0f,0.0f));
		normalTransform.inverse();
		normalTransform.transpose();

		for(auto n : *normals) {
			n = cml::transform_point_4D(normalTransform, n);
			n.normalize();
			mNormals.push_back( n );
		}

		hasNormal = true;
	} else {
		for(int i=0; i<(int)vertices->size(); i++) {
			mNormals.push_back( vec3(0.f,1.f,0.f) );
		}
	}

	// add colors

	if(color != vec3i(0,0,0)) {
		hasColor = true;
	}
	for(int i=0; i<(int)vertices->size(); i++) {
		mColors.push_back( color );
	}

	// add indices

	for(auto idx : *indices) {
		mFaceIndices.push_back( idx + vec3i(elementOffset, elementOffset, elementOffset) );
	}

	return true;
}

bool PlyExporter::addMesh(vector<vec3i> *indices, vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *colors, vec3 offset) {

	int elementOffset = (int)mVertices.size();

	// add vertices

	for(auto v : *vertices) {
		mVertices.push_back( v + offset );
	}

	// add normals

	if(normals) {
		if(vertices->size() != normals->size()) return false;

		for(auto n : *normals) {
			mNormals.push_back( n );
		}

		hasNormal = true;
	} else {
		for(int i=0; i<(int)vertices->size(); i++) {
			mNormals.push_back( vec3(0.f,1.f,0.f) );
		}
	}

	// add colors

	if(colors) {
		if(vertices->size() != colors->size()) return false;

		for(auto c : *colors) {
			mColors.push_back( c );
		}

		hasColor = true;
	} else {
		for(int i=0; i<(int)vertices->size(); i++) {
			mColors.push_back( vec3i(0, 0, 0) );
		}
	}

	// add indices

	for(auto idx : *indices) {
		mFaceIndices.push_back( idx + vec3i(elementOffset, elementOffset, elementOffset) );
	}

	return true;
}

bool PlyExporter::addMesh(vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *indices, vec3i faceColors, vec3 offset) {

	int elementOffset = (int)mVertices.size();

	// add vertices

	for (auto v : *vertices) {
		mVertices.push_back(v + offset);
	}

	// add normals

	if (normals) {
		if (vertices->size() != normals->size()) return false;

		for (auto n : *normals) {
			mNormals.push_back(n);
		}

		hasNormal = true;
	} else {
		for (int i = 0; i<(int)vertices->size(); i++) {
			mNormals.push_back(vec3(0.f, 1.f, 0.f));
		}
	}

	// add face colors

	if (true) {

		for (int i = 0; i<(int)indices->size(); i++) {
			mFaceColors.push_back(faceColors);
		}

		hasFaceColor = true;
	}

	// add indices

	for (auto idx : *indices) {
		mFaceIndices.push_back(idx + vec3i(elementOffset, elementOffset, elementOffset));
	}

	return true;
}

bool PlyExporter::addMesh(vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *indices, vector<vec3i> *faceColors, vec3 offset) {

	int elementOffset = (int)mVertices.size();

	// add vertices

	for(auto v : *vertices) {
		mVertices.push_back(v + offset);
	}

	// add normals

	if(normals) {
		if(vertices->size() != normals->size()) return false;

		for(auto n : *normals) {
			mNormals.push_back( n );
		}

		hasNormal = true;
	} else {
		for(int i=0; i<(int)vertices->size(); i++) {
			mNormals.push_back( vec3(0.f,1.f,0.f) );
		}
	}

	// add face colors

	if(faceColors) {
		if(indices->size() != faceColors->size()) return false;

		int blankColorNum = (int)(mFaceIndices.size() - mFaceColors.size());
		for(int i=0; i<blankColorNum; i++) {
			mFaceColors.push_back( vec3i(0,0,0) );
		}

		for(auto c : *faceColors) {
			mFaceColors.push_back( c );
		}

		hasFaceColor = true;
	} else {
		// do nothing
	}

	// add indices

	for(auto idx : *indices) {
		mFaceIndices.push_back( idx + vec3i(elementOffset, elementOffset, elementOffset) );
	}

	return true;
}

bool PlyExporter::addPoint(string fileName, vec3 offset, vec3i color) {

	vector<vec3> v, n;

	PlyLoader::loadMesh(fileName, &v, &n);

	if( !addPoint(&v, &n, offset, color) ) return false;

	return true;
}

bool PlyExporter::addPoint(string fileName, matrix transform, vec3i color) {

	vector<vec3> v, n;

	PlyLoader::loadMesh(fileName, &v, &n);

	if( !addPoint(&v, &n, transform, color) ) return false;

	return true;
}

bool PlyExporter::addPoint(vector<vec3> *vertices, vector<vec3> *normals, vec3 offset, vec3i color) {

	matrix mat = cml::identity_4x4();
	cml::matrix_translation(mat, offset);

	return addPoint(vertices, normals, mat, color);
}

bool PlyExporter::addPoint(vector<vec3> *vertices, vector<vec3> *normals, matrix transform, vec3i color) {

	// add vertices

	for(auto v : *vertices) {
		v = cml::transform_point_4D(transform, v);
		mVertices.push_back( v );
	}

	// add normals

	if(normals) {
		if(vertices->size() != normals->size()) return false;

		matrix normalTransform = transform;
		cml::matrix_set_translation(normalTransform, vec3(0.0f,0.0f,0.0f));
		normalTransform.inverse();
		normalTransform.transpose();

		for(auto n : *normals) {
			n = cml::transform_point_4D(normalTransform, n);
			n.normalize();
			mNormals.push_back( n );
		}

		hasNormal = true;
	} else {
		for(int i=0; i<(int)vertices->size(); i++) {
			mNormals.push_back( vec3(0.f,1.f,0.f) );
		}
	}

	// add colors

	if(color != vec3i(0,0,0)) {
		hasColor = true;
	}
	for(int i=0; i<(int)vertices->size(); i++) {
		mColors.push_back( color );
	}

	return true;
}

bool PlyExporter::addPoint(vector<vec3> *vertices, vector<vec3> *normals, vector<vec3i> *colors, vec3 offset) {

	// add vertices

	for(auto v : *vertices) {
		mVertices.push_back( v+offset );
	}

	// add normals

	if(normals) {
		if(vertices->size() != normals->size()) return false;

		for(auto n : *normals) {
			mNormals.push_back( n );
		}

		hasNormal = true;
	} else {
		for(int i=0; i<(int)vertices->size(); i++) {
			mNormals.push_back( vec3(0.f,1.f,0.f) );
		}
	}

	// add colors

	if(colors) {
		if(vertices->size() != colors->size()) return false;

		for(auto c : *colors) {
			mColors.push_back( c );
		}

		hasColor = true;
	} else {
		for(int i=0; i<(int)vertices->size(); i++) {
			mColors.push_back( vec3i(0, 0, 0) );
		}
	}

	return true;
}

bool PlyExporter::addLine(vector<vec2i> *indices, vector<vec3> *vertices) {

	int numLines = (int)indices->size();

	int vertexOffset = (int)mVertices.size();

	// add vertices

	for (auto v : *vertices) {
		mVertices.push_back(v);
	}

	// add indices

	for (int i = 0; i<numLines; i++) {
		vec2i idx = (*indices)[i];
		mEdgeIndices.push_back(vec2i(vertexOffset + idx[0], vertexOffset + idx[1]));
	}

	return true;
}

bool PlyExporter::addLine(vector<vec3> *pointPairs) {

	int numLines = (int)pointPairs->size() / 2;

	int elementOffset = (int)mVertices.size();

	// add vertices

	for (auto v : *pointPairs) {
		mVertices.push_back(v);
	}

	// add normals

	if (hasNormal) {
		for (int i = 0; i < numLines * 2; i++) {
			mNormals.push_back(vec3(0.f, 1.f, 0.f));
		}
	}

	// add colors

	if (hasColor) {
		for (int i = 0; i < numLines * 2; i++) {
			mColors.push_back(vec3i(0, 0, 0));
		}
	}

	// add indices

	for (int i = 0; i<numLines; i++) {
		mEdgeIndices.push_back(vec2i(elementOffset + i * 2, elementOffset + i * 2 + 1));
	}

	return true;
}

bool PlyExporter::addLine(vector<vec3> *points1, vector<vec3> *points2) {

	if(points1->size() != points2->size()) return false;

	int numLines = (int)points1->size();

	int elementOffset = (int)mVertices.size();

	// add vertices

	for(auto v : *points1) {
		mVertices.push_back( v );
	}

	for(auto v : *points2) {
		mVertices.push_back( v );
	}

	// add normals

	if (hasNormal) {
		for (int i = 0; i < numLines * 2; i++) {
			mNormals.push_back(vec3(0.f, 1.f, 0.f));
		}
	}

	// add colors
	if (hasColor) {
		for (int i = 0; i<numLines * 2; i++) {
			mColors.push_back(vec3i(0, 0, 0));
		}
	}

	// add indices

	for(int i=0; i<numLines; i++) {
		mEdgeIndices.push_back( vec2i(elementOffset+i, elementOffset+numLines+i) );
	}

	return true;
}

bool PlyExporter::addComment(string comment) {

	if (comment.empty()) return true;
	const int max_len = 100;
	int len = (int)comment.length();
	for (int j = 0; j <= (len - 1) / max_len; j++) {
		mComments.push_back(comment.substr(max_len*j, max_len));
	}

	return true;
}

bool PlyExporter::output(string fileName, bool ascii) {

	if(hasNormal && mNormals.size() != mVertices.size()) return false;
	if(hasColor && mColors.size() != mVertices.size()) return false;

	ofstream outFile;

	outFile.open(fileName);
	{
		outFile << "ply" << endl;
		if(ascii) {
			outFile << "format ascii 1.0" << endl;
		} else {
			outFile << "format binary_little_endian 1.0" << endl;
		}
		if( !mComments.empty() ) {
			for(auto comment : mComments) {
				outFile << "comment " << comment << endl;
			}
		}
		outFile << "element vertex " << mVertices.size() << endl;
		outFile << "property float x" << endl;
		outFile << "property float y" << endl;
		outFile << "property float z" << endl;

		if(hasNormal) {
			outFile << "property float nx" << endl;
			outFile << "property float ny" << endl;
			outFile << "property float nz" << endl;
		}

		if(hasColor) {
			outFile << "property uchar red" << endl;
			outFile << "property uchar green" << endl;
			outFile << "property uchar blue" << endl;
			outFile << "property uchar alpha" << endl;
		}

		if(mFaceIndices.size()) {
			outFile << "element face " << mFaceIndices.size() << endl;
			outFile << "property list uchar int vertex_indices" << endl;

			if(hasFaceColor) {
				outFile << "property uchar red" << endl;
				outFile << "property uchar green" << endl;
				outFile << "property uchar blue" << endl;
				outFile << "property uchar alpha" << endl;
			}
		}

		if(mEdgeIndices.size()) {
			outFile << "element edge " << mEdgeIndices.size() << endl;

			outFile << "property int vertex1" << endl;
			outFile << "property int vertex2" << endl;
		}

		outFile << "end_header" << endl;

	}

	if(ascii) {

		for(int i=0; i<(int)(mVertices.size()); i++) {

			outFile << mVertices[i] << " ";

			if(hasNormal) {
				outFile << mNormals[i] << " ";
			}

			if(hasColor) {
				outFile << mColors[i] << " 255 ";
			}

			outFile << endl;
		}

		if(mFaceIndices.size()) {
			int numFaces = (int)mFaceIndices.size();
			for(int i=0; i<numFaces; i++) {
				outFile << "3 " << mFaceIndices[i] << endl;
				if(hasFaceColor) {
					if(i < (int)mFaceColors.size()) {
						outFile << mFaceColors[i] << " 255" << endl;
					} else {
						outFile << "0 0 0 255" << endl;
					}
				}
			}
		}

		if(mEdgeIndices.size()) {
			int numEdges = (int)mEdgeIndices.size();
			for(int i=0; i<numEdges; i++) {
				outFile << mEdgeIndices[i] << endl;
			}
		}

	} else {

		outFile.close();

		outFile.open(fileName, ios::app | ios::binary);
		{
			for(int i=0; i<(int)(mVertices.size()); i++) {

				outFile.write( (char*)(mVertices[i].data()), sizeof(vec3) );

				if(hasNormal) {
					outFile.write( (char*)(mNormals[i].data()), sizeof(vec3) );
				}

				if(hasColor) {
					for(int c=0; c<3; c++) {
						outFile.put((unsigned char)(mColors[i][c]));						
					}
					outFile.put(255u); // alpha as padding
				}
			}

			if(mFaceIndices.size()) {
				int numFaces = (int)mFaceIndices.size();
				for(int i=0; i<numFaces; i++) {
					outFile.put(3u);
					outFile.write( (char*)(mFaceIndices[i].data()), sizeof(vec3i) );
					if(hasFaceColor) {
						if(i < (int)mFaceColors.size()) {
							for(int c=0; c<3; c++) {
								outFile.put((unsigned char)(mFaceColors[i][c]));
							}
							outFile.put(255u); // alpha as padding
						} else {
							for(int c=0; c<3; c++) {
								outFile.put(0u);
							}
							outFile.put(255u); // alpha as padding
						}
					}
				}
			}

			if(mEdgeIndices.size()) {
				int numEdges = (int)mEdgeIndices.size();
				for(int i=0; i<numEdges; i++) {
					outFile.write( (char*)(mEdgeIndices[i].data()), sizeof(vec2i) );
				}
			}
		}
	}

	outFile.close();

	return true;
}