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


#include "RenderMesh.h"

#include <Windows.h>
#include <gl/glew.h>
#include <gl/gl.h>

#include "Mesh/MeshIO.h"
#include "Mesh/MeshView.h"

#include "Image/ImageTransform.h"
#include "Image/ImageIO.h"

#include "RenderContext.h"

using namespace Monster;

GLuint RenderMesh::mTexID;
GLuint RenderMesh::mFBOID;
GLuint RenderMesh::mRBID;
GLuint RenderMesh::mShaderID;
int RenderMesh::mImageSize;

bool RenderMesh::render(
	TTriangleMesh &mesh,
	Eigen::Matrix4f &viewMatrix,
	Eigen::Matrix4f &rotateMatrix,
	vector<vector<bool>> &masks,
	vector<vector<double>> &depths,
	vector<vector<vec3d>> &normals)
{

	GLuint vao;    // vertex array object
	GLuint vbo[3]; // vertex buffer object (position / normal / index)
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(3, vbo);
	if (!checkGLError(-1)) return false;

	GLsizeiptr bufferSize = mesh.amount * 3 * sizeof(float);

	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, bufferSize, (GLvoid*)&(mesh.positions.front()), GL_STATIC_DRAW);
	GLint loc_aPosition = glGetAttribLocation(mShaderID, "aPosition");
	glVertexAttribPointer(loc_aPosition, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc_aPosition);
	if (!checkGLError(-2)) return false;

	glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, bufferSize, (GLvoid*)&(mesh.normals.front()), GL_STATIC_DRAW);
	GLint loc_aNormal = glGetAttribLocation(mShaderID, "aNormal");
	glVertexAttribPointer(loc_aNormal, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(loc_aNormal);
	if (!checkGLError(-3)) return false;

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[2]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * 3 * sizeof(int), (GLvoid*)&(mesh.indices.front()), GL_STATIC_DRAW);
	if (!checkGLError(-4)) return false;

	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(mShaderID);
	if (!checkGLError(1)) return false;

	glBindFragDataLocation(mShaderID, 0, "oColor");
	if (!checkGLError(2)) return false;

	glUniformMatrix4fv(glGetUniformLocation(mShaderID, "uViewMatrix"), 1, false, viewMatrix.data());
	glUniformMatrix4fv(glGetUniformLocation(mShaderID, "uRotateMatrix"), 1, false, rotateMatrix.data());
	if (!checkGLError(3)) return false;

	glDrawElements(GL_TRIANGLES, (GLsizei)(mesh.indices.size() * 3), GL_UNSIGNED_INT, 0);
	if (!checkGLError(4)) return false;

	vector<unsigned short> imageBuffer(mImageSize*mImageSize * 4);
	glReadPixels(0, 0, mImageSize, mImageSize, GL_RGBA, GL_UNSIGNED_SHORT, &imageBuffer.front());
	if (!checkGLError(5)) return false;

	if (!ImageTransform::flipVertical(imageBuffer, imageBuffer, mImageSize, mImageSize, 4)) return false;

	// visualization
	if (false) {
		if (!ImageIO::writeImage("image.png", imageBuffer, mImageSize, mImageSize, 4)) return false;
		system("pause");
	}

	masks.assign(mImageSize, vector<bool>(mImageSize, false));
	depths.assign(mImageSize, vector<double>(mImageSize, 1.0));
	normals.assign(mImageSize, vector<vec3d>(mImageSize, vec3d(1.0, 1.0, 1.0)));

	int offset = 0;
	for (int row = 0; row < mImageSize; row++) {
		for (int col = 0; col < mImageSize; col++) {
			unsigned short r = imageBuffer[offset];
			unsigned short g = imageBuffer[offset + 1];
			unsigned short b = imageBuffer[offset + 2];
			unsigned short a = imageBuffer[offset + 3];
			double x = r / (double)32768 - 1.0;
			double y = g / (double)32768 - 1.0;
			double z = b / (double)32768 - 1.0;
			double d = a / (double)32768 - 1.0;
			vec3d n(x, y, z);
			if (d < 0.9) {
				masks[row][col] = true;
				depths[row][col] = d;
				normals[row][col] = n;
			}
			offset += 4;
		}
	}

	glBindVertexArray(0);
	glDeleteBuffers(3, vbo);
	glDeleteVertexArrays(1, &vao);

	return true;
}

bool RenderMesh::init(int imageSize) {

	if (!RenderContext::init()) return false;

	if (!initGL(imageSize)) return false;
	if (!initShader()) return false;

	return true;
}

bool RenderMesh::finish() {

	glDeleteProgram(mShaderID);
	glDeleteTextures(1, &mTexID);
	glDeleteRenderbuffers(1, &mRBID);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDeleteFramebuffers(1, &mFBOID);

	if (!RenderContext::close()) return false;

	return true;
}

bool RenderMesh::initGL(int imageSize) {

	// init flag

	//cout << "Initializing flags..." << endl;
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_ALPHA_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);
	glDisable(GL_DITHER);
	glEnable(GL_LIGHTING);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
	glEnable(GL_LIGHT0);

	// init FBO

	//cout << "Initializing texture..." << endl;
	glGenTextures(1, &mTexID);
	glBindTexture(GL_TEXTURE_2D, mTexID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16, imageSize, imageSize, 0, GL_RGBA, GL_UNSIGNED_SHORT, NULL);

	//cout << "Initializing FBO..." << endl;
	glGenFramebuffers(1, &mFBOID);
	glBindFramebuffer(GL_FRAMEBUFFER, mFBOID);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mTexID, 0);
	glGenRenderbuffers(1, &mRBID);
	glBindRenderbuffer(GL_RENDERBUFFER, mRBID);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, imageSize, imageSize);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, mRBID);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) return error("Frame buffer error");

	glViewport(0, 0, (GLsizei)imageSize, (GLsizei)imageSize);

	mImageSize = imageSize;

	return true;
}

bool RenderMesh::initShader() {

	//cout << "Initializing shaders..." << endl;

	string szVert = \
		" #version 330                                          \n"
		" in vec3 aPosition;                                    \n"
		" in vec3 aNormal;                                      \n"
		" uniform mat4 uViewMatrix;                             \n"
		" uniform mat4 uRotateMatrix;                           \n"
		" out vec3 normal;                                      \n"
		" out float depth;                                      \n"
		" void main() {                                         \n"
		" 	gl_Position = uViewMatrix * vec4(aPosition, 1.0);   \n"
		" 	normal = (uRotateMatrix * vec4(aNormal, 1.0)).xyz;  \n"
		"   if(normal.z<0.0) normal = -normal;                  \n"
		"   depth = gl_Position.z;                              \n"
		" }                                                     \n"
		"";
	string szFrag = \
		" #version 330                                          \n"
		" in vec3 normal;                                       \n"
		" in float depth;                                       \n"
		" out vec4 oColor;                                      \n"
		" void main() {                                         \n"
		" 	oColor = vec4(normalize(normal), depth)*0.5+0.5;    \n"
		" }                                                     \n"
		"";

	mShaderID = glCreateProgram();
	unsigned int vertexShaderID, fragmentShaderID;

	vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
	fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

	glAttachShader(mShaderID, vertexShaderID);
	glAttachShader(mShaderID, fragmentShaderID);

	const char * sources[] = { szVert.c_str(), szFrag.c_str() };
	unsigned int ids[] = { vertexShaderID, fragmentShaderID };

	for (int i = 0; i<2; i++) {
		glShaderSource(ids[i], 1, sources + i, NULL);
		glCompileShader(ids[i]);
		int logLength = 0;
		glGetShaderiv(ids[i], GL_INFO_LOG_LENGTH, &logLength);
		if (logLength > 1) {
			char *log = new char[logLength];
			glGetShaderInfoLog(ids[i], logLength, 0, log);
			string szLog(log);
			delete[] log;
			return error("Shader compile error -- " + szLog);
		}
	}

	if (true) {
		glLinkProgram(mShaderID);
		int logLength = 0;
		glGetProgramiv(mShaderID, GL_INFO_LOG_LENGTH, &logLength);
		if (logLength > 1) {
			char *log = new char[logLength];
			glGetProgramInfoLog(mShaderID, logLength, 0, log);
			string szLog(log);
			delete[] log;
			return error("Shader link error -- " + szLog);
		}
	}

	return true;
}

bool RenderMesh::checkGLError(int checkPoint) {
	int errorID;
	if ((errorID = glGetError()) != GL_NO_ERROR) {
		stringstream ss;
		ss << "GLError " << errorID << " @ " << checkPoint;
		return error(ss.str());
	}
	return true;
}

bool RenderMesh::error(string s) {

	cout << "Error: " << s << endl;
	system("pause");

	return false;
}