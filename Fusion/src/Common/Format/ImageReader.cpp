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


#include "ImageReader.h"

#include <iostream>
#include <fstream>

#include "Library/FreeImageHelper.h"

ImageReader ImageReader::mInst;

ImageReader::ImageReader() {

#ifdef FREEIMAGE_LIB
	FreeImage_Initialise();
#endif
}

ImageReader::~ImageReader() {
#ifdef FREEIMAGE_LIB
	FreeImage_DeInitialise();
#endif
}

bool ImageReader::readImage(string fileName, vector<unsigned char> &image, int &width, int &height, int &channel) {

	return mInst.read(fileName, image, width, height, channel);
}

bool ImageReader::readImage(string fileName, vector<unsigned short> &image, int &width, int &height, int &channel) {

	return mInst.read(fileName, image, width, height, channel);
}

bool ImageReader::read(string fileName, vector<unsigned char> &image, int &width, int &height, int &channel) {

	FREE_IMAGE_FORMAT fif = FreeImage_GetFileType(fileName.c_str());
	if (fif == FIF_UNKNOWN) {
		FreeImage_GetFIFFromFilename(fileName.c_str());
	}
	if (fif == FIF_UNKNOWN) {
		cout << "Error: cannot determine image format from file name " << fileName << endl;
		return false;
	}
	if (!FreeImage_FIFSupportsReading(fif)) {
		cout << "Error: FreeImage does not support reading this type of image: " << fileName << endl;
		return false;
	}

	FIBITMAP *bitmap = FreeImage_Load(fif, fileName.c_str());
	if (!bitmap) {
		cout << "Error: failed to load image data" << endl;
		return false;
	}
	if (!FreeImage_FlipVertical(bitmap)) return false;

	width = (int)FreeImage_GetWidth(bitmap);
	height = (int)FreeImage_GetHeight(bitmap);
	int bpp = (int)FreeImage_GetBPP(bitmap);
	if (bpp == 8) {
		channel = 1;
	} else if (bpp == 24) {
		channel = 3;
	} else {
		cout << "Error: failed to process image with BPP = " << bpp << endl;
		return false;
	}

	image.resize(height*width*channel);

	unsigned char *bits = FreeImage_GetBits(bitmap);
	int pitch = FreeImage_GetPitch(bitmap);
	int image_ptr = 0;
	
	for (int h = 0; h < height; h++) {
		unsigned char *pixel = bits;
		for (int w = 0; w < width; w++) {
			if (channel == 1) {
				image[image_ptr] = pixel[0];
				pixel++;
				image_ptr++;
			} else {
				image[image_ptr] = pixel[FI_RGBA_RED];
				image[image_ptr + 1] = pixel[FI_RGBA_GREEN];
				image[image_ptr + 2] = pixel[FI_RGBA_BLUE];
				pixel += 3;
				image_ptr += 3;
			}
		}
		bits += pitch;
	}

	FreeImage_Unload(bitmap);
	return true;
}

bool ImageReader::read(string fileName, vector<unsigned short> &image, int &width, int &height, int &channel) {

	FREE_IMAGE_FORMAT fif = FreeImage_GetFileType(fileName.c_str());
	if (fif == FIF_UNKNOWN) {
		FreeImage_GetFIFFromFilename(fileName.c_str());
	}
	if (fif == FIF_UNKNOWN) {
		cout << "Error: cannot determine image format from file name " << fileName << endl;
		return false;
	}
	if (!FreeImage_FIFSupportsReading(fif)) {
		cout << "Error: FreeImage does not support reading this type of image: " << fileName << endl;
		return false;
	}

	FIBITMAP *bitmap = FreeImage_Load(fif, fileName.c_str());
	if (!bitmap) {
		cout << "Error: failed to load image data" << endl;
		return false;
	}
	if (!FreeImage_FlipVertical(bitmap)) return false;

	width = (int)FreeImage_GetWidth(bitmap);
	height = (int)FreeImage_GetHeight(bitmap);
	int bpp = (int)FreeImage_GetBPP(bitmap);
	if (bpp == 16) {
		channel = 1;
	} else if (bpp == 48) {
		channel = 3;
	} else if (bpp == 64) {
		channel = 4;
	} else {
		cout << "Error: failed to process image with BPP = " << bpp << endl;
		return false;
	}

	image.resize(height*width*channel);

	for (int h = 0; h < height; h++) {
		BYTE* scanline = FreeImage_GetScanLine(bitmap, h);
		memcpy((BYTE*)(image.data() + h*width*channel), scanline, sizeof(unsigned short)*width*channel);
	}

	FreeImage_Unload(bitmap);
	return true;
}