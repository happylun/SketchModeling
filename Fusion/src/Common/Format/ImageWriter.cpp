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


#include "ImageWriter.h"

#include <iostream>
#include <fstream>

#include "Library/FreeImageHelper.h"

ImageWriter ImageWriter::mInst;

ImageWriter::ImageWriter() {

#ifdef FREEIMAGE_LIB
	FreeImage_Initialise();
#endif
}

ImageWriter::~ImageWriter() {
#ifdef FREEIMAGE_LIB
	FreeImage_DeInitialise();
#endif
}

bool ImageWriter::writeImage(string fileName, vector<unsigned char> &image, int width, int height, int channel) {

	return mInst.write(fileName, image, width, height, channel);
}

bool ImageWriter::writeImage(string fileName, vector<unsigned short> &image, int width, int height, int channel) {

	return mInst.write(fileName, image, width, height, channel);
}

bool ImageWriter::write(string fileName, vector<unsigned char> &image, int width, int height, int channel) {

	int bufferSize = (int)image.size();
	if (bufferSize != width*height*channel) {
		cout << "Error: incorrect image size:" << endl;
		cout << width << " x " << height << " x " << channel << " != " << bufferSize << endl;
		return false;
	}
	if (channel != 1 && channel != 3) {
		cout << "Error: cannot write image with " << channel << " channels" << endl;
		return false;
	}

	FREE_IMAGE_FORMAT fif = FreeImage_GetFIFFromFilename(fileName.c_str());
	if (fif == FIF_UNKNOWN) {
		cout << "Error: cannot determine image format from file name " << fileName << endl;
		return false;
	}
	if (!FreeImage_FIFSupportsWriting(fif)) {
		cout << "Error: FreeImage does not support writing this type of image: " << fileName << endl;
		return false;
	}

	FIBITMAP *bitmap = FreeImage_Allocate(width, height, channel * 8);
	if (!bitmap) {
		cout << "Error: failed to allocate memory for writing image" << endl;
		return false;
	}

	unsigned char *bits = FreeImage_GetBits(bitmap);
	int pitch = FreeImage_GetPitch(bitmap);
	int image_ptr = 0;
	
	for (int h = 0; h < height; h++) {
		unsigned char *pixel = bits;
		for (int w = 0; w < width; w++) {
			if (channel == 1) {
				pixel[0] = image[image_ptr];
				pixel++;
				image_ptr++;
			} else {
				pixel[FI_RGBA_RED] = image[image_ptr];
				pixel[FI_RGBA_GREEN] = image[image_ptr+1];
				pixel[FI_RGBA_BLUE] = image[image_ptr+2];
				pixel += 3;
				image_ptr += 3;
			}
		}
		bits += pitch;
	}

	if (!FreeImage_FlipVertical(bitmap)) return false;

	if (!FreeImage_Save(fif, bitmap, fileName.c_str())) {
		cout << "Error: cannot write image to file " << fileName << endl;
		FreeImage_Unload(bitmap);
		return false;
	}

	FreeImage_Unload(bitmap);
	return true;
}

bool ImageWriter::write(string fileName, vector<unsigned short> &image, int width, int height, int channel) {

	int bufferSize = (int)image.size();
	if (bufferSize != width*height*channel) {
		cout << "Error: incorrect image size:" << endl;
		cout << width << " x " << height << " x " << channel << " != " << bufferSize << endl;
		return false;
	}
	if (channel != 1 && channel != 3 && channel != 4) {
		cout << "Error: cannot write image with " << channel << " channels" << endl;
		return false;
	}

	FREE_IMAGE_FORMAT fif = FreeImage_GetFIFFromFilename(fileName.c_str());
	if (fif == FIF_UNKNOWN) {
		cout << "Error: cannot determine image format from file name " << fileName << endl;
		return false;
	}
	if (!FreeImage_FIFSupportsWriting(fif)) {
		cout << "Error: FreeImage does not support writing this type of image: " << fileName << endl;
		return false;
	}

	FIBITMAP *bitmap;
	if (channel == 1) {
		bitmap = FreeImage_AllocateT(FIT_UINT16, width, height);
	} else if (channel == 3) {
		bitmap = FreeImage_AllocateT(FIT_RGB16, width, height);
	} else if (channel == 4) {
		bitmap = FreeImage_AllocateT(FIT_RGBA16, width, height);
	} else {
		cout << "Error: cannot write image with " << channel << " channels" << endl;
		return false;
	}
	if (!bitmap) {
		cout << "Error: failed to allocate memory for writing image" << endl;
		return false;
	}

	for (int h = 0; h < height; h++) {
		BYTE* scanline = FreeImage_GetScanLine(bitmap, h);
		memcpy(scanline, (BYTE*)(image.data() + h*width*channel), sizeof(unsigned short)*width*channel);
	}

	if (!FreeImage_FlipVertical(bitmap)) return false;

	if (!FreeImage_Save(fif, bitmap, fileName.c_str())) {
		cout << "Error: cannot write image to file " << fileName << endl;
		FreeImage_Unload(bitmap);
		return false;
	}

	FreeImage_Unload(bitmap);
	return true;
}
