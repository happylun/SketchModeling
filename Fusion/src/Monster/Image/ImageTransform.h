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


#pragma once

#include <vector>

using namespace std;

namespace Monster {

	class ImageTransform {

	private:

		// make it non-instantiable
		ImageTransform();
		~ImageTransform();

	public:

		template<class T>
		static bool flipVertical(
			vector<T> &inImage,
			vector<T> &outImage,
			int width, int height, int channel)
		{
			if (width * height * channel != (int)inImage.size()) {
				cout << "Error: incorrect image size" << endl;
				return false;
			}
			vector<T> buffer(inImage.size());
			int lineSize = width * channel;
			for (int h = 0; h < height; h++) {
				auto first = inImage.begin() + h * lineSize;
				auto last = first + lineSize;
				auto dest = buffer.begin() + (height - h - 1) * lineSize;
				copy(first, last, dest);
			}
			outImage.swap(buffer);
			return true;
		}

		template<class T>
		static bool flipHorizontal(
			vector<T> &inImage,
			vector<T> &outImage,
			int width, int height, int channel)
		{
			if (width * height * channel != (int)inImage.size()) {
				cout << "Error: incorrect image size" << endl;
				return false;
			}
			vector<T> buffer(inImage.size());
			int lineSize = width * channel;
			for (int h = 0; h < height; h++) {
				auto first = inImage.begin() + h * lineSize;
				auto last = first + lineSize;
				vector<T> line(first, last);
				vector<T> flipLine(line.rbegin(), line.rend());
				auto dest = buffer.begin() + h * lineSize;
				copy(flipLine.begin(), flipLine.end(), dest);
			}
			outImage.swap(buffer);
			return true;
		}
	};
}