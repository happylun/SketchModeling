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

#include <string>

using namespace std;

typedef __int64 TTimer;

class TimerUtility {

private:

	TimerUtility() {}
	~TimerUtility() {}

public:
	
	static TTimer tic();
	static int toc(TTimer t = 0); // msec
	static string toString(int time); // msec
	static void sleep(int time); // msec

private:

	static __int64 frequency;
	static __int64 counter;
};