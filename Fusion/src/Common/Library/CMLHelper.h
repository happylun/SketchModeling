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

#include <cml/cml.h>

typedef cml::matrix33f_c matrix3f;
typedef cml::matrix33d_c matrix3d;
typedef cml::matrix44f_c matrix4f;
typedef cml::matrix44d_c matrix4d;
typedef matrix4f         matrix;

typedef cml::matrix<float, cml::external<4, 4>, cml::col_basis, cml::col_major> matrix4ef;
typedef cml::matrix<double, cml::external<4, 4>, cml::col_basis, cml::col_major> matrix4ed;

typedef cml::vector2f vec2f;
typedef cml::vector2d vec2d;
typedef cml::vector2i vec2i;
typedef vec2f         vec2;

typedef cml::vector3f vec3f;
typedef cml::vector3d vec3d;
typedef cml::vector3i vec3i;
typedef vec3f         vec3;

typedef cml::vector4f vec4f;
typedef cml::vector4d vec4d;
typedef cml::vector4i vec4i;
typedef vec4f         vec4;

typedef cml::quaternionf quatf;
typedef cml::quaterniond quatd;
typedef quatf            quat;
