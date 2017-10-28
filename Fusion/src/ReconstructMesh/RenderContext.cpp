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


#include "RenderContext.h"

#include <windows.h>
#include <gl/glew.h>
#include <gl/gl.h>

HINSTANCE hInstance;
HWND      hWnd;
HDC       hDC;
HGLRC     hRC;

using namespace Monster;

LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM  wParam, LPARAM  lParam) {
	return DefWindowProc (hWnd, uMsg, wParam, lParam);
}

bool RenderContext::init() {

	const LPCWSTR appname = TEXT("GLOffscreen");
	hInstance = GetModuleHandle(NULL);

	WNDCLASS wndclass;
	{
		wndclass.style         = 0;
		wndclass.lpfnWndProc   = WndProc;
		wndclass.cbClsExtra    = 0;
		wndclass.cbWndExtra    = 0;
		wndclass.hInstance     = hInstance;
		wndclass.hIcon         = LoadIcon(hInstance, appname);
		wndclass.hCursor       = LoadCursor(NULL,IDC_ARROW);
		wndclass.hbrBackground = (HBRUSH)(COLOR_WINDOW+1);
		wndclass.lpszMenuName  = appname;
		wndclass.lpszClassName = appname;
	}
	if (!RegisterClass(&wndclass)) {
		DWORD err = GetLastError();
		if (err != ERROR_CLASS_ALREADY_EXISTS) return false;
	}

	hWnd = CreateWindow(appname, appname, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT, 1, 1, NULL, NULL, hInstance, NULL);
	if(!(hDC = GetDC(hWnd))) return false;

	PIXELFORMATDESCRIPTOR pfd;
	{
		pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
		pfd.nVersion = 1;
		pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
		pfd.dwLayerMask = PFD_MAIN_PLANE;
		pfd.iPixelType = PFD_TYPE_COLORINDEX;
		pfd.cColorBits = 16;
		pfd.cDepthBits = 16;
		pfd.cAccumBits = 0;
		pfd.cStencilBits = 0;
	}

	int pixelformat;
	if(!(pixelformat = ChoosePixelFormat(hDC, &pfd))) return false;
	if(!(SetPixelFormat(hDC, pixelformat, &pfd))) return false;

	if(!(hRC = wglCreateContext(hDC))) return false;
	if(!wglMakeCurrent(hDC, hRC)) return false;
	if(glewInit() != GLEW_OK) return false;

	return true;
}

bool RenderContext::close() {

	wglDeleteContext(hRC);
	ReleaseDC(hWnd, hDC);

	return true;
}