# 3D Shape Reconstruction from Sketches via Multi-view Convolutional Networks
## Fusion Part

----------

[Project Page](http://people.cs.umass.edu/~zlun/papers/SketchModeling/)

## Introduction

This code base contains C++ code for fusing the network output maps into a 3D shape. The code here requires output data from our network. Please check our code for Network Part on how to get depth and normal maps from input sketches.

**Visual Studio 2015** or above is required to compile the code. Our code is released under **GPL v3** license.

## How to Compile

1. Open Visual Studio solution `build\Monster.sln`
2. Select targeting configurations (`Release | Win32` is recommended)
3. Build the solution by pressing `F7`


## How to Run

1. After compiling the project, the binary executables are inside folder `output\ReconstructMesh`
2. Make sure you have the input sketch data and the network output maps data. If not, please check our code for Network Part on how to get those data
3. In Visual Studio, right click project `ReconstructMesh` > `Properties`. From the left-side panel, select `Configuration Properties` > `Debugging`. On the right-side window, setup the `Command Arguments` and `Working Directory` correctly. See **Command line arguments** section below for more details 
4. Run the program by pressing `Ctrl+F5`. It will take a while 
5. After the program finishes running, the fused point cloud file (`points.ply`) should be in the result folder specified in command line arguments
6. Use the [Screened Poisson Surface Reconstruction executable](http://www.cs.jhu.edu/~misha/Code/PoissonRecon) to get a surface mesh from the output point cloud. Here is the suggested parameter setting

	```
	PoissonRecon.exe --in points.ply --out mesh.ply --depth 11 --samplesPerNode 5.0 --pointWeight 0.1
	```

7. After getting the surface mesh, optionally you can run our code in the fine-tuning stage to fine-tune the surface mesh. See **Command line arguments** section below for more details

## Command line arguments

Suppose the sketch input data folder is `Data\CharacterDraw` and the network output folder is `Data\CharacterDraw\output`, you can set the working directory to `Data` and run our program using the following command (in one line):

	ReconstructMesh.exe 1 FS 
		./CharacterDraw/hires/m1/
		./CharacterDraw/output/images/m1/
		./CharacterDraw/output/reconstruct/m1/
		./CharacterDraw/view/view.off 

Below are the detailed explanation for each command-line argument in order:

- `stage` : 1 or 2. Stage 1 is for fusion and stage 2 is for mesh fine-tuning
- `sketch_views` : a string consist of one or more characters from 'F' (front), 'S' (side), 'T' (top) denoting the views of the input sketches
- `sketch_folder` : path to the folder containing input sketches 
- `map_folder` : path to the folder containing network output maps
- `result_folder` : path to the folder for fusion output results
- `view_point_file` : an `off` format mesh file encoding output camera positions in vertex information
- `--skip_optimization` : [OPTIONAL] if set, the program will skip the optimization step for fusion
- `--symmetrization` : [OPTIONAL] if set, the program will symmetrize the output point cloud (good for airplane category where shapes are always symmetric)

## Third-party Libraries

1. All third-party library header files and pre-built binary files are included in folder `3rdparty`
2. This project requires OpenGL. You will need an OpenGL capable graphics card and development environment
3. All third-party libraries used in this project are listed below:
	- [CML 1.0.3](http://cmldev.net): simple vector, matrix and quaternion operations
	- [Eigen 3.3.2](http://eigen.tuxfamily.org): advanced matrix operations
	- [FreeImage 3.17.0](http://freeimage.sourceforge.net/): image file input/output
	- [GLEW 1.11.0](http://glew.sourceforge.net): OpenGL Extension Wrangler Library
	- [libigl](http://libigl.github.io/libigl/): tetrahedral mesh manipulation
	- [Thea](https://github.com/sidch/Thea): Kd tree implementation (modified)

## Other Notes

- If you would like to use our code, please cite the following paper:

	> Zhaoliang Lun, Matheus Gadelha, Evangelos Kalogerakis, Subhransu Maji, Rui Wang,
	"3D Shape Reconstruction from Sketches via Multi-view Convolutional Networks",
	*Proceedings of the International Conference on 3D Vision (3DV) 2017*

- For any questions or comments, please contact Zhaoliang Lun ([zlun@cs.umass.edu](mailto:zlun@cs.umass.edu))
