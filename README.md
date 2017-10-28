# 3D Shape Reconstruction from Sketches via Multi-view Convolutional Networks

----------

[Project Page](http://people.cs.umass.edu/~zlun/papers/SketchModeling/)

## Introduction

This archive contains source code for training / testing our algorithm for converting 2D sketches into 3D shapes. The code base contains two parts: the **Network Part** and the **Fusion Part**.

### Network Part

The network part contains Python code for predicting depth and normal maps from input sketch images. The code uses TensorFlow framework to train and test the deep neural networks. Please read the README file within the `Network` folder for more details.

### Fusion Part

The fusion part contains C++ code for fusing depth and normal maps to 3D shapes. Visual Studio is required to compile and run our code. Please read the README file within the `Fusion` folder for more details.

## Other Notes

- Our code is released under **GPL v3** license.
- If you would like to use our code, please cite the following paper:

	> Zhaoliang Lun, Matheus Gadelha, Evangelos Kalogerakis, Subhransu Maji, Rui Wang,
	"3D Shape Reconstruction from Sketches via Multi-view Convolutional Networks",
	*Proceedings of the International Conference on 3D Vision (3DV) 2017*

- For any questions or comments, please contact Zhaoliang Lun ([zlun@cs.umass.edu](mailto:zlun@cs.umass.edu))