# 3D Shape Reconstruction from Sketches via Multi-view Convolutional Networks
## Network Part

----------

[Project Page](http://people.cs.umass.edu/~zlun/papers/SketchModeling/)

## Introduction

This code base contains Python code for testing or training our network model. Our network takes sketch images as input and outputs multi-view depth and normal maps. To fuse the multi-view depth and normal maps into a 3D shape, please refer to our code for Fusion Part.

Before running our code, please make sure that your machine has satisfied the **Package Requirements** and **Hardware Requirements** listed below. Then you can follow the **How to Run** instructions to run our code.

Our code is released under **GPL v3** license.

## Package Requirements

- [Python 3.5](https://www.python.org/)
- [NumPy 1.12.1](http://www.numpy.org/)
- [SciPy 0.19.0](https://www.scipy.org/)
- [TensorFlow 1.0.1](https://www.tensorflow.org/)

## Hardware Requirements

Our code has been tested only on **GeForce GTX Titan X** and **Tesla M40** graphics card. A graphics card with 12+ GB video memory is recommended.

## How to Run

### Testing

- Download testing data files [here](http://antares.cs.umass.edu/project_data/AdversarialMonsters/TestingData.7z)
- Download pre-trained network parameters files [here](https://www.dropbox.com/s/1prh2tpn0w1ak3l/Zhaoliang%20Lun%20-%20Checkpoint.7z?dl=0)
- Launch a command line window and go inside folder `code/MonsterNet/`
- Run this command

	```
	python main.py --test --data_dir [DATA_DIR] --train_dir [TRAINING_DIR] --test_dir [TESTING_DIR]
	```

- You will need to configure the input sketch data directory (`[DATA_DIR]`), the pre-trained network parameters directory (`[TRAINING_DIR]`) and the output result directory (`[TESTING_DIR]`) correspondingly. See **Command-line Arguments** section below for more details

### Training

- Download training data [here](https://www.dropbox.com/s/3a8xf2oozrvuriv/Zhaoliang%20Lun%20-%20TrainingData.7z?dl=0)
- Launch a command line window and go inside folder `code/MonsterNet/`
- Run this command

	```
	python main.py --train --data_dir [DATA_DIR] --train_dir [TRAINING_DIR]
	```

- You will need to configure the training data directory (`[DATA_DIR]`) and the network parameters checkpoint directory (`[TRAINING_DIR]`) correspondingly. See **Command-line Arguments** section below for more details
- If you would like to train the network from scratch, make sure you have deleted the checkpoint files inside the checkpoint directory (`[TRAINING_DIR]`)

### Command-line Arguments

- `--train` : run network training pass (default: False)
- `--test` : run testing pass - predict maps from input sketches (default: False) 
- `--encode` : run feature encoding pass - extract features from input sketches (default: False)
- `--predict_normal` : if True, predict both depth and normal information; if False, predict depth only (default: True) 
- `--continuous_view` : if True, use a modified network architecture of Tatarchenko et al. with continuous viewing parameters as input (default: False)
- `--no_adversarial` : if True, skip adversarial loss term (default: False)
- `--batch_size INT` : use 2 for GPU with 12GB memory; use 4 for GPU with 24GB memory; the effective batch size is always 40 (default: 2)   
- `--image_size INT` : image size of input sketches and output maps (default: 256) 
- `--sketch_variations INT` : number of sketch style variations for training (default: 4)
- `--sketch_views STRING` : a string consist of one or more characters from 'F' (front), 'S' (side), 'T' (top) denoting the views of the input sketches (default: FS)  
- `--max_epochs FLOAT` : maximum number of epochs for training (default: 100)
- `--gpu_fraction FLOAT` : maximum fraction of GPU memory used (default: 0.9) 
- `--data_dir STRING` : path to the dataset directory
- `--train_dir STRING` : path to the training checkpoints & network parameters files directory 
- `--test_dir STRING` : path to the testing results output directory
- `--encode_dir STRING` : path to the encoding feature output directory
- `--view_file STRING` : an `off` format mesh file encoding output camera positions in vertex information

## Other Notes

- If you would like to use our code, please cite the following paper:

	> Zhaoliang Lun, Matheus Gadelha, Evangelos Kalogerakis, Subhransu Maji, Rui Wang,
	"3D Shape Reconstruction from Sketches via Multi-view Convolutional Networks",
	*Proceedings of the International Conference on 3D Vision (3DV) 2017*

- For any questions or comments, please contact Zhaoliang Lun ([zlun@cs.umass.edu](mailto:zlun@cs.umass.edu))
