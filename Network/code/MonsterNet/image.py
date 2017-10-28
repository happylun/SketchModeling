"""
	This file is part of the Sketch Modeling project.

	Copyright (c) 2017
	-Zhaoliang Lun (author of the code) / UMass-Amherst

	This is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This software is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this software.  If not, see <http://www.gnu.org/licenses/>.
"""


import tensorflow as tf
import numpy as np
from scipy import ndimage

import os

########################### image processing ###########################

def normalize_image(image):
	# normalize to [-1.0, 1.0]
	if image.dtype == tf.uint8:
		return tf.to_float(image)/127.5-1.0
	elif image.dtype == tf.uint16:
		return tf.to_float(image)/32767.5-1.0
	else:
		return tf.to_float(image)

def unnormalize_image(image, maxval=255.0):
	# restore image to [0.0, maxval]
	return (image+1.0)*maxval*0.5

def saturate_image(image, dtype=tf.uint8):
	return tf.saturate_cast(image, dtype)

def convert_to_rgb(image, channels=3):
	return tf.tile(image, [1,1,1,channels])


########################### masks ###########################

def extract_boolean_mask(image):
	"""
		input:
			image:   n x H x W x C : images with value range [-1.0, 1.0] in each channel
		output:
			mask:    n x H x W x 1 : boolean mask (depth channel value < 0.9)
	"""

	depth = tf.slice(image, [0,0,0,3], [-1,-1,-1,1])
	shape = depth.get_shape()
	mask = tf.where(tf.greater(depth, 0.9),
		tf.constant(False, dtype=tf.bool, shape=shape),
		tf.constant(True, dtype=tf.bool, shape=shape))
	return mask

def convert_to_real_mask(bool_mask):
	"""
		input:
			bool_mask:   boolean mask image
		output:
			real_mask:   real number mask image (-1.0: false, 1.0: true)
	"""

	shape = bool_mask.get_shape()
	return tf.where(bool_mask,
		tf.constant(1.0, dtype=tf.float32, shape=shape),
		tf.constant(-1.0, dtype=tf.float32, shape=shape))

def convert_to_boolean_mask(real_mask):
	"""
		input:
			real_mask:   real number mask image (-1.0: false, 1.0: true)
		output:
			bool_mask:   boolean mask image
	"""
	shape = real_mask.get_shape()
	return tf.where(tf.greater(real_mask, 0.0),
		tf.constant(True, dtype=tf.bool, shape=shape),
		tf.constant(False, dtype=tf.bool, shape=shape))

def apply_mask(content, mask):
	"""
		input:
			content:   n x H x W x C  : image content
			mask:      n x H x W x 1  : image mask (>0: true)
		output:
			output:    use content value if mask is true; 1.0 otherwise
	"""
	channel = content.get_shape()[3].value
	if channel > 1:
		mask = tf.tile(mask, [1,1,1,channel])
	return tf.where(tf.greater(mask, 0.0), content, tf.ones_like(content))


########################### filter ###########################

def get_sobel_filter():

	# 3x3 sobel filter
	filter_v = tf.convert_to_tensor(np.array([ \
		[-1.0, 0.0, 1.0],
		[-2.0, 0.0, 2.0],
		[-1.0, 0.0, 1.0]]), dtype=tf.float32)
	filter_h = tf.convert_to_tensor(np.array([ \
		[ 1.0,  2.0,  1.0],
		[ 0.0,  0.0,  0.0],
		[-1.0, -2.0, -1.0]]), dtype=tf.float32)
	return filter_v, filter_h

def get_dog_filter(kernel_size):
		
	# derivative of gaussian filter
	kernel_point = np.zeros((kernel_size, kernel_size))
	kernel_point[kernel_size//2,kernel_size//2] = 1
	kernel_v = ndimage.filters.gaussian_filter(kernel_point, sigma=kernel_size//2, order=[0,1]) * (kernel_size*kernel_size)
	kernel_h = kernel_v.T
	filter_v = tf.constant(kernel_v, dtype=tf.float32)
	filter_h = tf.constant(kernel_h, dtype=tf.float32)
	filter_v = tf.expand_dims(tf.expand_dims(filter_v, -1), -1)
	filter_h = tf.expand_dims(tf.expand_dims(filter_h, -1), -1)
	return filter_v, filter_h

def apply_edge_filter(images):
	"""
		input:
			images:   n x H x W x C    input images
		output:
			outputs:  n x H x W x 1    output edge images
	"""

	if images.get_shape()[3].value == 1:
		gray_images = images
	else:
		gray_images = tf.image.rgb_to_grayscale(images)

	if not hasattr(apply_edge_filter, "filter"):
		apply_edge_filter.filter = get_dog_filter(15)

	edge_v = tf.nn.conv2d(gray_images, filter=apply_edge_filter.filter[0], strides=[1,1,1,1], padding='SAME')
	edge_h = tf.nn.conv2d(gray_images, filter=apply_edge_filter.filter[1], strides=[1,1,1,1], padding='SAME')
	outputs = tf.square(edge_v) + tf.square(edge_h)

	return outputs


########################### encoding ###########################

def encode_batch_images(batch):
	"""
		input:
			batch:  n x H x W x C   input images batch
		output:
			packed: n x String      output PNG-encoded strings
	"""
	# output:
	unpacked = tf.unstack(batch)
	num = len(unpacked)
	encoded = [None] * num
	for k in range(num):
		encoded[k] = tf.image.encode_png(unpacked[k])
	return tf.stack(encoded)

def encode_raw_batch_images(batch):
	"""
		input:
			batch:  n x H x W x C   input raw images batch
		output:
			packed: n x String      output PNG-encoded strings
	"""
	return encode_batch_images(saturate_image(unnormalize_image(batch)))

def write_image(name, image):
	"""
		input:
			name:  String     file name
			image: String     PNG-encoded string
	"""
	path = os.path.dirname(name)
	if not os.path.exists(path):
		os.makedirs(path)
	file = open(name, 'wb')
	file.write(image)
	file.close()

