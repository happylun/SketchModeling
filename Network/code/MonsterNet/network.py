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

import tensorflow.contrib.layers as tf_layers

import layer
import image

def generateUNet(images, num_output_views, num_output_channels):
	"""
		input:
			images               : n     x H x W x Ci   input images ( 256 x 256 x Ci )
			num_output_views     : int                  number of output views
			num_output_channels  : int                  number of output image channels
		output:
			results              : (n*m) x H x W x Co   output images ( 256 x 256 x Co )
			features             : n     x D            output features ( 512 )
	"""

	###### encoding ######

	e1 = tf_layers.conv2d(images, num_outputs= 64, kernel_size=4, stride=2, scope='e1', normalizer_fn=None) # 128 x 128 x  64
	e2 = tf_layers.conv2d(    e1, num_outputs=128, kernel_size=4, stride=2, scope='e2')                     #  64 x  64 x 128
	e3 = tf_layers.conv2d(    e2, num_outputs=256, kernel_size=4, stride=2, scope='e3')                     #  32 x  32 x 256
	e4 = tf_layers.conv2d(    e3, num_outputs=512, kernel_size=4, stride=2, scope='e4')                     #  16 x  16 x 512
	e5 = tf_layers.conv2d(    e4, num_outputs=512, kernel_size=4, stride=2, scope='e5')                     #   8 x   8 x 512
	e6 = tf_layers.conv2d(    e5, num_outputs=512, kernel_size=4, stride=2, scope='e6')                     #   4 x   4 x 512
	e7 = tf_layers.conv2d(    e6, num_outputs=512, kernel_size=4, stride=2, scope='e7')                     #   2 x   2 x 512

	num_batches = images.get_shape()[0].value
	features = tf.reshape(e7, [num_batches, -1]) # 2048

	###### decoding ######

	nc = num_output_channels
	rpv = [None] * num_output_views # results per view
	for view in range(num_output_views):

		with tf.variable_scope('decoder_%d' % view):
			d6 = tf_layers.dropout(layer.unconv_layer(                    e7, num_outputs=512, kernel_size=4, stride=2, scope='d6'))                    #   4 x   4 x 512
			d5 = tf_layers.dropout(layer.unconv_layer(tf.concat([d6, e6], 3), num_outputs=512, kernel_size=4, stride=2, scope='d5'))                    #   8 x   8 x 512
			d4 =                   layer.unconv_layer(tf.concat([d5, e5], 3), num_outputs=512, kernel_size=4, stride=2, scope='d4')                     #  16 x  16 x 512
			d3 =                   layer.unconv_layer(tf.concat([d4, e4], 3), num_outputs=256, kernel_size=4, stride=2, scope='d3')                     #  32 x  32 x 256
			d2 =                   layer.unconv_layer(tf.concat([d3, e3], 3), num_outputs=128, kernel_size=4, stride=2, scope='d2')                     #  64 x  64 x 128
			d1 =                   layer.unconv_layer(tf.concat([d2, e2], 3), num_outputs= 64, kernel_size=4, stride=2, scope='d1')                     # 128 x 128 x  64
			rpv[view] =            layer.unconv_layer(tf.concat([d1, e1], 3), num_outputs= nc, kernel_size=4, stride=2, scope='re', normalizer_fn=None, activation_fn=tf.tanh)

	height = images.get_shape()[1].value
	width = images.get_shape()[2].value
	results = tf.reshape(tf.transpose(tf.stack(rpv), [1,0,2,3,4]), [-1, height, width, nc])

	return results, features

def generateCNet(images, angles, num_output_channels):
	"""
		input:
			images               : n     x H x W x Ci   input images ( 256 x 256 x Ci )
			angles               : n     x 4            output viewing angle parameters
			num_output_channels  : int                  number of output image channels
		output:
			results              : n     x H x W x Co   output images ( 256 x 256 x Co )
			features             : n     x D            output features ( 512 )
	"""

	###### encoding ######

	e1 = tf_layers.conv2d(images, num_outputs= 64, kernel_size=4, stride=2, scope='e1', normalizer_fn=None) # 128 x 128 x  64
	e2 = tf_layers.conv2d(    e1, num_outputs=128, kernel_size=4, stride=2, scope='e2')                     #  64 x  64 x 128
	e3 = tf_layers.conv2d(    e2, num_outputs=256, kernel_size=4, stride=2, scope='e3')                     #  32 x  32 x 256
	e4 = tf_layers.conv2d(    e3, num_outputs=512, kernel_size=4, stride=2, scope='e4')                     #  16 x  16 x 512
	e5 = tf_layers.conv2d(    e4, num_outputs=512, kernel_size=4, stride=2, scope='e5')                     #   8 x   8 x 512
	e6 = tf_layers.conv2d(    e5, num_outputs=512, kernel_size=4, stride=2, scope='e6')                     #   4 x   4 x 512
	e7 = tf_layers.conv2d(    e6, num_outputs=512, kernel_size=4, stride=2, scope='e7')                     #   2 x   2 x 512

	num_batches = images.get_shape()[0].value
	ifeat = tf.reshape(e7, [num_batches, -1]) # 2048
	ifeat = tf_layers.fully_connected(ifeat, 2048, scope='ifc') # 2048
	features = ifeat

	vfeat = tf_layers.stack(
		angles,
		tf_layers.fully_connected,
		[64,            # 64
		64,             # 64
		64],            # 64
		scope='vfc')

	###### decoding ######

	nc = num_output_channels
	mp = 1 # multiplier for filter size (should be something close to the square root of number of output views)

	feat = tf_layers.stack(
		tf.concat([ifeat, vfeat], 1),
		tf_layers.fully_connected,
		[1024*mp,          # 1024*mp
		1024*mp,           # 1024*mp
		2048*mp],          # 2048*mp
		scope='fc')
	feat = tf.reshape(feat, [-1, 2, 2, 512*mp]) # 2 x 2 x 512*mp

	#d6 =      layer.unconv_layer(                  feat, num_outputs=512*mp, kernel_size=4, stride=2, scope='d6')               #   4 x   4 x 512*mp
	#d5 =      layer.unconv_layer(tf.concat([d6, e6], 3), num_outputs=512*mp, kernel_size=4, stride=2, scope='d5')               #   8 x   8 x 512*mp
	#d4 =      layer.unconv_layer(tf.concat([d5, e5], 3), num_outputs=512*mp, kernel_size=4, stride=2, scope='d4')               #  16 x  16 x 512*mp
	#d3 =      layer.unconv_layer(tf.concat([d4, e4], 3), num_outputs=256*mp, kernel_size=4, stride=2, scope='d3')               #  32 x  32 x 256*mp
	#d2 =      layer.unconv_layer(tf.concat([d3, e3], 3), num_outputs=128*mp, kernel_size=4, stride=2, scope='d2')               #  64 x  64 x 128*mp
	#d1 =      layer.unconv_layer(tf.concat([d2, e2], 3), num_outputs= 64*mp, kernel_size=4, stride=2, scope='d1')               # 128 x 128 x  64*mp
	#results = layer.unconv_layer(tf.concat([d1, e1], 3), num_outputs= nc,    kernel_size=4, stride=2, scope='re', normalizer_fn=None, activation_fn=tf.tanh)

	d6 =      layer.unconv_layer(feat, num_outputs=512*mp, kernel_size=4, stride=2, scope='d6')               #   4 x   4 x 512*mp
	d5 =      layer.unconv_layer(d6,   num_outputs=512*mp, kernel_size=4, stride=2, scope='d5')               #   8 x   8 x 512*mp
	d4 =      layer.unconv_layer(d5,   num_outputs=512*mp, kernel_size=4, stride=2, scope='d4')               #  16 x  16 x 512*mp
	d3 =      layer.unconv_layer(d4,   num_outputs=256*mp, kernel_size=4, stride=2, scope='d3')               #  32 x  32 x 256*mp
	d2 =      layer.unconv_layer(d3,   num_outputs=128*mp, kernel_size=4, stride=2, scope='d2')               #  64 x  64 x 128*mp
	d1 =      layer.unconv_layer(d2,   num_outputs= 64*mp, kernel_size=4, stride=2, scope='d1')               # 128 x 128 x  64*mp
	results = layer.unconv_layer(d1,   num_outputs= nc,    kernel_size=4, stride=2, scope='re', normalizer_fn=None, activation_fn=tf.tanh)

	return results, features

def discriminate(data):
	"""
		intput:
			data    : n x H x W x C     data to be discriminated ( 256 x 256 x C )
		output:
			probs   : n                 probabilities being real
	"""

	d1 = tf_layers.conv2d(data, num_outputs= 64, kernel_size=4, stride=2, scope='d1', normalizer_fn=None) # 128 x 128 x  64
	d2 = tf_layers.conv2d(d1,   num_outputs=128, kernel_size=4, stride=2, scope='d2')                     #  64 x  64 x 128
	d3 = tf_layers.conv2d(d2,   num_outputs=256, kernel_size=4, stride=2, scope='d3')                     #  32 x  32 x 256
	d4 = tf_layers.conv2d(d3,   num_outputs=512, kernel_size=4, stride=2, scope='d4')                     #  16 x  16 x 512
	d5 = tf_layers.conv2d(d4,   num_outputs=512, kernel_size=4, stride=2, scope='d5')                     #   8 x   8 x 512
	d6 = tf_layers.conv2d(d5,   num_outputs=512, kernel_size=4, stride=2, scope='d6')                     #   4 x   4 x 512
	d7 = tf_layers.conv2d(d6,   num_outputs=512, kernel_size=4, stride=2, scope='d7')                     #   2 x   2 x 512

	feature = tf.reshape(d7, [-1, 2048]) # 2048
	probs = tf_layers.fully_connected(feature, 1, scope='fc', normalizer_fn=None, activation_fn=tf.sigmoid) # 1
	probs = tf.reshape(probs, [-1])

	return probs