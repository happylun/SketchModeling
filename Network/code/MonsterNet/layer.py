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
import tensorflow.contrib.framework as tf_framework

WEIGHT_STDDEV = 0.005
WEIGHT_DECAY = 0.0001
BN_DECAY = 0.997
BN_EPSILON = 1e-5

def leaky_relu(tensor, slope=0.2):
	"""
		input:
			tensor   : input tensor of any shape
		output:
			result   : output tensor having the same shape as input tensor
	"""
	return tf.maximum(tensor*slope, tensor)

def unet_scopes(bn_scope):

	bn_params = {
		'is_training': True,
		'decay': BN_DECAY,
		'epsilon': BN_EPSILON,
		'trainable': False,
		'updates_collections': bn_scope,
	}

	with tf_framework.arg_scope(
			[tf_layers.conv2d, tf_layers.fully_connected],
			weights_initializer=tf.truncated_normal_initializer(stddev=WEIGHT_STDDEV),
			weights_regularizer=tf_layers.l2_regularizer(WEIGHT_DECAY),
			biases_initializer=tf.zeros_initializer(),
			normalizer_fn=tf_layers.batch_norm,
			normalizer_params=bn_params,
			activation_fn=leaky_relu) as scope:
		if bn_scope is None:
			return scope
		else:
			with tf_framework.arg_scope([tf_layers.batch_norm], **bn_params) as scope_with_bn:
				return scope_with_bn

def cnet_scopes(bn_scope):

	with tf_framework.arg_scope(
			[tf_layers.conv2d, tf_layers.fully_connected],
			weights_initializer=tf.truncated_normal_initializer(stddev=WEIGHT_STDDEV),
			weights_regularizer=tf_layers.l2_regularizer(WEIGHT_DECAY),
			biases_initializer=tf.zeros_initializer(),
			normalizer_fn=None,
			activation_fn=leaky_relu) as scope:
		return scope

def residual_layer(inputs, kernel, scope):
	"""
		input:
			inputs            : n x H x W x C    feature maps to be passed into residual block
			kernel            : scalar           internal filter kernel size
			scope             : string           scope name
		output:
			outputs           : n x H x W x C    output feature maps
	"""

	channels = inputs.get_shape()[3].value
	layer1 = tf_layers.conv2d(inputs, num_outputs=channels, kernel_size=kernel, stride=1, scope=scope+'/layer1')
	layer2 = tf_layers.conv2d(layer1, num_outputs=channels, kernel_size=kernel, stride=1, scope=scope+'/layer2', activation_fn=None)
	outputs = layer2 + inputs
	return outputs

def unconv_layer(inputs, num_outputs, kernel_size, stride, scope, normalizer_fn=tf_layers.batch_norm, activation_fn=tf.nn.relu):
	"""
		input:
			inputs            : n x H x W x C    feature maps to be passed into unconv layer
			num_outputs       : scalar           number of channels in output feature map
			kernel_size       : scalar           internal filter kernel size
			scope             : string           scope name
			normalizer_fn     : function         normalizer function
			activation_fn     : function         activation function
		output:
			outputs           : n x H x W x C    output feature maps
	"""

	# return tf_layers.conv2d_transpose(inputs, num_outputs=num_outputs, kernel_size=kernel_size, stride=stride, scope=scope, normalizer_fn=normalizer_fn, activation_fn=activation_fn)

	h = inputs.get_shape()[1].value
	w = inputs.get_shape()[2].value
	c = inputs.get_shape()[3].value

	# upsampled = tf.image.resize_bilinear(inputs, [h*stride, w*stride])
	upsampled = tf.image.resize_nearest_neighbor(inputs, [h*stride, w*stride])

	outputs = tf_layers.conv2d(upsampled, num_outputs=num_outputs, kernel_size=kernel_size, stride=1, scope=scope, normalizer_fn=normalizer_fn, activation_fn=activation_fn)

	# features = tf_layers.conv2d(upsampled, num_outputs=c, kernel_size=kernel_size, stride=1, scope=scope+'/conv1')
	# outputs = tf_layers.conv2d(features, num_outputs=num_outputs, kernel_size=kernel_size, stride=1, scope=scope+'/conv2', normalizer_fn=normalizer_fn, activation_fn=activation_fn)

	return outputs
