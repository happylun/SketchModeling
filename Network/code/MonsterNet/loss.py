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

import image
import reproject as rp
import view as vw

def compute_depth_loss(predicts, targets, mask, normalized=True):
	"""
		input:
			predicts   : n x H x W x 1      predicted depths
			targets    : n x H x W x 1      ground-truth depths
			mask       : n x H x W x 1      boolean mask
			normalized : boolean            whether output loss should be normalized by pixel number
		output:
			loss       : scalar             loss value
	"""

	num_batches = predicts.get_shape()[0].value
	num_channels = predicts.get_shape()[3].value

	diff = tf.abs(predicts-targets) # L-1 loss
	# diff = tf.square(predicts-targets) # L-2 loss
	diff = tf.boolean_mask(diff, tf.squeeze(mask, [3]))
	if normalized:
		depth_loss = tf.reduce_mean(diff) * (num_batches*num_channels)
	else:
		depth_loss = tf.reduce_sum(diff)

	return depth_loss

def compute_normal_loss(predicts, targets, mask, normalized=True):
	"""
		input:
			predicts   : n x H x W x 3      predicted normals
			targets    : n x H x W x 3      ground-truth normals
			mask       : n x H x W x 1      boolean mask
			normalized : boolean            whether output loss should be normalized by pixel number
		output:
			loss       : scalar             loss value
	"""

	num_batches = predicts.get_shape()[0].value
	num_channels = predicts.get_shape()[3].value

	# with unit length 1-n_1*n_2 = 0.5*||n_1-n_2||^2
	diff = tf.square(predicts-targets)
	diff = tf.boolean_mask(diff, tf.squeeze(mask, [3]))
	if normalized:
		normal_loss = tf.reduce_mean(diff) * (num_batches*num_channels)
	else:
		normal_loss = tf.reduce_sum(diff)

	return normal_loss

def compute_mask_loss(predicts, targets, normalized=True):
	"""
		input:
			predicts   : n x H x W x C      generated masks (-1: false, 1: true)
			targets    : n x H x W x C      ground-truth masks (-1: false, 1: true)
			normalized : boolean            whether output loss should be normalized by pixel number
		output:
			loss       : scalar             loss value
	"""
	
	p = predicts * 0.5 + 0.5 # convert to probability
	z = targets * 0.5 + 0.5
	# L = -z*log(p)-(1-z)*log(1-p)
	mask_loss = tf.reduce_sum(-tf.multiply(tf.log(tf.maximum(1e-6, p)), z)-tf.multiply(tf.log(tf.maximum(1e-6, 1-p)), 1-z))

	if normalized:
		mask_shape = predicts.get_shape().as_list()
		num_pixels = np.prod(mask_shape[1:])
		mask_loss /= num_pixels

	return mask_loss

def compute_pixel_loss(predicts, targets, normalized=True):
	"""
		input:
			predicts   : n x H x W x C      predicted images
			targets    : n x H x W x C      ground-truth images
			normalized : boolean            whether output loss should be normalized by pixel number
		output:
			loss       : scalar             loss value
	"""

	num_batches = predicts.get_shape()[0].value
	num_channels = predicts.get_shape()[3].value

	diff = tf.abs(predicts-targets) # L-1 loss
	# diff = tf.square(predicts-targets) # L-2 loss
	if normalized:
		pixel_loss = tf.reduce_mean(diff) * (num_batches*num_channels)
	else:
		pixel_loss = tf.reduce_sum(diff)

	return pixel_loss

def compute_consist_loss(contents, normalized=True):
	"""
		input:
			contents   : n x H x W x 4      normal/depth maps (nx, ny, nz, d)
			normalized : boolean            whether output loss should be normalized by pixel number
		output:
			loss       : scalar             loss value
	"""

	# Lx = | kappa * nx + dZdx * nz |
	# Ly = | kappa * ny + dZdy * nz |

	shape = contents.get_shape().as_list()
	num_batches = shape[0]
	H = shape[1]
	W = shape[2]
	kappaX = 5.0 / H # NOTE: view radius = 2.5
	kappaY = 5.0 / W

	filter_x = tf.convert_to_tensor(np.array([\
		[1.0, 0.0, -1.0],
		[4.0, 0.0, -4.0],
		[1.0, 0.0, -1.0]]), dtype=tf.float32)
	filter_y = tf.convert_to_tensor(np.array([\
		[-1.0, -4.0, -1.0],
		[0.0, 0.0, 0.0],
		[1.0, 4.0, 1.0]]), dtype=tf.float32)
	filter_x = tf.expand_dims(tf.expand_dims(filter_x, -1), -1)
	filter_y = tf.expand_dims(tf.expand_dims(filter_y, -1), -1)

	nx, ny, nz, d = tf.split(contents, 4, axis=3)

	dZdx = tf.nn.conv2d(d, filter=filter_x, strides=[1,1,1,1], padding='SAME')
	dZdy = tf.nn.conv2d(d, filter=filter_y, strides=[1,1,1,1], padding='SAME')

	Lx = tf.abs(tf.scalar_mul(kappaX, nx) + tf.multiply(dZdx, nz))
	Ly = tf.abs(tf.scalar_mul(kappaY, ny) + tf.multiply(dZdy, nz))

	if normalized:
		consist_loss = (tf.reduce_mean(Lx)+tf.reduce_mean(Ly)) * num_batches
	else:
		consist_loss = tf.reduce_sum(Lx)+tf.reduce_sum(Ly)

	return consist_loss


def compute_corres_geom_loss(predicts, corres, views):
	"""
		input:
			predicts  : (n*v) x H x W x 4       predicted images
			corres    : n     x G x M x v       correspondence point indices (G groups of M correspondences across v span views)
			views     : vw.Views                view points data
		output:
			loss      : scalar                  loss value
	"""

	if views.num_edges == 0:
		return 0

	position_factor = 1.0
	direction_factor = 1.0

	shape = predicts.get_shape().as_list()
	H = shape[1]
	W = shape[2]
	num_batches = shape[0] / views.num_views
	num_samples = corres.get_shape()[2].value

	points, dirs = rp.transform_tensor(predicts, views.views) # (n*V) x H x W x 3

	batch_points = tf.unpack(tf.reshape(points, [-1,views.num_views,H,W,3])) # [V x H x W x 3] * n
	batch_dirs = tf.unpack(tf.reshape(dirs, [-1,views.num_views,H,W,3])) # [V x H x W x 3] * n
	batch_corres = tf.unpack(corres) # [G x M x v] * n

	batch_losses = [None] * num_batches
	for batch_id in range(num_batches):
		all_points = tf.reshape(batch_points[batch_id], [-1,3]) # (V*H*W) x 3
		all_dirs = tf.reshape(batch_dirs[batch_id], [-1,3]) # (V*H*W) x 3
		all_corres = tf.reshape(batch_corres[batch_id], [-1]) # (G*M*v)
		slice_points = tf.reshape(tf.gather(all_points, all_corres), [views.num_edges,-1,views.edge_size,3]) # G x M x v x 3
		slice_dirs = tf.reshape(tf.gather(all_dirs, all_corres), [views.num_edges,num_samples,views.edge_size,3]) # G x M x v x 3

		# compute position loss as variance of reprojected point positions across nearby views
		normalized_points = slice_points - tf.tile(tf.reduce_mean(slice_points, reduction_indices=2, keep_dims=True), [1,1,views.edge_size,1])  # G x M x v x 3
		position_loss = tf.reduce_mean(tf.multiply(normalized_points, normalized_points))*3.0

		# compute direction loss as mean(1-dot(n,n)) for all pairs of reprojected directions across nearby views
		lensq_dirs = tf.maximum(tf.reduce_sum(tf.multiply(slice_dirs, slice_dirs), reduction_indices=3, keep_dims=True), 1e-3)
		normalized_dirs = tf.multiply(slice_dirs, tf.tile(tf.rsqrt(lensq_dirs), (1,1,1,3)))
		transposed = tf.reshape(tf.transpose(normalized_dirs, [2,0,1,3]), [views.edge_size, -1]) # V x (G*M*3)
		direction_loss = 1.0 - tf.reduce_mean(tf.matmul(transposed, transposed, transpose_b=True))*(1.0/(views.num_edges*num_samples))

		batch_losses[batch_id] = position_factor*position_loss + direction_factor*direction_loss

	loss = tf.reduce_sum(tf.stack(batch_losses))

	return loss

def compute_corres_mask_loss(predicts, corres, views):
	"""
		input:
			predicts  : (n*v) x H x W x 1       predicted masks
			corres    : n     x G x M x v       correspondence point indices (G groups of M correspondences across v span views)
			views     : vw.Views                view points data
		output:
			loss      : scalar                  loss value
	"""

	if views.num_edges == 0:
		return 0

	shape = predicts.get_shape().as_list()
	H = shape[1]
	W = shape[2]
	num_batches = shape[0] / views.num_views
	num_samples = corres.get_shape()[2].value

	probs = predicts*0.5+0.5 # [-1,1] => [0,1]

	batch_probs = tf.unpack(tf.reshape(probs, [-1,views.num_views,H,W,1])) # [V x H x W x 1] * n
	batch_corres = tf.unpack(corres) # [G x M x v] * n

	batch_losses = [None] * num_batches
	for batch_id in range(num_batches):
		all_probs = tf.reshape(batch_probs[batch_id], [-1,1]) # (V*H*W) x 1
		all_corres = tf.reshape(batch_corres[batch_id], [-1]) # (G*M*v)
		slice_probs = tf.reshape(tf.gather(all_probs, all_corres), [views.num_edges,-1,views.edge_size,1]) # G x M x v x 1

		# compute mask loss as Jensen-Shannon divergence of predicted mask probabilities across nearby views
		mask_loss = tf.reduce_mean( compute_entropy(tf.reduce_mean(slice_probs, reduction_indices=1)) - tf.reduce_mean(compute_entropy(slice_probs), reduction_indices=1) )

		batch_losses[batch_id] = mask_loss

	loss = tf.reduce_sum(tf.pack(batch_losses))

	return loss

def compute_entropy(tensor):
	"""
		input:
			tensor  : any shape tensor
		output:
			entropy : tensor having the same shape with input tensor
	"""

	entropy = - tf.multiply(tensor, tf.log(tensor+1e-6)) - tf.multiply(1.0-tensor, tf.log(1.0-tensor+1e-6))
	return entropy