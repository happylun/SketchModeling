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

import os

class ReProj(object):

	def __init__(self):
		self.proj = np.identity(4)
		self.view = np.identity(4)

	def set_ortho_projection(self, l=-2.5, r=2.5, b=-2.5, t=2.5, n=0.1, f=5.0):
		"""
			args:
				l: left
				r: right
				b: bottom
				t: top
				n: near
				f: far
			ref: https://www.opengl.org/sdk/docs/man2/xhtml/glOrtho.xml
		"""
		self.proj = np.array([ \
			[2.0/(r-l),  0.0,        0.0,         -(r+l)/(r-l)],
			[0.0,        2.0/(t-b),  0.0,         -(t+b)/(t-b)],
			[0.0,        0.0,        -2.0/(f-n),  -(f+n)/(f-n)],
			[0.0,        0.0,        0.0,         1.0         ]])
		self.proj_inv = np.linalg.inv(self.proj)

	def set_viewpoint(self, viewpoint):
		"""
			args:
				viewpoint: eye position (assuming center at origin, up on Y axis)
			ref: http://www.ibm.com/support/knowledgecenter/ssw_aix_53/com.ibm.aix.opengl/doc/openglrf/gluLookAt.htm
		"""
		E = viewpoint
		C = np.array([0.0, 0.0, 0.0])
		U = np.array([0.0, 1.0, 0.0])
		L = C-E;
		L = L/np.linalg.norm(L)
		S = np.cross(L, U)
		if np.linalg.norm(S) == 0:
			U = np.array([0.0, 0.0, -1.0])
			S = np.cross(L, U)
		S = S/np.linalg.norm(S)
		Up = np.cross(S, L)
		R = np.identity(4)
		R[0, 0:3] = S
		R[1, 0:3] = Up
		R[2, 0:3] = -L
		T = np.identity(4)
		T[0:3, 3] = -E
		self.view = np.dot(R, T)
		self.view_inv = np.linalg.inv(self.view)

	def transform(self, depth):
		"""
			input:
				depth:  H x   W    depth map with value range [-1, 1]
			output:
				points: (HxW) x 3  point set
		"""
		H = depth.shape[0]
		W = depth.shape[1]
		num_points = np.count_nonzero(depth<1.0)
		valid_points = [None] * num_points
		point_id = 0
		for u in range(W):
			for v in range(H):
				if depth[v,u] < 1.0:
					valid_points[point_id] = [(u*2.0+1.0-W)/W, (H-v*2.0-1.0)/H, depth[v,u], 1.0]
					point_id += 1
		if num_points<=0:
			valid_points = np.empty([0,4])
		points = np.dot(self.view_inv, np.dot(self.proj_inv, np.array(valid_points).T))[0:3,:].T
		return points

def export_ply(filename, points, normals=None):
	"""
		args:
			filename: string     file name
			points:   (HxW) x 3  point set
			normals:  (HxW) x 3  point set
	"""
	path = os.path.dirname(filename)
	if not os.path.exists(path):
		os.makedirs(path)
	f = open(filename, 'w')
	f.write('ply\n')
	f.write('format ascii 1.0\n')
	f.write('element vertex %d\n' % points.shape[0])
	f.write('property float x\n')
	f.write('property float y\n')
	f.write('property float z\n')
	if normals is not None:
		f.write('property float nx\n')
		f.write('property float ny\n')
		f.write('property float nz\n')
	f.write('end_header\n')
	for k in range(points.shape[0]):
		f.write('%f %f %f\n' % (points[k,0], points[k,1], points[k,2]))
		if normals is not None:
			f.write('%f %f %f\n' % (normals[k,0], normals[k,1], normals[k,2]))
	f.close()

def transform_tensor(predicts, views):
	"""
		input:
			predicts  : (n*V) x H x W x 4    predicted tensor (in n batches & V views)
			views     : V x 3                view point positions (numpy array)
		output:
			points    : (n*V) x H x W x 3    re-projected points position tensor
			dirs      : (n*V) x H x W x 3    re-projected normals direction tensor
	"""

	shape = predicts.get_shape().as_list()
	num_views = views.shape[0]
	num_batches = shape[0] / num_views

	# calculate reprojection matrix

	reproj = ReProj()
	reproj.set_ortho_projection()

	xform_per_view = [None] * num_views
	rotate_per_view = [None] * num_views
	for view_id in range(num_views):
		reproj.set_viewpoint(views[view_id,:])
		xform_per_view[view_id] = tf.constant(np.dot(reproj.view_inv, reproj.proj_inv), dtype=tf.float32) # [4 x 4] * V
		rotate_per_view[view_id] = tf.constant(reproj.view_inv, dtype=tf.float32) # [4 x 4] * V

	# separate depth/normal by views

	predicts_per_view = tf.transpose(tf.reshape(predicts, [-1, num_views, shape[1], shape[2], shape[3]]), [1, 0, 2, 3, 4]) # V x n x H x W x 4
	depths_per_view = tf.unstack(tf.slice(predicts_per_view, [0,0,0,0,3], [-1,-1,-1,-1,1]))  # [n x H x W x 1] * V
	normals_per_view = tf.unstack(tf.slice(predicts_per_view, [0,0,0,0,0], [-1,-1,-1,-1,3])) # [n x H x W x 3] * V

	# calculate projected coordinates

	H = shape[1]
	W = shape[2]
	vec_u = tf.constant([(u*2.0+1.0-W)/W for u in range(W)]) # W
	vec_v = tf.constant([(H-v*2.0-1.0)/H for v in range(H)]) # H
	mat_u = tf.tile(tf.reshape(vec_u, [1,1,-1,1]), (num_batches,H,1,1)) # n x H x W x 1
	mat_v = tf.tile(tf.reshape(vec_v, [1,-1,1,1]), (num_batches,1,W,1)) # n x H x W x 1
	mat_w = tf.ones([num_batches, H, W, 1])

	homo_points_per_view = [tf.concat([mat_u, mat_v, mat_d, mat_w], 3) for mat_d in depths_per_view] # [n x H x W x 4] * V
	homo_dirs_per_view = [tf.concat([mat_n, mat_w], 3) for mat_n in normals_per_view] # [n x H x W x 4] * V

	# transform points

	points_per_view = [None] * num_views
	dirs_per_view = [None] * num_views
	for view_id in range(num_views):
		xformed = tf.matmul(tf.reshape(homo_points_per_view[view_id], [-1,4]), xform_per_view[view_id], transpose_b=True) # (n*H*W) x 4
		rotated = tf.matmul(tf.reshape(homo_dirs_per_view[view_id], [-1,4]), rotate_per_view[view_id], transpose_b=True) # (n*H*W) x 4
		points_per_view[view_id] = tf.slice(tf.reshape(xformed, [-1,H,W,4]), [0,0,0,0], [-1,-1,-1,3]) # n x H x W x 3
		dirs_per_view[view_id] = tf.slice(tf.reshape(rotated, [-1,H,W,4]), [0,0,0,0], [-1,-1,-1,3]) # n x H x W x 3

	# organize output points

	points = tf.transpose(tf.stack(points_per_view), [1,0,2,3,4]) # n x v x H x W x 3
	points = tf.reshape(points, [-1, H, W, 3]) # (n*V) x H x W x 3

	dirs = tf.transpose(tf.stack(dirs_per_view), [1,0,2,3,4]) # n x v x H x W x 3
	dirs = tf.reshape(dirs, [-1, H, W, 3]) # (n*V) x H x W x 3

	return points, dirs