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
import math

import image
import view as vw

NUM_CORRESPONDENCES = 1024

def load_data(config, views, shape_list, shuffle=True, batch_size=-1):
	"""
		input:
			config               tf.app.flags        command line arguments
			views                vw.View             view points information
			shape_list           list of string      input shape name list
			shuffle              bool                whether input shape list should be shuffled
		output:
			name_batch           n     x string      shape names
			source_batch         n     x H x W x Ci  source images
			target_batch         (n*m) x H x W x Co  target images in m views
			mask_batch           (n*m) x H x W x 1   target boolean masks in m views
			angle_batch          (n*m) x 4           target viewing angle params in m views
			num_shapes           int                 number of loaded shapes
	"""

	if batch_size==-1:
		batch_size = config.batch_size

	# handle affix

	num_source_views = len(config.sketch_views)
	source_prefix_list = ['sketch/' for view in range(num_source_views)]
	source_interfix_list = ['/sketch-%c' % v for v in config.sketch_views]
	if config.test:
		sketch_variation = '0'
	else:
		sketch_variation_queue = tf.train.string_input_producer(['%d' % v for v in range(config.sketch_variations)], shuffle=True)
		sketch_variation = sketch_variation_queue.dequeue()
	source_suffix_list = ['-'+sketch_variation+'.png' for view in range(num_source_views)]

	num_dnfs_views = max(2, len(config.sketch_views))
	dnfs_prefix_list = ['dnfs/' for view in range(num_dnfs_views)]
	dnfs_interfix_list = ['/dnfs-%d' % config.image_size for view in range(num_dnfs_views)]
	dnfs_suffix_list = ['-%d.png' % view for view in range(num_dnfs_views)]

	num_dn_views = 12
	dn_prefix_list = ['dn/' for view in range(num_dn_views)]
	dn_interfix_list = ['/dn-%d' % config.image_size for view in range(num_dn_views)]
	dn_suffix_list = ['-%d.png' % view for view in range(num_dn_views)]

	num_target_views = num_dnfs_views + num_dn_views
	target_prefix_list = dnfs_prefix_list + dn_prefix_list
	target_interfix_list = dnfs_interfix_list + dn_interfix_list
	target_suffix_list = dnfs_suffix_list + dn_suffix_list
	num_target_views = views.num_views

	# build input queue

	if config.continuous_view and config.test:
		shape_list_queue = tf.train.input_producer([name for name in shape_list for view in range(num_target_views)], shuffle=False)
	else:
		shape_list_queue = tf.train.input_producer(shape_list, shuffle=shuffle)

	# load data from queue
	
	shape_name = shape_list_queue.dequeue()
	source_files = [config.data_dir+source_prefix_list[view]+shape_name+source_interfix_list[view]+source_suffix_list[view] for view in range(num_source_views)]
	if not config.continuous_view:
		target_files = [config.data_dir+target_prefix_list[view]+shape_name+target_interfix_list[view]+target_suffix_list[view] for view in range(num_target_views)]
		target_angles = tf.zeros([num_target_views, 4])
	else:
		angle_list = [vw.view2angle(view) for view in views.views]
		view_list_queue = tf.train.slice_input_producer([angle_list, target_prefix_list, target_interfix_list, target_suffix_list], shuffle=(not config.test))
		target_files = [config.data_dir+view_list_queue[1]+shape_name+view_list_queue[2]+view_list_queue[3]] # only one single image
		target_angles = [view_list_queue[0]]
	
	# decode source images
	
	source_images = [tf.image.decode_png(tf.read_file(file), channels=1, dtype=tf.uint8) for file in source_files]
	source_image = tf.concat(source_images, 2) # put multi-view images into different channels
	source_image = image.normalize_image(tf.slice(source_image, [0,0,0], [config.image_size, config.image_size, -1])) # just do a useless slicing to establish size
	source_image = tf.concat([source_image, tf.image.flip_left_right(source_image)], 2) # HACK: add horizontally flipped image as input

	# decode target images

	if not config.test:
		target_images = tf.stack([tf.image.decode_png(tf.read_file(file), channels=4, dtype=tf.uint16) for file in target_files])
		target_images = image.normalize_image(tf.slice(target_images, [0,0,0,0], [-1,config.image_size, config.image_size, -1]))
	else:
		target_images = tf.ones([len(target_files), config.image_size, config.image_size, 4]) # dummy target for testing
	target_masks = image.extract_boolean_mask(target_images)

	if config.predict_normal:
		# pre-process normal background
		target_shape = target_images.get_shape().as_list()
		target_background = tf.concat([tf.zeros(target_shape[:-1]+[2]), tf.ones(target_shape[:-1]+[2])], 3) # (0,0,1,1)
		target_images = tf.where(tf.tile(target_masks, [1,1,1,target_shape[3]]), target_images, target_background)
	else:
		# retain depth only
		target_images = tf.slice(target_images, [0,0,0,3], [-1,-1,-1,1])
	
	target_images = tf.concat([target_images, image.convert_to_real_mask(target_masks)], 3)

	# create prefetching tensor

	num_shapes = len(shape_list)
	min_queue_examples = max(1, int(num_shapes * 0.01))

	tensor_data = [shape_name, source_image, target_images, target_masks, target_angles]

	if shuffle:
		num_preprocess_threads = 12
		batch_data = tf.train.shuffle_batch(
			tensor_data,
			batch_size=batch_size,
			num_threads=num_preprocess_threads,
			capacity=min_queue_examples + 3 * batch_size,
			min_after_dequeue=min_queue_examples)
	else:
		num_preprocess_threads = 1
		batch_data = tf.train.batch(
			tensor_data,
			batch_size=batch_size,
			num_threads=num_preprocess_threads,
			capacity=min_queue_examples)

	name_batch = batch_data[0]
	source_batch = batch_data[1]
	target_batch =  batch_data[2]
	target_batch = tf.reshape(target_batch, [-1]+target_batch.get_shape().as_list()[2:])
	mask_batch =  batch_data[3]
	mask_batch = tf.reshape(mask_batch, [-1]+mask_batch.get_shape().as_list()[2:])
	angle_batch = batch_data[4]
	angle_batch = tf.reshape(angle_batch, [-1]+angle_batch.get_shape().as_list()[2:])

	#print('name: ', name_batch)
	#print('source: ', source_batch)
	#print('target: ', target_batch)
	#print('mask: ', mask_batch)
	#print('angle: ', angle_batch)

	return name_batch, source_batch, target_batch, mask_batch, angle_batch, num_shapes

def load_train_data(config, views, batch_size=-1):

	print("Loading training data...")

	shape_list_file = open(os.path.join(config.data_dir, 'train-list.txt'), 'r')
	shape_list = shape_list_file.read().splitlines()
	shape_list_file.close()

	return load_data(config, views, shape_list, shuffle=True, batch_size=batch_size)

def load_test_data(config, views, batch_size=-1):

	print("Loading testing data...")

	shape_list_file = open(os.path.join(config.data_dir, 'test-list.txt'), 'r')
	shape_list = shape_list_file.read().splitlines()
	shape_list_file.close()

	return load_data(config, views, shape_list, shuffle=False, batch_size=batch_size)

def load_encode_data(config, views, batch_size=-1):

	print("Loading encoding data...")

	shape_list_file = open(os.path.join(config.data_dir, 'list.txt'), 'r')
	shape_list = shape_list_file.read().splitlines()
	shape_list_file.close()

	return load_data(config, views, shape_list, shuffle=False, batch_size=batch_size)

def load_validate_data(config, views, batch_size=-1):

	print("Loading validation data...")

	shape_list_file = open(os.path.join(config.data_dir, 'validate-list.txt'), 'r')
	shape_list = shape_list_file.read().splitlines()
	shape_list_file.close()

	return load_data(config, views, shape_list, shuffle=False, batch_size=batch_size)

def write_bin_data(file_name, data):

	path = os.path.dirname(file_name)
	if not os.path.exists(path):
		os.makedirs(path)
	data.tofile(file_name)

def write_pfm_data(file_name, data):

	path = os.path.dirname(file_name)
	if not os.path.exists(path):
		os.makedirs(path)
	file = open(file_name, 'wb')
	
	if data.shape[2] == 1:
		file.write('Pf\n')
	elif data.shape[2] == 3:
		file.write('PF\n')
	else:
		raise ValueError('incorrect number of channels')

	file.write(('%d %d\n' % (data.shape[1], data.shape[0])))
	file.write('-1.0\n')

	data = np.flipud(data) # PFM format stores pixels from bottom to top...
	data.tofile(file)

	file.close()