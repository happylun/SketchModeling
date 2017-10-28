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

import time
import os

import data
import monnet as mn
import view as vw

FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_boolean('train', False,
							"""Flag for training routine.""")
tf.app.flags.DEFINE_boolean('test', False,
							"""Flag for testing routine.""")
tf.app.flags.DEFINE_boolean('encode', False,
							"""Flag for encoding routine.""")
tf.app.flags.DEFINE_boolean('predict_normal', True,
							"""Flag for predicting normal.""")
tf.app.flags.DEFINE_boolean('continuous_view', False,
							"""Flag for using continuous view architecture.""")
tf.app.flags.DEFINE_boolean('no_adversarial', False,
							"""Flag for adversarial loss term.""")
tf.app.flags.DEFINE_integer('batch_size', 2,
							"""Number of images to process in a batch.""")
tf.app.flags.DEFINE_integer('image_size', 256,
							"""Size of images to be learned.""")
tf.app.flags.DEFINE_integer('sketch_variations', 4,
							"""Number of variations on input source.""")
tf.app.flags.DEFINE_string('sketch_views', 'FS',
							"""Views used in sketch input ( [F]ront / [T]op / [S]ide )""")
tf.app.flags.DEFINE_float('max_epochs', 100.0,
							"""Maximum epochs for optimization.""")
tf.app.flags.DEFINE_float('gpu_fraction', 0.9,
							"""Upper-bound fraction of GPU memory usage.""")
tf.app.flags.DEFINE_string('data_dir', './../../../../Data/CharacterDraw/',
							"""Directory containing training/testing images.""")
tf.app.flags.DEFINE_string('train_dir', './../../../../Data/Checkpoint/Character/',
							"""Directory where to write training logs.""")
tf.app.flags.DEFINE_string('test_dir', './../../../../Data/CharacterDraw/output/',
							"""Directory where to write testing logs.""")
tf.app.flags.DEFINE_string('encode_dir', './../../../../Data/CharacterDraw/encode/',
							"""Directory where to write encoding logs.""")
tf.app.flags.DEFINE_string('view_file', 'view.off',
							"""File with view points information.""")

def main(argv=None):

	print('start running...')
	start_time = time.time()

	############################################ build graph ############################################

	monnet = mn.MonNet(FLAGS)

	if int(FLAGS.train) + int(FLAGS.test) + int(FLAGS.encode) != 1:
		print('please specify \'train\' or \'test\' or \'encode\'')
		return

	views = vw.Views(os.path.join(FLAGS.data_dir, 'view', FLAGS.view_file))

	if FLAGS.train:
		train_names, train_sources, train_targets, train_masks, train_angles, num_train_shapes = data.load_train_data(FLAGS, views)
		valid_names, valid_sources, valid_targets, valid_masks, valid_angles, num_valid_shapes = data.load_validate_data(FLAGS, views)

		with tf.variable_scope("monnet") as scope:
			monnet.build_network(\
				names=train_names,
				sources=train_sources,
				targets=train_targets,
				masks=train_masks,
				angles=train_angles,
				views=views,
				is_training=True)
			scope.reuse_variables() # sharing weights
			monnet.build_network(\
				names=valid_names,
				sources=valid_sources,
				targets=valid_targets,
				masks=valid_masks,
				angles=valid_angles,
				views=views,
				is_validation=True)
	elif FLAGS.test:
		test_names, test_sources, test_targets, test_masks, test_angles, num_test_shapes = data.load_test_data(FLAGS, views)

		with tf.variable_scope("monnet") as scope:
			monnet.build_network(\
				names=test_names,
				sources=test_sources,
				targets=test_targets,
				masks=test_masks,
				angles=test_angles,
				views=views,
				is_testing=True)
	elif FLAGS.encode:
		encode_names, encode_sources, encode_targets, encode_masks, encode_angles, num_encode_shapes = data.load_encode_data(FLAGS, views)

		with tf.variable_scope("monnet") as scope:
			monnet.build_network(\
				names=encode_names,
				sources=encode_sources,
				targets=encode_targets,
				masks=encode_masks,
				angles=encode_angles,
				views=views,
				is_encoding=True)


	############################################ compute graph ############################################

	gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=FLAGS.gpu_fraction)

	with tf.Session(config=tf.ConfigProto(gpu_options=gpu_options,
		log_device_placement=False,
		allow_soft_placement=True)) as sess:

		if FLAGS.train:
			monnet.train(sess, views, num_train_shapes, num_valid_shapes)
		elif FLAGS.test:
			monnet.test(sess, views, num_test_shapes)
		elif FLAGS.encode:
			monnet.encode(sess, views, num_encode_shapes)

		sess.close()

	duration = time.time() - start_time
	print('total running time: %.1f\n' % duration)


if __name__ == '__main__':
	tf.app.run()