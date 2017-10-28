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

import tensorflow.contrib.framework as tf_framework

import time
import os
import math

import data
import image
import network
import layer
import loss
import reproject as rp
import view as vw

class MonNet(object):

	def __init__(self, config):
		self.config = config

	def build_network(self, names, sources, targets, masks, angles, views, is_training=False, is_validation=False, is_testing=False, is_encoding=False):
		"""
			input:
				names         :  n     x String             shape names
				sources       :  n     x H x W x C          source images
				targets       :  (n*m) x H x W x C          target images in m views (ground-truth)
				masks         :  (n*m) x H x W x 1          target boolean masks in m views (ground-truth)
				angles        :  (n*m) x 4                  viewing angle parameters (m=1 for continuous view prediction)
				views         :  vw.Views                   view points information
				is_training   :  boolean                    whether it is in training routine
				is_validation :  boolean                    whether it is handling validation data set
				is_testing    :  boolean                    whether it is in testing routine
				is_encoding   :  boolean                    whether it is encoding input
		"""

		print('Building network...')

		source_size = sources.get_shape().as_list()
		if self.config.continuous_view:
			num_output_views = 1
		else:
			num_output_views = views.num_views

		# scope names

		var_scope_G = 'G_net'
		var_scope_D = 'D_net'
		bn_scope_G = 'G_bn'
		bn_scope_D = 'D_bn'
		train_summary_G_name = 'train_summary_G'
		train_summary_D_name = 'train_summary_D'
		valid_summary_name = 'valid_summary'

		# generator

		num_channels = targets.get_shape()[3].value
		if not self.config.continuous_view:
			with tf.variable_scope(var_scope_G):
				with tf_framework.arg_scope(layer.unet_scopes(bn_scope_G)):
					preds, features = network.generateUNet(sources, num_output_views, num_channels) # (n*m) x H x W x C ; n x D
		else:
			with tf.variable_scope(var_scope_G):
				with tf_framework.arg_scope(layer.cnet_scopes(bn_scope_G)):
					preds, features = network.generateCNet(sources, angles, num_channels) # n x H x W x C ; n x D

		if is_encoding:
			self.encode_names = names
			self.encode_features = features
			return # all stuffs below are irrelevant to encoding pass

		# extract prediction contents

		preds_content = tf.slice(preds, [0,0,0,0], [-1,-1,-1,num_channels-1])
		preds_mask = tf.slice(preds, [0,0,0,num_channels-1], [-1,-1,-1,1])
		preds = image.apply_mask(preds_content, preds_mask)
		targets_content = tf.slice(targets, [0,0,0,0], [-1,-1,-1,num_channels-1])
		targets_mask = tf.slice(targets, [0,0,0,num_channels-1], [-1,-1,-1,1])
		targets = image.apply_mask(targets_content, targets_mask)
		if self.config.predict_normal:
			preds_normal = tf.slice(preds_content, [0,0,0,0], [-1,-1,-1,3])
			preds_depth = tf.slice(preds_content, [0,0,0,3], [-1,-1,-1,1])
			targets_normal = tf.slice(targets_content, [0,0,0,0], [-1,-1,-1,3])
			targets_depth = tf.slice(targets_content, [0,0,0,3], [-1,-1,-1,1])
		else:
			preds_depth = preds_content
			preds_normal = tf.tile(tf.zeros_like(preds_depth), [1,1,1,3])
			targets_depth = targets_content
			targets_normal = tf.tile(tf.zeros_like(targets_depth), [1,1,1,3])

		# expand tensors
		
		sources_expanded = tf.reshape(tf.tile(sources, [1,num_output_views,1,1]),[-1,source_size[1],source_size[2],source_size[3]]) # (n*m) x H x W x C

		names_expanded = tf.reshape(tf.tile(tf.expand_dims(names,1),[1,num_output_views]),[-1])
		names_suffix = ["--%d" % view for batch in range(source_size[0]) for view in range(num_output_views)]
		names_expanded = tf.reduce_join([names_expanded, names_suffix], 0)
		self.names = names_expanded

		# discriminator

		if not self.config.no_adversarial:
			with tf.variable_scope(var_scope_D):
				with tf_framework.arg_scope(layer.unet_scopes(bn_scope_D)):
					disc_data = tf.concat([targets, preds], 0)
					disc_data = tf.concat([tf.concat([sources_expanded, sources_expanded], 0), disc_data], 3) # HACK: insert input data for discrimination in UNet
					probs = network.discriminate(disc_data) # (n*m*2)

		# losses

		# NOTE: learning hyper-parameters
		lambda_p = 1.0    # image loss
		lambda_a = 0.01   # adversarial loss

		dl = loss.compute_depth_loss(preds_depth, targets_depth, masks)
		nl = loss.compute_normal_loss(preds_normal, targets_normal, masks)
		ml = loss.compute_mask_loss(preds_mask, targets_mask)
		loss_g_p = dl + nl + ml

		if self.config.no_adversarial:
			loss_g_a = 0.0
			loss_d_r = 0.0
			loss_d_f = 0.0
		else:
			probs_targets, probs_preds = tf.split(probs, 2, axis=0) # (n*m)
			loss_g_a = tf.reduce_sum(-tf.log(tf.maximum(probs_preds, 1e-6)))
			loss_d_r = tf.reduce_sum(-tf.log(tf.maximum(probs_targets, 1e-6)))
			loss_d_f = tf.reduce_sum(-tf.log(tf.maximum(1.0-probs_preds, 1e-6)))

		loss_G = loss_g_p * lambda_p + loss_g_a * lambda_a
		loss_D = loss_d_r + loss_d_f

		if is_validation:
			self.valid_losses = tf.stack([loss_G, loss_g_p, loss_g_a, loss_D, loss_d_r, loss_d_f])
			self.valid_images = tf.stack([
				image.encode_raw_batch_images(preds),
				image.encode_raw_batch_images(targets),
				image.encode_raw_batch_images(preds_normal),
				image.encode_raw_batch_images(preds_depth),
				image.encode_raw_batch_images(preds_mask)])
			self.valid_summary_losses = tf.placeholder(tf.float32, shape=self.valid_losses.get_shape())
			vG_all, vG_p, vG_a, vD_all, vD_r, vD_f = tf.unstack(self.valid_summary_losses)
			tf.summary.scalar('vG_all', vG_all, collections=[valid_summary_name])
			tf.summary.scalar('vG_p',   vG_p,   collections=[valid_summary_name])
			tf.summary.scalar('vG_a',   vG_a,   collections=[valid_summary_name])
			tf.summary.scalar('vD_all', vD_all, collections=[valid_summary_name])
			tf.summary.scalar('vD_r',   vD_r,   collections=[valid_summary_name])
			tf.summary.scalar('vD_f',   vD_f,   collections=[valid_summary_name])
			self.valid_summary_op = tf.summary.merge_all(valid_summary_name)
			return # all stuffs below are irrelevant to validation pass

		self.train_losses_G = tf.stack([loss_G, loss_g_p, loss_g_a])
		self.train_losses_D = tf.stack([loss_D, loss_d_r, loss_d_f])
		tf.summary.scalar('G_all', loss_G,   collections=[train_summary_G_name])
		tf.summary.scalar('G_p',   loss_g_p, collections=[train_summary_G_name])
		tf.summary.scalar('G_a',   loss_g_a, collections=[train_summary_G_name])
		tf.summary.scalar('D_all', loss_D,   collections=[train_summary_D_name])
		tf.summary.scalar('D_r',   loss_d_r, collections=[train_summary_D_name])
		tf.summary.scalar('D_f',   loss_d_f, collections=[train_summary_D_name])

		# statistics on variables

		all_vars = tf.trainable_variables()
		all_vars_G = [var for var in all_vars if var_scope_G in var.name]
		all_vars_D = [var for var in all_vars if var_scope_D in var.name]
		#print('Num all vars: %d' % len(all_vars))
		#print('Num vars on G net: %d' % len(all_vars_G))
		#print('Num vars on D net: %d' % len(all_vars_D))
		num_params_G = 0
		num_params_D = 0
		# print('G vars:')
		for var in all_vars_G:
			num_params_G += np.prod(var.get_shape().as_list())
			# print(var.name, var.get_shape().as_list())
		# print('D vars:')
		for var in all_vars_D:
			num_params_D += np.prod(var.get_shape().as_list())
			# print(var.name, var.get_shape().as_list())
		#print('Num all params: %d + %d = %d' % (num_params_G, num_params_D, num_params_G+num_params_D))
		#input('pause')

		# optimization

		# NOTE: learning hyper-parameters
		init_learning_rate = 0.0001
		adam_beta1 = 0.9
		adam_beta2 = 0.999
		opt_step = tf.Variable(0, trainable=False)
		learning_rate = tf.train.exponential_decay(init_learning_rate, global_step=opt_step, decay_steps=10000, decay_rate=0.96, staircase=True)
		
		opt_G = tf.train.AdamOptimizer(learning_rate=learning_rate, beta1=adam_beta1, beta2=adam_beta2, name='ADAM_G')
		opt_D = tf.train.AdamOptimizer(learning_rate=learning_rate, beta1=adam_beta1, beta2=adam_beta2, name='ADAM_D')
		# opt_G = tf.train.GradientDescentOptimizer(learning_rate=learning_rate, name='SGD_G')
		# opt_D = tf.train.GradientDescentOptimizer(learning_rate=learning_rate, name='SGD_D')

		grad_G = opt_G.compute_gradients(loss_G, var_list=all_vars_G, colocate_gradients_with_ops=True)
		self.grad_G_placeholder = [(tf.placeholder(tf.float32, shape=grad[1].get_shape()), grad[1]) for grad in grad_G if grad[0] is not None]
		self.grad_G_list = [grad[0] for grad in grad_G if grad[0] is not None]
		self.update_G_op = opt_G.apply_gradients(self.grad_G_placeholder, global_step=opt_step) # only update opt_step in G net

		if not self.config.no_adversarial:
			grad_D = opt_D.compute_gradients(loss_D, var_list=all_vars_D, colocate_gradients_with_ops=True)
			self.grad_D_placeholder = [(tf.placeholder(tf.float32, shape=grad[1].get_shape()), grad[1]) for grad in grad_D if grad[0] is not None]
			self.grad_D_list = [grad[0] for grad in grad_D if grad[0] is not None]
			self.update_D_op = opt_D.apply_gradients(self.grad_D_placeholder)

		# visualization stuffs

		sources_original, sources_flipped = tf.split(sources_expanded, 2, axis=3)
		if len(self.config.sketch_views) == 1: # single input
			sources_front = sources_original
			sources_side = tf.ones_like(sources_front) # fake side sketch
			sources_top = tf.ones_like(sources_front) # fake top sketch
		elif len(self.config.sketch_views) == 2: # double input
			sources_front, sources_side = tf.split(sources_original, 2, axis=3)
			sources_top = tf.ones_like(sources_front) # fake top sketch
		elif len(self.config.sketch_views) == 3: # triple input
			sources_front, sources_side, sources_top = tf.split(sources_original, 3, axis=3)
		if sources_front.get_shape()[3].value == 1 and targets.get_shape()[3].value == 4:
			alpha_front = tf.ones_like(sources_front)
			alpha_side = tf.ones_like(sources_side)
			alpha_top = tf.ones_like(sources_top)
			rgb_front = image.convert_to_rgb(sources_front, channels=3)
			rgb_side = image.convert_to_rgb(sources_side, channels=3)
			rgb_top = image.convert_to_rgb(sources_top, channels=3)
			sources_front = tf.concat([rgb_front, alpha_front], 3)
			sources_side = tf.concat([rgb_side, alpha_side], 3)
			sources_top = tf.concat([rgb_top, alpha_top], 3)

		input_row = tf.concat([sources_front, sources_side], 2)
		output_row = tf.concat([targets, preds], 2)

		result_tile = tf.concat([input_row, output_row], 1)
		result_tile = image.saturate_image(image.unnormalize_image(result_tile))

		tf.summary.image('result', result_tile, 12, [train_summary_G_name])

		self.train_summary_G_op = tf.summary.merge_all(train_summary_G_name)
		self.train_summary_D_op = tf.summary.merge_all(train_summary_D_name)

		# output images

		num_sketch_views = len(self.config.sketch_views)
		if num_sketch_views==1:
			all_input_row = sources_front
		elif num_sketch_views==2:
			all_input_row = tf.concat([sources_front, sources_side], 2)
		elif num_sketch_views==3:
			all_input_row = tf.concat([sources_front, sources_side, sources_top], 2)
		img_input = image.saturate_image(image.unnormalize_image(all_input_row, maxval=65535.0), dtype=tf.uint16)
		img_gt = image.saturate_image(image.unnormalize_image(targets, maxval=65535.0), dtype=tf.uint16)
		img_output = image.saturate_image(image.unnormalize_image(preds, maxval=65535.0), dtype=tf.uint16)
		png_input = image.encode_batch_images(img_input)
		png_gt = image.encode_batch_images(img_gt)
		png_output = image.encode_batch_images(img_output)

		img_normal = image.saturate_image(image.unnormalize_image(preds_normal, maxval=65535.0), dtype=tf.uint16)
		img_depth = image.saturate_image(image.unnormalize_image(preds_depth, maxval=65535.0), dtype=tf.uint16)
		img_mask = image.saturate_image(image.unnormalize_image(preds_mask, maxval=65535.0), dtype=tf.uint16)
		png_normal = image.encode_batch_images(img_normal)
		png_depth = image.encode_batch_images(img_depth)
		png_mask = image.encode_batch_images(img_mask)
		self.pngs = tf.stack([png_input, png_gt, png_output, png_normal, png_depth, png_mask])

		# output results

		pixel_shape = preds.get_shape().as_list()
		num_pixels = np.prod(pixel_shape[1:])
		self.errors = tf.reduce_sum(tf.abs(preds-targets), [1,2,3]) / num_pixels # just a quick check
		self.results = preds

		# batch normalization

		bn_G_collection = tf.get_collection(bn_scope_G)
		bn_D_collection = tf.get_collection(bn_scope_D)
		self.bn_G_op = tf.group(*bn_G_collection)
		self.bn_D_op = tf.group(*bn_D_collection)

	def train(self, sess, views, num_train_shapes, num_valid_shapes):

		print('Training...')

		ckpt = tf.train.get_checkpoint_state(self.config.train_dir)
		init_op = tf.global_variables_initializer()
		sess.run(init_op)
		if ckpt and ckpt.model_checkpoint_path:
			self.saver = tf.train.Saver(keep_checkpoint_every_n_hours=10.0, max_to_keep=2)
			self.saver.restore(sess, ckpt.model_checkpoint_path)
			try:
				self.step = int(ckpt.model_checkpoint_path.split('/')[-1].split('-')[-1])
			except ValueError:
				self.step = 0
		else:
			self.saver = tf.train.Saver(tf.global_variables(), keep_checkpoint_every_n_hours=10.0, max_to_keep=2)
			self.step = 0

		coord = tf.train.Coordinator()
		threads = tf.train.start_queue_runners(sess=sess, coord=coord)
		self.summarizer = tf.summary.FileWriter(self.config.train_dir, sess.graph)

		print_interval = 40 // self.config.batch_size # steps
		update_interval = 40 // self.config.batch_size # steps
		summary_interval = 200 # steps
		validate_interval = 200 # steps
		output_interval = 1000 # steps
		checkpoint_interval = 1000 # steps

		print('Start iterating...')

		start_time = time.time()

		train_D_net = not self.config.no_adversarial
		batch_grad_G_list = None
		batch_grad_D_list = None
		batch_losses_G = None
		batch_losses_D = None
		step_losses_G = None
		step_losses_D = None

		while True:

			# compute epochs

			epochs = 1.0*(self.step+1)*self.config.batch_size/num_train_shapes
			do_print = ((self.step+1) % print_interval == 0)
			do_update = ((self.step+1) % update_interval == 0)
			do_validate = ((self.step+1) % validate_interval == 0)
			do_summary = ((self.step+1) % summary_interval == 0)
			do_checkpoint = ((self.step+1) % checkpoint_interval == 0)
			do_output = ((self.step+1) % output_interval == 0)

			# training networks

			step_G_list = sess.run(self.grad_G_list + [self.bn_G_op, self.train_losses_G])
			step_grad_G_list = step_G_list[:-2]
			step_losses_G = step_G_list[-1] / self.config.batch_size
			batch_grad_G_list = self.cumulate_gradients(batch_grad_G_list, step_grad_G_list)

			if train_D_net:
				step_D_list = sess.run(self.grad_D_list + [self.bn_D_op, self.train_losses_D])
				step_grad_D_list = step_D_list[:-2]
				step_losses_D = step_D_list[-1] / self.config.batch_size
				batch_grad_D_list = self.cumulate_gradients(batch_grad_D_list, step_grad_D_list)
			else:
				if step_losses_D is None:
					step_losses_D = [0.0, 0.0, 0.0]

			batch_losses_G = step_losses_G if batch_losses_G is None else batch_losses_G+step_losses_G
			batch_losses_D = step_losses_D if batch_losses_D is None else batch_losses_D+step_losses_D

			# update gradients

			if do_update:
				grad_G_dict = {}
				for k in range(len(self.grad_G_placeholder)):
					grad_G_dict[self.grad_G_placeholder[k][0]] = batch_grad_G_list[k] / update_interval
				sess.run(self.update_G_op, feed_dict=grad_G_dict)
				batch_grad_G_list = None

				if train_D_net:
					grad_D_dict = {}
					for k in range(len(self.grad_D_placeholder)):
						grad_D_dict[self.grad_D_placeholder[k][0]] = batch_grad_D_list[k] / update_interval
					sess.run(self.update_D_op, feed_dict=grad_D_dict)
					batch_grad_D_list = None

				if not self.config.no_adversarial:
					batch_losses_G = batch_losses_G / update_interval
					if batch_losses_D is not None:
						batch_losses_D = batch_losses_D / update_interval
						train_D_net = (batch_losses_D[0] > batch_losses_G[2] * 0.1) # NOTE: subscript
					batch_losses_G = None
					batch_losses_D = None

			# validation

			if do_validate:
				self.validate_loss(sess, num_valid_shapes)

			if do_output:
				self.validate_output(sess, num_valid_shapes, epochs)

			# log

			if do_summary:
				summary_G_str = sess.run(self.train_summary_G_op)
				self.summarizer.add_summary(summary_G_str, self.step)
				if train_D_net:
					summary_D_str = sess.run(self.train_summary_D_op)
					self.summarizer.add_summary(summary_D_str, self.step)

			if do_checkpoint:
				self.saver.save(sess, os.path.join(self.config.train_dir,'model.ckpt'), global_step=self.step+1)
			
			if do_print:
				now_time = time.time()
				batch_duration = now_time - start_time
				start_time = now_time
				log_str_1 = 'Step %7d: %5.1f sec, epoch: %7.2f, ' % (self.step+1, batch_duration, epochs)
				log_str_2 = 'losses: %7.3g, %7.3g, %7.3g, %7.3g, %7.3g, %7.3g;' % \
					(step_losses_G[0], step_losses_G[1], step_losses_G[2], step_losses_D[0], step_losses_D[1], step_losses_D[2])
				print(log_str_1, end='')
				print(log_str_2)
				log_file_name = os.path.join(self.config.train_dir,'log.txt')
				with open(log_file_name, 'a') as log_file:
					log_file.write(log_str_1+log_str_2+'\n')

			if epochs >= self.config.max_epochs:
				break

			self.step += 1

		coord.request_stop()
		coord.join(threads)

	def test(self, sess, views, num_shapes):

		print('Testing...')

		self.saver = tf.train.Saver()
		ckpt = tf.train.get_checkpoint_state(self.config.train_dir)
		if ckpt and ckpt.model_checkpoint_path:
			self.saver.restore(sess, ckpt.model_checkpoint_path)
			try:
				self.step = int(ckpt.model_checkpoint_path.split('/')[-1].split('-')[-1])
			except ValueError:
				self.step = 0
		else:
			print('Cannot find any checkpoint file')
			return

		coord = tf.train.Coordinator()
		threads = tf.train.start_queue_runners(sess=sess, coord=coord)
		self.summarizer = tf.summary.FileWriter(self.config.test_dir, sess.graph)

		output_count = 0
		output_prefix = 'dn14'
		output_images_folder = 'images'
		output_results_folder = 'results'

		log_file_name = os.path.join(self.config.test_dir,'log.txt')
		log_file = open(log_file_name, 'a')

		started = False
		finished = False
		last_shape_name = ''
		last_view_name = ''
		while not finished:
			names,results,errors,images = sess.run([self.names, self.results, self.errors, self.pngs])
			for k in range(len(names)):
				shape_name, view_name = names[k].decode('utf8').split('--')
				if last_shape_name == shape_name:
					view_name = ('%s' % (int(last_view_name)+1))
				last_shape_name = shape_name
				last_view_name = view_name
				print('Processed %d: %s--%s %f' % (output_count, shape_name, view_name, errors[k]))

				if view_name == '0' and started:
					log_file.write('\n')
				started = True
				log_file.write('%6f ' % errors[k])

				# export images
				name_input = os.path.join(self.config.test_dir, output_images_folder, shape_name, 'input.png')
				image.write_image(name_input, images[0, k])
				name_gt = os.path.join(self.config.test_dir, output_images_folder, shape_name, ('gt-'+output_prefix+'--'+view_name+'.png'))
				name_output = os.path.join(self.config.test_dir, output_images_folder, shape_name, ('pred-'+output_prefix+'--'+view_name+'.png'))
				image.write_image(name_gt, images[1, k])
				image.write_image(name_output, images[2, k])
				
				name_normal = os.path.join(self.config.test_dir, output_images_folder, shape_name, ('normal-'+output_prefix+'--'+view_name+'.png'))
				name_depth = os.path.join(self.config.test_dir, output_images_folder, shape_name, ('depth-'+output_prefix+'--'+view_name+'.png'))
				name_mask = os.path.join(self.config.test_dir, output_images_folder, shape_name, ('mask-'+output_prefix+'--'+view_name+'.png'))
				image.write_image(name_normal, images[3, k])
				image.write_image(name_depth, images[4, k])
				image.write_image(name_mask, images[5, k])

				# export results
				name_output = os.path.join(self.config.test_dir, output_results_folder, shape_name, (output_prefix+'-'+view_name+'.png'))
				image.write_image(name_output, images[2, k])

				# check termination
				output_count += 1
				if output_count >= num_shapes * views.num_views:
					finished = True
					break

		coord.request_stop()
		coord.join(threads)

	def encode(self, sess, views, num_shapes):

		print('Encoding...')

		self.saver = tf.train.Saver()
		ckpt = tf.train.get_checkpoint_state(self.config.train_dir)
		if ckpt and ckpt.model_checkpoint_path:
			self.saver.restore(sess, ckpt.model_checkpoint_path)
			self.step = int(ckpt.model_checkpoint_path.split('/')[-1].split('-')[-1])
		else:
			print('Cannot find any checkpoint file')
			return

		coord = tf.train.Coordinator()
		threads = tf.train.start_queue_runners(sess=sess, coord=coord)
		self.summarizer = tf.summary.FileWriter(self.config.encode_dir, sess.graph)

		output_count = 0
		output_folder = 'features'

		finished = False
		while not finished:
			names,features = sess.run([self.encode_names, self.encode_features])
			for k in range(len(names)):
				shape_name = names[k].decode('utf8')
				print('Processed %d: %s' % (output_count, shape_name))

				# export results
				name_output = os.path.join(self.config.encode_dir, output_folder, (shape_name+'.bin'))
				data.write_bin_data(name_output, features[k])

				# check termination
				output_count += 1
				if output_count >= num_shapes:
					finished = True
					break

		coord.request_stop()
		coord.join(threads)

	def validate_loss(self, sess, num_shapes):

		num_processed_shapes = 0
		cum_losses = None
		while num_processed_shapes < num_shapes:
			losses = sess.run(self.valid_losses)
			losses = np.array(losses)
			cum_losses = losses if cum_losses is None else cum_losses+losses
			num_processed_shapes += self.config.batch_size
		cum_losses /= num_processed_shapes

		print('===== validation loss: %.3g' % cum_losses[0])

		summary_str = sess.run(self.valid_summary_op, feed_dict={self.valid_summary_losses:cum_losses})
		self.summarizer.add_summary(summary_str, self.step)

	def validate_output(self, sess, num_shapes, epochs):

		print('===== validation output')
		valid_results_folder = 'epoch-%.2f' % epochs
		names, images = sess.run([self.names, self.valid_images])

		for k in range(len(names)):
			shape_name, view_name = names[k].decode('utf8').split('--')
			if view_name == '0':
				print(shape_name)
			
			name_output = os.path.join(self.config.train_dir, valid_results_folder, shape_name, ('output--'+view_name+'.png'))
			name_gt = os.path.join(self.config.train_dir, valid_results_folder, shape_name, ('gt--'+view_name+'.png'))
			image.write_image(name_output, images[0, k])
			image.write_image(name_gt, images[1, k])
			
			name_normal = os.path.join(self.config.train_dir, valid_results_folder, shape_name, ('normal--'+view_name+'.png'))
			name_depth = os.path.join(self.config.train_dir, valid_results_folder, shape_name, ('depth--'+view_name+'.png'))
			name_mask = os.path.join(self.config.train_dir, valid_results_folder, shape_name, ('mask--'+view_name+'.png'))
			image.write_image(name_normal, images[2, k])
			image.write_image(name_depth, images[3, k])
			image.write_image(name_mask, images[4, k])

		# loop over all remaining shapes in the queue...
		num_processed_shapes = self.config.batch_size
		while num_processed_shapes < num_shapes:
			sess.run(self.names)
			num_processed_shapes += self.config.batch_size

	def cumulate_gradients(self, cum_grads, grads):
		if cum_grads is None:
			cum_grads = grads
		else:
			for k in range(len(grads)):
				cum_grads[k] += grads[k]
		return cum_grads