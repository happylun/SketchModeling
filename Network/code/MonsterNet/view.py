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


#import tensorflow as tf
import numpy as np

class Views(object):

	def __init__(self, filename, num_views=-1):
		"""
			self.views:   V x 3
			self.groups   G x v
		"""

		f = open(filename, 'r')

		f.readline() # OFF
		self.num_views, self.num_groups, num_edges = map(int, f.readline().split())

		view_data = []
		for view_id in range(self.num_views):
			view_data.append(list(map(float, f.readline().split())))
		self.views = np.array(view_data)

		group_data = []
		for group_id in range(self.num_groups):
			group_data.append(list(map(int, f.readline().split()[1:])))
		self.groups = np.array(group_data)

		f.close()
		
		if num_views >= 0: # select views
			self.num_views = num_views
			self.num_groups = 0
			self.views = self.views[:self.num_views]
			self.groups = self.groups[:0]

		self.num_edges = self.num_views+self.num_groups-2
		self.edge_size = 2

		# HACK: minimal data for local testing
		#self.num_views = 3
		#self.num_groups = 0
		#self.num_edges = 1
		#self.edge_size = 2
		#self.views = self.views[:self.num_views]
		#self.groups = self.groups[:self.num_groups]

		#print('Views:')
		#print(self.views)
		#print('Groups:')
		#print(self.groups)

def view2angle(view):
	"""
		input:
			view  : 3  : (x,y,z)
		output:
			angle : 4  : (cos(theta), sin(theta), cos(phi), sin(phi))
	"""

	r = np.linalg.norm(view) # sqrt(x^2+y^2+z^2)
	rxz = np.linalg.norm(view[[0,2]]) # sqrt(x^2+z^2)
	ct = view[1] / r # cos(theta) = y/r
	st = rxz / r # sin(theta) = sqrt(x^2+z^2)/r
	if rxz>0:
		cp = view[0] / rxz # cos(phi) = x / sqrt(x^2+z^2)
		sp = view[2] / rxz # sin(phi) = z / sqrt(x^2+z^2)
	else: # zenith point
		cp = 0.0
		sp = 0.0
	return [ct, st, cp, sp]
