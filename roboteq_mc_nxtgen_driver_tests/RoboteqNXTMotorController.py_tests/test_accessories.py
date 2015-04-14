#!/usr/bin/env python

from __future__ import print_function #this is just to use print in a lambda function http://stackoverflow.com/questions/2970858/python-lambda-with-print-statement

class MyLog:
	def __init__(self):
		self.l = []
	def clean(self):
		self.l=[]
	def has(self, msg):
		return msg in self.l
	def log(self, msg):
		self.l.append(msg)
		print("	MC: ", msg)
