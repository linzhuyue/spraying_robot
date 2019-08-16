#! /usr/bin/env python
# coding=utf-8
from random import randint
class SearchEntry():
    	def __init__(self, x, y, g_cost, f_cost=0, pre_entry=None):
		self.x = x
		self.y = y
		# cost move form start entry to this entry
		self.g_cost = g_cost
		self.f_cost = f_cost
		self.pre_entry = pre_entry
	
	def getPos(self):
		return (self.x, self.y)

class Map():
	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.map = [[0 for x in range(self.width)] for y in range(self.height)]
	
	def createBlock(self, block_num):
		for i in range(block_num):
			x, y = (randint(0, self.width-1), randint(0, self.height-1))
			self.map[y][x] = 1
	
	def generatePos(self, rangeX, rangeY):
		x, y = (randint(rangeX[0], rangeX[1]), randint(rangeY[0], rangeY[1]))
		while self.map[y][x] == 1:
			x, y = (randint(rangeX[0], rangeX[1]), randint(rangeY[0], rangeY[1]))
		return (x , y)

	def showMap(self):
		print("+" * (3 * self.width + 2))

		for row in self.map:
			s = '+'
			for entry in row:
				s += ' ' + str(entry) + ' '
			s += '+'
			print(s)

	    print("+" * (3 * self.width + 2))
def 