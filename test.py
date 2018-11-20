import pdb
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math
import random
from copy import deepcopy


default_map = [[255]*512 for i in range(512)]
sample1_map = deepcopy(default_map)
sample2_map = deepcopy(default_map)
for i in range(100, 200):
	default_map[i][200:500] = [0]*(500-200)
for i in range(400,500):
	default_map[i][100:300] = [0]*(300-100)
for j in range(200, 400):
	default_map[j][200:300] = [125]*(300-200)

for i in range(50, 150):
	sample2_map[i][100:400] = [0]*(400-100)
for i in range(150, 200):
	sample2_map[i][100:400] = [125]*(400-100)
for i in range(200,250):
	sample2_map[i][100:400] = [0]*(400-100)
for i in range(250, 300):
	sample2_map[i][100:400] = [75]*(400-100)
for i in range(300, 350):
	sample2_map[i][100:400] = [0]*(400-100)
for i in range(350, 400):
	sample2_map[i][100:350] = [0]*(350-100)

for i in range(100, 200):
	sample1_map[i][100:200] = [0]*(200-100)
	sample1_map[i][300:400] = [0]*(400-300)
for i in range(300,400):
	sample1_map[i][100:200] = [0]*(200-100)
	sample1_map[i][300:400] = [0]*(400-300)
for i in range(400, 512):
	sample1_map[i][100:200] = [0]*(200-100)
	sample1_map[i][300:400] = [0]*(400-300)

plt.clf()

plt.imshow(default_map, cmap='gray')

plt.grid()