# -*- coding: utf-8 -*-
"""
@author: Astrid Bourgois
"""

from sklearn.ensemble import RandomForestClassifier
from skimage.transform import pyramid_gaussian
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from libtiff import TIFF
from PIL import Image
from os import path
from netCDF4 import Dataset
import math
import argparse

def veriteTerrain(input_dir):
  sol = path.join(input_dir,'sol.tif')
  building = path.join(input_dir,'building.tif')

  imageSol=TIFF.open(sol)
  imgSol = imageSol.read_image()

  imageBuilding=TIFF.open(building)
  imgBuilding = imageBuilding.read_image()

  print(imgSol.shape)

  veriteTerrain = imgBuilding

  for i in range (500):
    for j in range (500) :
      if imgSol[i,j] == 1 :
        veriteTerrain[i,j] = 0
      elif veriteTerrain[i,j] == 0 :
        veriteTerrain[i,j] = float('nan')

  return veriteTerrain

# Entrees :
#	fileName : nom du fichier netCDF à ouvrir
#	nb_zoom : nombre d'image de la pyramide de Gauss
# Sortie :
#	Grad : tableau des gradients d'altitude des images de la pyramide de Gauss
def preparation_donnees(altitudes, nb_zoom):

	#creation de la pyramide gaussienne
	pyramid=tuple(pyramid_gaussian(altitudes,nb_zoom))

	#calcul des gradients
	Grad = []
	for i in range(nb_zoom):
	  x,y = np.gradient(pyramid[i])
	  Grad.append(np.sqrt(np.add(np.square(x), np.square(y))))

	return Grad

parser = argparse.ArgumentParser()
parser.add_argument('input_dir')
parser.add_argument('nb_zoom')
args = parser.parse_args()

nb_zoom = int(args.nb_zoom)
in_dir = args.input_dir

#ouverture fichier netCDF
dataset = Dataset(path.join(in_dir,'data.nc'))

# recuperation des altitudes
altitudes = dataset.variables['altitude'][:,:]

y = veriteTerrain(in_dir)

end = len(y[0])
hend = end/2

ytrain = y[:,:hend-1]#
ytest = y[hend:end-1]#

g = preparation_donnees(altitudes[:,:hend-1], nb_zoom)
train = g
g = preparation_donnees(altitudes[hend:end-1], nb_zoom)
test = g

clf = RandomForestClassifier(n_estimators=100, max_depth=2,random_state=0)
clf.fit(train, ytrain)

clf.predict(test)
