# -*- coding: utf-8 -*-
"""
@author: Astrid Bourgois
"""

from sklearn.ensemble import RandomForestClassifier
from sklearn import metrics
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

  veriteTerrain = imgBuilding

  for i in range (len(imgSol)):
    for j in range (len(imgSol[0])) :
      if imgSol[i,j] == 1 :
        veriteTerrain[i,j] = 0
      elif veriteTerrain[i,j] == 0 :
        veriteTerrain[i,j] = 3

  return veriteTerrain

# Entrees :
#	fileName : nom du fichier netCDF à ouvrir
#	nb_zoom : nombre d'image de la pyramide de Gauss
# Sortie :
#	Grad : tableau des gradients d'altitude des images de la pyramide de Gauss
def preparation_donnees(altitudes, nb_zoom):

	#creation de la pyramide gaussienne
	pyramid= tuple(pyramid_gaussian(altitudes, nb_zoom))

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
hend = int(math.floor(end/2))
print(hend)

g = preparation_donnees(altitudes[:,:hend], nb_zoom)
train = g[0]
g = preparation_donnees(altitudes[:,hend:end], nb_zoom)
test = g[0]

ytrain = y[:,:hend]#
ytest = y[:,hend:end]#

clf = RandomForestClassifier(n_estimators=100, max_depth=2,random_state=0)
clf.fit(train, ytrain)

predict = clf.predict(test)

res1 = np.equal(ytest, predict)
known = np.array(ytest)
known[known != 3] = True
known[known == 3] = False

error = np.logical_and(res1,known)

accuracy_score = float(sum(sum(error))) / float(sum(sum(known)))
print(accuracy_score)

img = ytest
img[img == 3] = 128
img[img == 1] = 255
img = Image.fromarray(img)
img = img.convert('L')
img.save('sortie/veriteTerrain.jpg')

img = predict
img[img == 3] = 128
img[img == 1] = 255
img = Image.fromarray(predict)
img = img.convert('L')
img.save('sortie/predict.jpg')

known[known == 1] = 255
img = Image.fromarray(known)
img = img.convert('L')
img.save('sortie/known.jpg')

img = Image.fromarray(res1)
img = img.convert('L')
img.save('sortie/erreur.jpg')

#res = metrics.accuracy_score(ytest, predict, normalize=True)
#print(res)
