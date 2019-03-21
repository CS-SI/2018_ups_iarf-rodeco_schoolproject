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


def verite_terrain(in_dir):
  sol = path.join(in_dir,'sol.tif')
  building = path.join(in_dir,'building.tif')

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
        veriteTerrain[i,j] = 3

  return veriteTerrain


# Entrees :
#	fileName : nom du fichier netCDF à ouvrir
#	nb_zoom : nombre d'image de la pyramide de Gauss
# Sortie :
#	Grad : tableau des gradients d'altitude des images de la pyramide de Gauss
def preparation_donnees(altitudes, nb_zoom):

  #creation de la pyramide gaussienne
  pyramid=tuple(pyramid_gaussian(altitudes,nb_zoom))
  print(pyramid[0].shape, pyramid[1].shape)
  #calcul des gradients
  Grad = []
  i = 0
  j = 0

  for i in range(nb_zoom):
    x,y = np.gradient(pyramid[i])
    Grad.append(np.sqrt(np.add(np.square(x), np.square(y))))

  for i in range(1,nb_zoom,1):

    aux = np.zeros(pyramid[0].shape)

    for o in range(i):
      for x in range(Grad[i].shape[0]):
        for y in range(Grad[i].shape[1]):

          u = x * 2
          j = y * 2
          aux[u,j]= Grad[i][x,y]
          aux[u+1,j] = Grad[i][x,y]
          aux[u,j+1] = Grad[i][x,y]
          aux[u+1,j+1] = Grad[i][x,y]

      Grad[i] = aux
  return Grad


def preparation_features (altitude):

  alt = altitude
  grad = (preparation_donnees(alt, nb_zoom))
  alt = np.ndarray.flatten(alt)
  alt = alt.reshape(alt.shape[0],1)

  features = np.array([np.ndarray.flatten(grad[i]) for i in range(nb_zoom)])
  features = np.transpose(features)
  features = np.concatenate((alt,features),axis=1)

  return features


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



y = verite_terrain(in_dir)

end = len(y[0])
hend = int(math.floor(end/2))

# préparation donnees train et test
train = preparation_features(altitudes[:,:hend])

test = preparation_features(altitudes[:,hend:end])

# praparation des verite terrain train et test
ytrain = np.ndarray.flatten(y[:,:hend])
ytrain = ytrain.reshape(ytrain.shape[0],1)

ytest = np.ndarray.flatten(y[:,hend:end])
ytest = ytest.reshape(ytest.shape[0],1)


# applique le masque aux donnée d'entrainement
ytrain = np.ma.masked_equal(ytrain, 3)
mask = np.repeat(np.ma.getmask(ytrain), train.shape[1], axis=1)
train = np.ma.array(train, mask=mask)
ytrain = ytrain.compressed()
train = train.compressed()
train = train.reshape(ytrain.shape[0],nb_zoom+1)

#ytest = np.ma.masked_equal(ytest, 3)
#mask = np.repeat(np.ma.getmask(ytest), test.shape[1], axis=1)
#test = np.ma.array(test, mask=mask)
#ytest = ytest.compressed()
#ytest = ytest.reshape(ytest.shape[0],1)
#test = test.compressed()
#test = test.reshape(ytest.shape[0],nb_zoom+1)

weight = np.ndarray.flatten(np.array(ytrain))
weight[weight == 0] = 0.7
weight[weight == 1] = 0.5
print(weight.shape)

#print(train.shape, ytrain.shape)
#print(ytrain, weight)

# preparation du random forest
clf = RandomForestClassifier(n_estimators=100, max_depth=8,random_state=0,
                             class_weight="balanced", criterion='entropy')

# apprentissage du rf
clf.fit(train, ytrain, weight)

print(clf.n_classes_)

# test du rf
predict = clf.predict(test)
predict = predict.reshape(predict.shape[0],1)

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
img = Image.fromarray(img.reshape(end,hend))
img = img.convert('L')
img.save('sortie/veriteTerrain.jpg')

img = predict
img[img == 3] = 128
img[img == 1] = 255
img = Image.fromarray(img.reshape(end,hend))
img = img.convert('L')
img.save('sortie/predict.jpg')

known[known == 1] = 255
img = Image.fromarray(known.reshape(end,hend))
img = img.convert('L')
img.save('sortie/known.jpg')

img = Image.fromarray(error.reshape(end,hend))
img = img.convert('L')
img.save('sortie/erreur.jpg')

#res = metrics.accuracy_score(ytest, predict, normalize=True)
#print(res)
