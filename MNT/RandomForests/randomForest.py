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
import libtiff
from libtiff import TIFF
from PIL import Image
from os import path
from netCDF4 import Dataset
import math
import argparse

libtiff.libtiff_ctypes.suppress_warnings()

def verite_terrain(in_dir):
  sol = path.join(in_dir,'sol.tif')
  building = path.join(in_dir,'building.tif')

  imageSol=TIFF.open(sol)
  imgSol = imageSol.read_image()

  imageBuilding=TIFF.open(building)
  imgBuilding = imageBuilding.read_image()

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
  pyramid=tuple(pyramid_gaussian(altitudes,nb_zoom, multichannel=False))

  #calcul des gradients
  Grad = []
  i = 0
  j = 0

  for i in range(nb_zoom):
      x,y = np.gradient(pyramid[i])
      Grad.append(np.sqrt(np.add(np.square(x), np.square(y))))

  for i in range(1, nb_zoom, 1):
      Grad[i] = np.repeat(Grad[i], math.pow(2,i), axis=0)
      Grad[i] = np.repeat(Grad[i], math.pow(2,i), axis=1)

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

parser.add_argument('train_lower_x')
parser.add_argument('train_higher_x')
parser.add_argument('train_lower_y')
parser.add_argument('train_higher_y')

parser.add_argument('test_lower_x')
parser.add_argument('test_higher_x')
parser.add_argument('test_lower_y')
parser.add_argument('test_higher_y')

args = parser.parse_args()

nb_zoom = int(args.nb_zoom)
in_dir = args.input_dir

train_box = (int(args.train_lower_x), int(args.train_higher_x),
             int(args.train_lower_y), int(args.train_higher_y))
train_size = (train_box[1] - train_box[0], train_box[3] - train_box[2])

test_box = (int(args.test_lower_x), int(args.test_higher_x),
            int(args.test_lower_y), int(args.test_higher_y))
test_size = (test_box[1] - test_box[0], test_box[3] - test_box[2])


#ouverture fichier netCDF
dataset = Dataset(path.join(in_dir,'data.nc'))

# recuperation des altitudes
altitudes = dataset.variables['altitude'][:,:]



y = verite_terrain(in_dir)

# préparation donnees train et test
train = preparation_features(altitudes[train_box[0]:train_box[1],
                                       train_box[2]:train_box[3]])

test = preparation_features(altitudes[test_box[0]:test_box[1],
                                      test_box[2]:test_box[3]])

# praparation des verite terrain train et test
ytrain = np.ndarray.flatten(y[train_box[0]:train_box[1],
                              train_box[2]:train_box[3]])
ytrain = ytrain.reshape(ytrain.shape[0],1)

ytest = np.ndarray.flatten(y[test_box[0]:test_box[1],
                             test_box[2]:test_box[3]])
ytest = ytest.reshape(ytest.shape[0],1)

# applique le masque aux donnée d'entrainement
ytrain = np.ma.masked_equal(ytrain, 3)
mask = np.repeat(np.ma.getmask(ytrain), train.shape[1], axis=1)
train = np.ma.array(train, mask=mask)
ytrain = ytrain.compressed()
train = train.compressed()
train = train.reshape(ytrain.shape[0],nb_zoom+1)

print('Statistiques sur la véritée terrain d\'entrainement')
train_sol = float(len(ytrain[ytrain == 1]))
train_sursol = float(len(ytrain[ytrain == 0]))
train_total = float(len(ytrain))
train_ratio_sol = train_sol / train_total
train_ratio_sursol = train_sursol / train_total
print('Pourcentage de pixels de sol : ', train_ratio_sol * 100)
print('Pourcentage de pixels de sursol : ', train_ratio_sursol * 100)

print('Statistiques sur la véritée terrain de test')
test_sol = float(len(ytest[ytest == 1]))
test_sursol = float(len(ytest[ytest == 0]))
test_total = float(len(ytest[ytest != 3]))
test_ratio_sol = test_sol / test_total
test_ratio_sursol = test_sursol / test_total
print('Pourcentage de pixels de sol : ', test_ratio_sol * 100)
print('Pourcentage de pixels de sursol : ', test_ratio_sursol * 100)

#ytest = np.ma.masked_equal(ytest, 3)
#mask = np.repeat(np.ma.getmask(ytest), test.shape[1], axis=1)
#test = np.ma.array(test, mask=mask)
#ytest = ytest.compressed()
#ytest = ytest.reshape(ytest.shape[0],1)
#test = test.compressed()
#test = test.reshape(ytest.shape[0],nb_zoom+1)

weight = np.ndarray.flatten(np.array(ytrain))
weight[weight == 0] = 1/train_ratio_sol
weight[weight == 1] = 1/train_ratio_sursol

class_weight = {0:1/test_ratio_sol, 1:1/train_ratio_sursol}
#print(train.shape, ytrain.shape)
#print(ytrain, weight)

# preparation du random forest
clf = RandomForestClassifier(n_estimators=10, max_depth=None,random_state=None,
                             class_weight=class_weight, criterion='gini',
                             max_features=None ,n_jobs=-1)

# apprentissage du rf
clf.fit(train, ytrain)

# test du rf
predict = clf.predict(test)
predict = predict.reshape(predict.shape[0],1)

known = np.array(ytest)
known[known != 3] = True
known[known == 3] = False

error = np.logical_and(np.equal(predict, ytest), known)
accuracy_score = float(sum(error)) / float(sum(known))
print('Score de precision : ',accuracy_score)

img = ytest
img[img == 3] = 128
img[img == 1] = 255
img = Image.fromarray(ytest.reshape(test_size))
img = img.convert('L')
img.save('sortie/veriteTerrain.jpg')

predict[predict == 1] = 255
img = Image.fromarray(predict.reshape(test_size))
img = img.convert('1')
img.save('sortie/predict.jpg')

known[known == 1] = 255
img = Image.fromarray(known.reshape(test_size))
img = img.convert('1')
img.save('sortie/known.jpg')

error[error == 0] = 0
error[error == 1] = 255
img = Image.fromarray(error.reshape(test_size))
img = img.convert('L')
img.save('sortie/erreur.jpg')
