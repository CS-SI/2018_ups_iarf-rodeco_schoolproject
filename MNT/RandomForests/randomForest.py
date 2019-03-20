# -*- coding: utf-8 -*-
"""
Created on Wed Mar 20 11:30:19 2019

@author: etudiant
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

    in_folder = 'entree'

    sol = path.join(in_folder,'sol.tif')
    building = path.join(in_folder,'building.tif')
    
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
                veriteTerrain[i,j] = float('NaN')
            else:
                veriteTerrain[i,j] = 1
          
    plt.imshow(veriteTerrain)
    
    return res

# Entrees :
#	fileName : nom du fichier netCDF à ouvrir
#	nb_zoom : nombre d'image de la pyramide de Gauss
# Sortie : 
#	Grad : tableau des gradients d'altitude des images de la pyramide de Gauss
def preparation_donnees(input_dir, nb_zoom):
	
	#ouverture fichier netCDF
	dataset = Dataset(path.join(input_dir,'data.nc')) 

	# recuperation des altitudes
	altitudes = dataset.variables['altitude'][:,:]

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

g = preparation_donnees(in_dir, nb_zoom)
groundTruth = veriteTerrain(in_dir)

train = g[0:(end/2)-1,:]#
test = g[end/2:end,:]#

ytrain = groundTruth[0:(end/2)-1,:]#
ytest = groundTruth[end/2:end,:]#

clf = RandomForestClassifier(n_estimators=100, max_depth=2,random_state=0)
clf.fit(train, yTrain)

clf.predict(test)
