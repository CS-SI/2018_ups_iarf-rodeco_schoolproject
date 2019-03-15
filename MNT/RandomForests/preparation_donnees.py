import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from sklearn.ensemble import RandomForestClassifier
from netCDF4 import Dataset
from skimage.transform import pyramid_gaussian


# Entrees :
#	fileName : nom du fichier netCDF à ouvrir
#	nb_zoom : nombre d'image de la pyramide de Gauss
# Sortie : 
#	Grad : liste des gradients d'altitude des images de la pyramide de Gauss
def preparation_donnees(fileName, nb_zoom):
	
	#ouverture fichier netCDF
	dataset = Dataset(fileName) 

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

