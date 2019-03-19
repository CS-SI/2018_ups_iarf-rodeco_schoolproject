# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 15:43:52 2019

@author: Alexandre Berdeaux
"""

#from matplotlib import mpimg
from netCDF4 import Dataset
from PIL import Image
import matplotlib.pyplot as plot
import numpy.ma as ma
import numpy as np
import json
import xray
import argparse
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract image : Takes '
                                     'the part of base images used '
                                     'for pixel appariment.')

    parser.add_argument('data', metavar='data.nc',
                        help=('Resulting data from s2p output folder'))

    parser.add_argument('config', metavar='config.json',
                        help=('Configuration file in s2p output folder'))

    max_pixels = Image.MAX_IMAGE_PIXELS
    Image.MAX_IMAGE_PIXELS = None

    arguments = parser.parse_args()
    config_file = arguments.config

    with open(config_file, 'r') as f:
        user_cfg = json.load(f)

    dataset = Dataset(arguments.data, 'r','NETCDF3_CLASSIC')
    out_dir = os.path.dirname(arguments.data)

    secheight = dataset.variables['height_pos_secondary_image'][:,:,:]
    secheight = ma.masked_where(secheight < 0, secheight)

    secwidth = dataset.variables['width_pos_secondary_image'][:,:,:]
    secwidth = ma.masked_where(secwidth < 0, secwidth)

    #print([user_cfg['images'][i]['img'] for i in range(dataset.dimensions['sight'].size)])
    for j in range(1, dataset.dimensions['sight'].size, 1):
        minsech = min([min(secheight[i,:,j]) for i  in range(len(secheight))])
        maxsech = max([max(secheight[i,:,j]) for i in range(len(secheight))])
        minsecw = min([min(secwidth[i,:,j]) for i in range(len(secwidth))])
        maxsecw = max([max(secwidth[i,:,j]) for i in range(len(secwidth))])

        secimg = user_cfg['images'][j]['img']

        box = (minsecw,minsech,maxsecw,maxsech)
        print(box)

        print('Loading {}'.format(secimg))
        image = Image.open(secimg)
        print('Cropping image')
        area = image.crop(box)
        print('Saving croped image')
        area.save(os.path.join(out_dir,'sec{}.jp2'.format(j)))

    minrefh = dataset.getncattr('height_pos_complete_reference')
    maxrefh = minrefh + dataset.dimensions['height'].size
    minrefw = dataset.getncattr('width_pos_complete_reference')
    maxrefw = minrefw + dataset.dimensions['width'].size

    refimg = user_cfg['images'][0]['img']

    box = (minrefw,minrefh,maxrefw,maxrefh)
    print(box)

    print('Loading {}'.format(refimg))
    image = Image.open(refimg)
    print('Cropping image')
    area = image.crop(box)
    print('Saving croped image')
    area.save(os.path.join(out_dir,'ref.jp2'))

    Image.MAX_IMAGE_PIXELS = max_pixels



    #print(maxsech - minsech, maxsecw - minsecw)
