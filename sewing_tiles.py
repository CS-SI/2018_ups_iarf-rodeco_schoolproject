# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 15:43:52 2019

@author: Alexandre Berdeaux
"""

from PIL import Image, ImageEnhance
from netCDF4 import Dataset
import xray
import argparse
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Tiles sewing machine : Takes '
                                     'the different tiles data.nc from s2p and '
                                     'sew them back together.')

    parser.add_argument('tiles', metavar='tiles_folder',
                        help=('path to the folder where s2p\'s tiles '
                        'are stored'))

    args = parser.parse_args()
    s2pout_folder = args.tiles
    tiles_folder = '{}/tiles'.format(s2pout_folder)



    if not os.path.isdir(tiles_folder):
        print('The folder is not a s2p output, exiting')
        exit(1)

    nc_folder = os.path.join(s2pout_folder, 'data.nc')

    rowslist = os.listdir(tiles_folder)
    datasets = [[xray.open_dataset(os.path.join(tiles_folder, rows, cols, 'data.nc'))
                 for cols in os.listdir(os.path.join(tiles_folder,rows))]
                 for rows in os.listdir(tiles_folder)]

    for i in range(len(datasets)):
        datasets[i] = sorted(datasets[i], key=lambda set: int(set.attrs['width_pos_complete_reference']))

    mergedrows = [xray.concat(datasets[i], 'height') for i in range(len(datasets))]
    mergedrows = sorted(mergedrows, key=lambda dataset: dataset.attrs['height_pos_complete_reference'] )
    merged = xray.concat(mergedrows,'width')

    merged.to_netcdf(nc_folder,'w','NETCDF3_CLASSIC')
