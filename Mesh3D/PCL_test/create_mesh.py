import multiprocessing
import subprocess
import argparse
import os.path
import json
import yaml
import sys

from meshpylib.defaultcfg import cfg
import meshpylib.argument_loader as load_params


def init_cfg(cfg_usr):
    if 'in_ply' not in cfg_usr:
        print('No entry file')
        sys.exit(1)

    if 'mu' not in cfg_usr['greedyTriangulation'] or \
    'searchRadius' not in cfg_usr['greedyTriangulation']:
        print('Minimum configuration for Greedy Projection Triangulation not set')
        sys.exit(1)

    if 'alpha' not in cfg_usr['concaveHull']:
        print('Minimum configuration for Concave Hull not set')
        sys.exit(1)

    for key in cfg_usr.keys():
        if (cfg.__contains__(key)):
            if isinstance(cfg[key],dict):
                cfg[key].update(cfg_usr[key])
            else:
                cfg[key] = cfg_usr[key]
        else:
            cfg[key] = cfg_usr[key]

    if not os.path.exists(cfg['out_dir']):
        os.mkdir(cfg['out_dir'])


def launch_command(args):
    print('Launching {}\n'.format(args[0]))
    retval = subprocess.call(args)
    if retval != 0:
        print('ERROR : {} encountered an error, code {}\n'.format(args[0],retval))
    else:
        print('\n{} completed the reconstruction\n'.format(args[0]))


def launch_reconstruction(cfg_usr):

    init_cfg(cfg_usr)

    args = load_params.argument_normal()
    launch_command(args)

    args = load_params.argument_greedy()
    launch_command(args)

    args = load_params.argument_poisson()
    launch_command(args)

    args = load_params.argument_concave()
    launch_command(args)

    args = load_params.argument_convex()
    launch_command(args)

    args = load_params.argument_organizedfastmesh()
    launch_command(args)

def read_cfg(cfg_file):
    with open(cfg_file, 'r') as f:
        cfg_usr = yaml.safe_load(f)

    opath = cfg_usr['out_dir']

    if not os.path.isabs(opath):
        cfg_path = os.path.abspath(os.path.dirname(cfg_file))
        cfg_usr['out_dir'] = os.path.join(cfg_path, opath)
        print('out_dir is: {}\n'.format(cfg_usr['out_dir']))

    return cfg_usr


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Mesh Creator : allow to compare '
                                     'multiple reconstruction algorithme for a '
                                     'single point cloud')

    parser.add_argument('config', metavar='config.json',
                        help=('path to a json file containing the paths to '
                              'input and output files and the algorithm '
                              'parameters'))

    args = parser.parse_args()
    cfg_usr = read_cfg(args.config)

    launch_reconstruction(cfg_usr)
