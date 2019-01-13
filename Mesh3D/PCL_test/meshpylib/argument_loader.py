from defaultcfg import cfg

def argument_poisson():
    args = ['./poissonReconstruction']

    args += [cfg['in_ply']]

    args += [str(cfg['normalEstimation']['useKtree'])]
    args += [str(cfg['normalEstimation']['KSearch'])]

    reconsArgs = cfg['poissonReconstruction']
    args += [str(reconsArgs['depth'])]
    args += [str(reconsArgs['minDepth'])]
    args += [str(reconsArgs['solverDivide'])]
    args += [str(reconsArgs['isoDivide'])]
    args += [str(reconsArgs['degree'])]
    args += [str(reconsArgs['pointWeight'])]
    args += [str(reconsArgs['scale'])]
    args += [str(reconsArgs['samplePerNode'])]
    args += [str(reconsArgs['confidence'])]
    args += [str(reconsArgs['outputPolygon'])]
    args += [str(reconsArgs['manifold'])]

    args += ['{}/poissonReconstruction.ply'.format(cfg['out_dir'])]

    return args
