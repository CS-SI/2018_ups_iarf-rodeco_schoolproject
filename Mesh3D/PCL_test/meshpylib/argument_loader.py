from defaultcfg import cfg

def argument_mininit(name):
    args = ['./{}'.format(name)]

    args += [cfg['in_ply']]
    args += ['{}/{}.ply'.format(cfg['out_dir'], name)]

    return args


def argument_initwithnormal(name):
    args = argument_mininit(name)

    args += [str(cfg['normalEstimation']['useKtree'])]
    args += [str(cfg['normalEstimation']['KSearch'])]

    return args


def argument_greedy():
    name = 'greedyTriangulation'
    args = argument_initwithnormal(name)

    reconsArgs = cfg[name]
    args += [str(reconsArgs['mu'])]
    args += [str(reconsArgs['maximumNearestNeighbors'])]
    args += [str(reconsArgs['searchRadius'])]
    args += [str(reconsArgs['maximumAngle'])]
    args += [str(reconsArgs['maximumSurfaceAngle'])]
    args += [str(reconsArgs['minimumAngle'])]
    args += [str(reconsArgs['normalConsistency'])]
    args += [str(reconsArgs['consistentVertexOrdering'])]

    return args


def argument_poisson():
    name = 'poissonReconstruction'
    args = argument_initwithnormal(name)

    reconsArgs = cfg[name]
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

    return args

def argument_concave():
    name = 'concaveHull'
    args = argument_initwithnormal(name)

    reconsArgs = cfg[name]
    args += [str(reconsArgs['alpha'])]
    args += [str(reconsArgs['keepInformation'])]

    return args

def argument_convex():
    name = 'convexHull'
    args = argument_initwithnormal(name)

    reconsArgs = cfg[name]
    args += [str(reconsArgs['computeAreaVolume'])]

    return args

def argument_organizedfastmesh():
    name = 'organizedFastMesh'
    args = argument_mininit(name)

    reconsArgs = cfg[name]

    return args
