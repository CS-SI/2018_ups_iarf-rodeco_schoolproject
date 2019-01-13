cfg = {}

cfg['out_dir'] = "../reconstructed"

#Default normalReconstruction argument
cfg['normalEstimation'] = {}
cfg['normalEstimation']['useKtree'] = True
cfg['normalEstimation']['KSearch'] = 8

#Default poissonReconstruction argument
cfg['poissonReconstruction'] = {}
cfg['poissonReconstruction']['depth'] = 4
cfg['poissonReconstruction']['minDepth'] = 1
cfg['poissonReconstruction']['solverDivide'] = 8
cfg['poissonReconstruction']['isoDivide'] = 8
cfg['poissonReconstruction']['degree'] = 2
cfg['poissonReconstruction']['pointWeight'] = 1.0
cfg['poissonReconstruction']['scale'] = 1.25
cfg['poissonReconstruction']['samplePerNode'] = 1.0
cfg['poissonReconstruction']['confidence'] = False
cfg['poissonReconstruction']['outputPolygon'] = False
cfg['poissonReconstruction']['manifold'] = False
