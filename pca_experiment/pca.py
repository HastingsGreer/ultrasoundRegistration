import numpy as np
import vtk
from PIL import Image
from vtk.util.numpy_support import vtk_to_numpy

import matplotlib.pyplot as plt

from sklearn.decomposition import PCA
def getNumpyArray(fname):
    imr = vtk.vtkMetaImageReader()
    imr.SetFileName(fname)
    imr.Update()

    im = imr.GetOutput()
    rows, cols, layers = im.GetDimensions()
    sc = im.GetPointData().GetScalars()
    a = vtk_to_numpy(sc)

    a = a.reshape(layers, cols, rows)
    return a
    

def pcaFilename(fname):
    a = getNumpyArray(fname)
    print a.shape
     
    transposeA = a.transpose(0, 2, 1).reshape(-1, a.shape[1])
    
    P = PCA(n_components=6).fit(transposeA)
    transformedData = P.transform(transposeA)
    return transformedData, P
def pca2Filenames(f1, f2):
    a1 = getNumpyArray(f1)[:, 110:380, :]
    a2 = getNumpyArray(f2)[:, 110:380, :]
    return pca2Images(a1, a2)

def pca2Images(a1, a2, components = 4):
    print(a1.shape)
    transposeA1 = a1.transpose(0, 2, 1).reshape(-1, a1.shape[1])
    transposeA2 = a2.transpose(0, 2, 1).reshape(-1, a1.shape[1])
    
    combined = np.append(transposeA1, transposeA2, axis=0)
    P = PCA(n_components=components).fit(combined)
    t1 = P.transform(transposeA1).reshape(-1, a1.shape[2], components)
    t2 = P.transform(transposeA2).reshape(-1, a1.shape[2], components)
    return t1, t2, P    

def showPCA(transformedData, P):
    for pic, component in zip(range(len(P.components_)), P.components_):
        plt.plot(component)
        plt.show()
        
        plt.imshow(pic.reshape(-1, 512))
        plt.axes().set_aspect(12)
        print "a"
        plt.show()
        
def show2PCA(t1, t2, P):
    for idx, component in zip(range(len(P.components_)), P.components_):
        
        
        plt.plot(component)
        plt.show()
        plt.imshow(t1[:,:,idx], cmap="Greys")
        plt.show()
        plt.imshow(t2[:,:,idx], cmap="Greys")
        plt.show()
        
