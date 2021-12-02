import sys
sys.path.append("../install/lib")
import moveit_noros as moveit
import numpy as np
import math

affine = moveit.EigenAffine3d()
print(affine)

print("reset")
affine.reset()
print(affine)

print("rotate pi/3")
affine.rotate(math.pi/3, np.array([1,0,0]))
print(affine)

print("translate 1 2 3")
affine.translate(np.array([1,2,3]))
print(affine)

print("translate -3 -2 -1")
affine.translate(np.array([-3, -2, -1]))
print(affine)

print("set translation 4 5 6")
affine.setTranslation(np.array([3,4,5]))
print(affine)

print("set translation 9 8")
affine.setTranslation(np.array([9,8]))
print(affine)
