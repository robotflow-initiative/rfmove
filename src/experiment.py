import sys
sys.path.append("../install/lib")
import numpy
import moveit_noros as moveit

child = moveit.ExampleChild("aaa")
print(child.GetName())
moveit.changeChildName(child,"bbb")
print(child.GetName())

vec = [1,2,3]
print(vec)
moveit.changeVectorContains(vec)
print(vec)