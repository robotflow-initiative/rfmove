import sys

import moveit_noros

sys.path.append("../install/lib")
import numpy
import moveit_noros as moveit

child = moveit_noros.ExampleChild("A")
print(child.GetName())
moveit_noros.changeChildName(child, "B")
print(child.GetName())