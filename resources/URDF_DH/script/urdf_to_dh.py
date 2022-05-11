#!/usr/bin/python


import sys
import pandas as pd
import numpy as np
import openravepy
from openravepy import Environment,planningutils

if __name__ == "__main__":
    env=Environment()
    env.StopSimulation() # don't want another thread distributing things

    env.Load("../data/panda.dae",{'skipgeometry':'1'})
    body=env.GetBodies()[0]
    plugin=planningutils.GetDHParameters(body)
    dhlist=[]
    transformlist=[]
    for joint in plugin:
        dh=[]
        dh.append(joint.a)
        dh.append(joint.alpha)
        dh.append(joint.d)
        dh.append(joint.theta)
        dhlist.append(dh)
        transformlist.append(joint.transform)
    npdhdata=np.array(dhlist)
    dhdata=pd.DataFrame(npdhdata,columns=["a","alpha",'d',"theta"])
    dhdata.to_csv("../data/dhdata.csv",sep=",")

    npdhtransform=np.array(transformlist)
    dhtransform=pd.DataFrame(npdhtransform.reshape(-1,16))
    print(dhtransform)
    dhtransform.to_csv("../data/transform.csv",sep=",")
    