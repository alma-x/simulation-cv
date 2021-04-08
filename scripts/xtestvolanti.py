#!/usr/bin/env python3
# -*- coding: utf-8 -*-
arucoObjectDict={'cube5s':(582,#id
                            40),#size
#                'cube5d:(582,
#                          40),
#                'cube10d':(582,
#                            90),
                'centrifugaBase': (273,
                                   100),
                'centrifugaAxial': (429,
                                    100), 
                'centrifugaTangent': (221,
                                      100)               
                } 

targetObject='cube5s'
targetId,targetSize=arucoObjectDict[targetObject]
print(targetId,targetSize)