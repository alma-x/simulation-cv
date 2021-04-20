#!/usr/bin/env python
aruTargetDict={'panelSwitch1':(101,
                        40)
                ,'panelSwitch2':(102,
                        40)
                ,'panelSwitch3':(103,
                        40)
                ,'panelSwitch4':(104,
                        40)
                ,'panelSwitch5':(105,
                        40)
                ,'panelSwitch6':(106,
                        40)
                ,'panelSwitch7':(107,
                        40)
                ,'panelSwitch8':(108,
                        40)
                }

global targetList
targetList=['panelSwitch8','panelSwitch7','panelSwitch6','panelSwitch5','panelSwitch4'
            ,'panelSwitch3','panelSwitch2','panelSwitch1']

global targetCounter
targetCounter=0
global findNewTarget
findNewTarget=1



global targetMarkId,targetMarkSize
print(aruTargetDict['panelSwitch3'])
global selectionString
selectionString=targetList[5]
print(aruTargetDict[selectionString])
print(aruTargetDict[selectionString]==aruTargetDict['panelSwitch3'])

# apparently aruTargetDict[targetList[4]] it's different, even if prints ==