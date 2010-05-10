#!/usr/bin/python

from actions import *
from parameters import *

arm=arm()
arm_pr2=arm_pr2()
torso=torso()
tray=tray()

panels = [  
  ( "arm", [ 
	( "Stop", arm.Stop, ()),
	( "Home", arm.MoveTraj, (armParameter.home,)),
	( "Folded", arm.MoveTraj, (armParameter.folded,)),
	( "moveArm3", arm.MoveArm3, ("dadsasasfd","asdf")),
	]),
  ( "pr2_r_arm", [ 
	( "Home", arm_pr2.MoveTraj, (armParameter_pr2.home,)),
	( "Folded", arm_pr2.MoveTraj, (armParameter_pr2.folded,)),
	]),
  ( "torso", [ 
	( "Stop", torso.Stop, ()),
	( "Init", torso.Init, ()),
	( "Home", torso.MoveTraj, (torsoParameter.home,)),
	( "Front", torso.MoveTraj, (torsoParameter.front,)),
	( "Back", torso.MoveTraj, (torsoParameter.back,)),
	( "Left", torso.MoveTraj, (torsoParameter.left,)),
	( "Right", torso.MoveTraj, (torsoParameter.right,)),
	( "Shake", torso.MoveTraj, (torsoParameter.shake,)),
	( "Nod", torso.MoveTraj, (torsoParameter.nod,)),
	]),
  ( "tray", [ 
	( "Up", tray.MoveTraj, (trayParameter.up,)),
	( "Down", tray.MoveTraj, (trayParameter.down,)),
	]),
  ]
