#  +---------------------------------------------------------------------------+
#  |                              			                                   |
#  |               https://sites.google.com/site/bellonemauro/                 |
#  |                                                                           |
#  | Copyright (c) 2015, - All rights reserved.                                |
#  | Authors: Mauro Bellone                                                    |
#  | Released under BDS License.                                               |
#  +---------------------------------------------------------------------------+ 


This application has been developed for research purposes.

The external libraries for this applications are:
PCL-1.7
VTK
QT-4.8.6


IMPORTANT : if you want to build this app it is necessary to build VTK with QT module

known issues:
1) Point Cloud visualization does not work properly with PCL-1.8 
	
=======
PCL 1.7
VTK
QT4	--- QVTK is mandatory!!!


Building Instructions:

WRITE ME !!!
it compiles (I have tested) on MSVC2012 and ubuntu 14.04

the only problem which I noted is related to the qvtk package,
the QVTK package is required to run any pcl application with a qt visualizer. 
take a look here for further details: 
http://pointclouds.org/documentation/tutorials/qt_visualizer.php

