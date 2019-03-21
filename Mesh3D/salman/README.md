## Installation of environement: 
### CGAL dependencies.
run : 
```console
$ sudo apt-get install freeglut3
$ sudo apt-get install freeglut3-dev
$ sudo apt-get install binutils-gold
$ sudo apt-get install libglew-dev
$ sudo apt-get install mesa-common-dev
$ sudo apt-get install libeigen3-dev
$ sudo apt-get install libboost-all-dev
$ sudo apt-get install libmpfr-dev
$ sudo apt-get install libgmp-dev
$ sudo apt-get install libtbb-dev
$ sudo apt-get install zlib1g-dev libqglviewer-dev-qt5
$ sudo apt-get install libqglviewer-dev-qt5 
```
Install QT5.6 : 
from : http://download.qt.io/official_releases/qt/5.6/5.6.0/ 

download 

qt-opensource-linux-x64-5.6.0.run
in the terminal's path to the downloaded file and run : 
```console
$ chmod 755 qt-opensource-linux-x64-5.6.0.run
$ ./qt-opensource-linux-x64-5.6.0.run
```
install CGAL : 
run : 
```console
$ sudo apt-get install libcgal-dev
```
or run to build from source: 
```console
$ cd 
$ git clone https://github.com/CGAL/cgal.git
$ cd cgal
$ cmake . 
$ make 
$ make install
```
### OpenCV.
installation of OpenCV is well explained in this link : https://www.learnopencv.com/install-opencv-3-4-4-on-ubuntu-16-04/
### Netcdf:
The installation of netcdf-c is well explained in this link : https://www.unidata.ucar.edu/software/netcdf/docs/getting_and_building_netcdf.html

## Triangulation : 
in salman path run :
```console
$ mkdir build 
$ cd build 
$ cmake .. 
$ make 
$ ./Triangulation <image path>
```







