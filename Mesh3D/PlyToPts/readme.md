## Power Crust 

The original implementation of Power Crust algorithem under GPL license was made available as a tar ball by the authors at:
```console
git clone https://github.com/kubkon/powercrust.git
```
Amenta, Choi and Kolluri, The power crust, 6th ACM Symposium on Solid Modeling, 2001, pages 249-260
Amenta, Choi and Kolluri, The power crust, unions of balls, and the medial axis transform, Computational Geometry: Theory and Applications, 2001, 19:(2-3), pages 127-15

## Ply to Pts 

The original implementation of Power Crust uses the .pts File Format, but our and most common algorithm render the output as .ply File Format.
this script converts the .ply File Format  to.pts in order to perform the 3D reconstruction.

## Building the programs

In the ply to pts directory run:

```console
$ mkdir build
$ cd build
$ cmake ..
$ make
```

In the Power crust directory run:
```
$ make
```

To clean up, run:

```console
$ make clean
```

#Running the programs

in the ply to pts directory build run : 

```console
$ pts_writer ./[filename.ply]
```
the result is written in converted.pts


in powercrust/target directory run : 
```console
$ powercrust -i [filename.pts]
```


