## Robotic Vision course from etfos.unios.hr  

### Lab assignments

Assignments and reports are written in Croatian, 
but code and comments are in English  

### Compiling
Easy way is to let pkg-config list all opencv libraries to g++

Code can be compiled using: 

    g++ fooBar.cpp -o fooBinary `pkg-config --cflags --libs opencv`

or for DEBUG output

    g++ -DDEBUG fooBar.cpp -o fooBinary `pkg-config --cflags --libs opencv`

### Lab1 
Intro to OpenCV library   
Canny edge detector  
Croping pictures  
Initializing camera

### Lab2
Template matching

### Lab3
Hough transform

### Lab4
Feature matching 

### Lab5
3D reconstruction

### Lab6
Determining dominante plane 

