# Turtlelib Description

TurtleLib is a C++ library for performing 2D rigid body transformations, geometry primitives, and visualizations. It is designed as a standalone library (ROS independent) for use in projects that require 2D frame transformations, unit conversions, and SVG-based visualization.

## Features:
#### `angle.hpp`
Provides functions to convert and normalize angles between **degrees** and **radians**, ensuring they remain within standard ranges:  
- Degrees: (-180, 180]  
- Radians: (-π, π]

#### `geometry2d.hpp`
Defines 2D geometry primitives such as points and vectors, including:  
- Arithmetic operators (`+`, `-`, `*`)  
- Stream I/O operators (`<<`, `>>`)  
- Utility functions for basic 2D vector math.

#### `se2d.hpp`
Defines SE(2) 2D rigid body transforms, allowing:  
- Transformations of Points, Vectors, and Twists.  
- Inversion of transforms.

## Executables: 
#### `converter.cpp`
Command-line program that prompts the user for an angle and unit (`deg` or `rad`), converts it to the other unit, normalizes it, and outputs the result. Loops until EOF (CTRL-D) is entered.

#### `frame_main.cpp`
Interactive program for entering 2D transforms, points, and vectors:  
- Computes coordinates in multiple frames, and outputs results to stdout.
- Draws frames, points, and vectors to an SVG file (`/tmp/frames.svg`) for visualization
