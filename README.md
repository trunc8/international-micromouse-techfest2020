# International Micromouse Competition
## Techfest 2020

**Team ID**: MC-209989

#### Python Dependency
- numpy

#### Maze Details
The space between walls is 16cm and the walls are all 0.02m thick (0.01m on each side). So the cell size is 18cm\*18cm with walls and 16cm\*16cm without walls. 
Also there are 16 columns and 16 rows in the maze. Thus the maze is of dimensions 16\*16 with each cell of dimension 18\*18. So, the number of cells is 256.

#### Demo Video

[Link](https://youtu.be/ofqWxsuaTu0)

#### How to run
Spawn bot at default location-
```
roslaunch pkg_techfest_imc final.launch
```

Spawn bot at ( {desired x}, {desired y} )-
```
roslaunch pkg_techfest_imc final.launch initial_position_micromouse_x:={desired x} initial_position_micromouse_y:={desired y}

```
