### Note for gazebo plugin compiling

My system is ubuntu 14.04 with ros-indigo

When I build this, the errors show "no such file" things to the gazebo.hh and sdf.h

To solve this kind of error, we can just do the simple "ln -s" command like..

``` bash
ln -s /usr/include/gazebo-2.2/gazebo/ /usr/include/gazebo/
ln -s /usr/include/sdformat-1.4/sdf/ /usr/include/sdf
```

### compal_gazebo_multi.launch

This launch file needs all agv_control and agv_description pkg.

If it runs sucessfully, we can do: 

1. move agv2 by arrow key
2. push the lifter by using the oringinal topic /robot/lifter_motor
