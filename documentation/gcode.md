# Gcode

The gcode interpreter can handle give different commands, G0-G5. The circular commands, G02 and G03, are converted into linear commands, G01.

## Adding gcode files
Adding gcode files can be done either via the gui or by simply placing the gcode files into ```./python/g_code/g_codes/```. The simplest way of converting images to gcode is to use [inkscape](https://inkscape.org/). [This](https://www.youtube.com/watch?v=bbe56S_O-uI) video shows the steps necessary.

NOTE: The intention for the gcode is to have it move at only two different heights, z-values. At the first height the pen is paiting, this is called _draw_height_. The second height is when the pen is moving to a new location to paint and is called _move_height_. Both of these should be specified in the g_code config file. They're preset to 1 and 10.



