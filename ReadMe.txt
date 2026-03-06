working with claude sonnet4.6

made two files one urdf and the other html webplayer ( the webplayer worked, the urdf didnt)

we moved to zip files 

this is the first zip , it runs rviz sim 

tweaks :
- finger turn weird 
- base joint only revolves around z 

good:
- it looks like an arm  
- mostly works
-hopeful

commands Used :

ros2 launch urdf_tutorial display.launch.py model:=$HOME/Desktop/Arm/robot_arm_description/urdf/robot_arm.urdf
