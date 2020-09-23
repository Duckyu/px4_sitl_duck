![ui](./img/ui.png)

Before you use this package as ros package, you need to follow some step below.

***
	<HOW-TO-USE GAZEBO-SITL-SIMULATION>
	1. cd $(rospack find muin_px4)/Firmware
	2. make px4_sitl_default
	3. cd (path of your catkin workspace) && catkin_make
	4. chmod 777 muin_simulation.sh
	5. ./muin_simulation.sh

	+++ if you want to change world of simulation, you can change the world name in muin_integral.launch. Or else you may add an option on last command in 'muin_simulation.sh'(for example add "world:=$(find muin_px4)/worlds/iris.world").
***
