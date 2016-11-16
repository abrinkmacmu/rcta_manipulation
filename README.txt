To run PR2 robot************************

roslaunch rcta bringup_pr2_custom.launch







To run Roman robot************************

cd <to_rcta_manipulation>/scripts
screen -c screenrc_sim_fake

(the accelerator for this screen config is ctrl+x but workstation screen config is ctrl+a)







Configuration Robot starting positions ************************************

Change the args of these two lines in "bringup_pr2_custom.launch":

<node pkg="tf" type="static_transform_publisher" name="pr2_base_pub" args="0 1.5 0 0 0 0 world pr2/base_footprint 10"/>

<node pkg="tf" type="static_transform_publisher" name="roman_base_pub" args="0 0 0 1.57 0 0 world abs_nwu 10"/>



IK Server test**************************************

rosrun narms ik_test
	This will take both robots through 100 random poses in their shared workspace




