#! /bin/sh
template=$(dirname $0)
ur_description=$(rospack find ur_description)
ur_gazebo=$(rospack find ur_gazebo)

for robot_model in $(find $ur_description/config/* -type d -exec basename {} \;)
do
	sed -e "s/\${robot_model}/$robot_model/" $template/urx.xacro > $ur_description/urdf/${robot_model}.xacro
	sed -e "s/\${robot_model}/$robot_model/" $template/urx_macro.xacro > $ur_description/urdf/inc/${robot_model}_macro.xacro
	sed -e "s/\${robot_model}/$robot_model/" $template/view_urx.launch > $ur_description/launch/view_${robot_model}.launch
	sed -e "s/\${robot_model}/$robot_model/" $template/test_urx.launch > $ur_description/launch/test_${robot_model}.launch
	sed -e "s/\${robot_model}/$robot_model/" $template/load_urx.launch > $ur_description/launch/load_${robot_model}.launch
	sed -e "s/\${robot_model}/$robot_model/" $template/load_urx.launch > $ur_gazebo/launch/load_${robot_model}.launch
	sed -e "s/\${robot_model}/$robot_model/" $template/urx_bringup.launch > $ur_gazebo/launch/${robot_model}_bringup.launch
	sed -e "s/\${robot_model}/$robot_model/" $template/test_urx_gazebo.launch > $ur_gazebo/launch/test_${robot_model}.launch
done
