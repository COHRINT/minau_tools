if [ -z "$1" ]
then
      echo "./tune_pid.sh guppy depth 0.1"
      echo "Args: Name of agent, axis, setpoint"
      echo "Axes: depth, heading, x_vel, y_vel"
      exit 0
elif [ -z "$2" ]
then
    echo "./tune_pid.sh guppy depth 0.1"
    echo "Args: Name of agent, axis, setpoint"
    echo "Axes: depth, heading, x_vel, y_vel"
    exit 0
elif [ -z "$3" ]
then
    echo "./tune_pid.sh guppy depth 0.1"
    echo "Args: Name of agent, axis, setpoint"
    echo "Axes: depth, heading, x_vel, y_vel"
    exit 0
fi
rosservice call /$1/uuv_control/arm_control

# rosservice call /guppy/uuv_control/set_heading_depth {"heading":0.0}, {"depth":0.0}
# rosservice call /{}/uuv_control/set_heading_depth 'heading: heading_setpoint + \ndepth: 
rosservice call /guppy/uuv_control/set_heading_depth "heading: 0.0\n depth: 0.0"