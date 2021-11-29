rosservice call /guppy/mission_manager/enable_mission "{}"
rosservice call /dory/mission_manager/enable_mission "{}"
rosservice call /bruce/mission_manager/enable_mission "{}"
rosservice call /bubbles/mission_manager/enable_mission "{}"
rosservice call /squirt/mission_manager/enable_mission "{}"
rosservice call /red1/uuv_control/arm_control
rosservice call /red1/plan_manager/load_plan "filename: '/home/lochniss/minau/src/onboard/config/mission_description_cc_demo.xml'"
rosservice call /red1/plan_manager/arm_plan "start_time: ''"