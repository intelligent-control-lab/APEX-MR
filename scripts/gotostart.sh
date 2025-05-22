#!/bin/bash

# read the argument, if no argument, run the first two rosservice
if [ "$1" == "home" ]; then
    rosservice call /yk_destroyer/yk_set_joints "state:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
      name: ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
      position: [0, -0.26975886, -0.70435494, 0, -1.13620025, 0] 
      velocity: [0, 0, 0, 0, 0, 0]
      effort: [0, 0, 0, 0, 0, 0]"


    rosservice call /yk_architect/yk_set_joints "state:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
      name: ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
      position: [0, -0.26975886, -0.70435494, 0, -1.13620025, 0] 
      velocity: [0, 0, 0, 0, 0, 0]
      effort: [0, 0, 0, 0, 0, 0]"
else
    rosservice call /yk_destroyer/yk_set_joints "state:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
      name: ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
      position: [0, 0, 0, 0, -1.5714, 0]
      velocity: [0, 0, 0, 0, 0, 0]
      effort: [0, 0, 0, 0, 0, 0]"


    rosservice call /yk_architect/yk_set_joints "state:
      header:
        seq: 0
        stamp: {secs: 0, nsecs: 0}
        frame_id: ''
      name: ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
      position: [0, 0, 0, 0, -1.5714, 0]
      velocity: [0, 0, 0, 0, 0, 0]
      effort: [0, 0, 0, 0, 0, 0]"
fi

