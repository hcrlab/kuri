#!/usr/bin/env sh

viz() {
    rviz_config=$(rospack find hcrl_kuri_launch)/config/kuri.rviz
    rosrun rviz rviz -d $rviz_config
}

viz_mapping() {
    rviz_config=$(rospack find hcrl_kuri_launch)/config/kuri_mapping.rviz
    rosrun rviz rviz -d $rviz_config
}

teleop() {
    rqt_config=$(rospack find hcrl_kuri_launch)/config/teleop.perspective
    rqt --perspective-file $rqt_config
}
