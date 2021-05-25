#!/usr/bin/env sh

keyteleop() {
    rosrun kuri_teleop keyboard_teleop "$@"
}

teleop() {
    rqt_config=$(rospack find kuri_teleop)/config/teleop.perspective
    rqt --perspective-file $rqt_config
}