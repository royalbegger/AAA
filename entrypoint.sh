#!/bin/bash
source /jackal_ws/devel/setup.bash
cd /jackal_ws/src/AAA
exec ${@:1}
