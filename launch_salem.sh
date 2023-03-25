#!/bin/bash
xterm -title "Drivers kobuki" -e bash -c "cd ~/kobuki_base_ws; source install/setup.bash; ros2 launch kobuki_node kobuki_node-composed-launch.py" &
xterm -title "Cheese Service" -e bash -c "cd ~/ros2_ws; . install/setup.bash; cd src; python3 cheese_action_server.py $1"  &
xterm -title "CUI Service" -bg black -e bash -c "cd ~/ros2_ws; . install/setup.bash; ros2 run cui service" &
xterm -title "Navegación" -e bash -c "cd ~/ros2_ws; . install/setup.bash; cd src/navegacion_autonoma/navegacion_autonoma/; python3 mover.py" &
xterm -title "Control" -e bash -c "cd ~/ros2_ws; . install/setup.bash; ros2 run control control $1" 