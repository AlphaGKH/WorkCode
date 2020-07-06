#！/bin/bash
cd ../bin

gnome-terminal -- bash -c "./display_main* --log_dir=../log/" &
gnome-terminal -- bash -c "./perception_main* --log_dir=../log/" &
gnome-terminal -- bash -c "./planning_main* --log_dir=../log/" &
#gnome-terminal -- bash -c "./control_main* --log_dir=../log/" &
