How to Run: (artery_ROS/artery)

- cmake --build build --target run_gazebo-platoon
or
- cmake --build build --target run_gazebo-platoon > output.txt
gedit output.txt
or
- cmake --build build --target debug_gazebo-platoon (w/ gdb usage)


When changing directories tree:

- delete cmake caches

- cd artery_ROS/artery

- make vanetza + make inet + make veins (delete cmake caches if needed)

- delete /build

- mkdir build

- cd build

- cmake ..

- *Run*
