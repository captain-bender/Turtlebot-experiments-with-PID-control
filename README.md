# Turtlesim-with-P-controller


## Useful commands

Build the workspace:
```$
colcon build --symlink-install
```

Source the workspace:
```$
source install/setup.bash
```

Launch the simualtion:
```$
ros2 launch cps_pid_turtle follow_shape.launch.py
```

Run the odom analyser:
```$
ros2 run cps_pid_turtle analyse_odom
```

Record the bag:
```$
ros2 run cps_pid_turtle analyse_odom
```

Play the bag:
```$
ros2 bag play run --clock
```

Examine the bag:
```$
ros2 bag info run
```

Run the rqt-plot:
```$
ros2 run rqt_plot rqt_plot
``` 

### Author
Angelos Plastropoulos