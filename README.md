# Gazebo exploration

## How to launch
```bash
./launch_as2.bash
```

By default launches one drone exploration in word instance 60 (60 obstacles in 20x20m). Check options to change launching.
```bash
$ ./launch_as2.bash -h
Usage: ./launch_as2.bash [-i <instance>] [-2] [-3] [-r] [-t]
  options:
      -i: choose world instance (default: '60')
      -2: launch two drones
      -3: launch three drones
      -r: record rosbag
      -t: launch keyboard teleoperation
```

## How to start exploration
```bash
# Go to last tmux window (or open new terminal)
python explore.py
```

## How to stop
```bash
# On one tmux window
./kill.bash
```

## Known issues
- Sometimes Gazebo process is not killed properly. In that case, run `killall gzserver` and `killall gzclient` manually.
