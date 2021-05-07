# minau_tools

## Repeat Tester

Currently the repeat tester is set up with 2 blue_rovs searching a rectangular region for an enemy asset. The blue_rovs go to random waypoints,
constantly estimating their pose and the poses of the other blue_rov and the enemy asset. The truth poses and etddf estimates are bagged to be
analyzed later.

To use the repeat tester adjust 'config/repeat.csv', specifing the name of the test, how long the test will occur, and the dimensions of the search
zone. Note that each test must have a unique "Test_Group_Name" as the script makes a directory with this name to store the data in.

Once you have set up your configuration start up the sim:

```
roslaunch minau_tools scenario.sitl.red_intruder.repeat.ocean.launch
```

Press play on the sim and then run

```
roslaunch minau_tools repeat.launch
```

To start the repeat tester.

### Data Analyzer

To analyze the bagged data from the repeat tests all you need to do is use the scripts/repeat_data_analyzer.py script. This extracts the average estimate error and the standart deviation of the error for each test.

```bash
cd scripts
./repeat_data_analyzer.py repeat_data/<YOUR TEST NAME> <YOUR TEST X DIM> <YOUR TEST Y DIMENSION> <NUMBER OF BLUE ASSETS IN YOUR TEST>
```

This will create a csv in the directory of the specifed test with all the errors and standard deviations.

### Hyper Timing

To play around with hyper timing, go to 'minau_tools/worlds/ocean_waves_hyper.world' and adjust the settings for the physics engine. There are a
couple things you can do. First you can increase the step size, this will make is so the physics sim has a larger step size and can run quicker.
The sim seems to crash with step size above 0.01. You can also increase the real_time_update_rate. This will allow your computer to update things
at a quicker rate if it is powerful enough to do so.

Once you have made changes to max_step_size and real_time_update_rate, also update real_time_factor by multiplying the max_step_size and
real_time_update_rate.
