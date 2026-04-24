
## bmk3

- Ran the test_camera_orientation script on the urban04 dataset from 125 to 250 and 1000 to 1125
- Examined how the weighted rmse changes for different pitch and roll angles
- This file is only partial, there is some missing data that has not been computed yet
- It ran for 20 hours! and only got through like 30 frames....
- This one was a problem!
- I hadn't reset the yaw offset each time a new pitch and roll orientation was checked
- So, the yaw offset ended up getting really large and all the data was useless

## bmk4

- The hopefully final need for another dataset
- Everything the same as bmk3 but hopefully just works.

## bmk5

- Ran the test_camera_orientation on the urban01 dataset
- hoping that this resolves the problems with that run
- also used more granular pitch and roll resolution

