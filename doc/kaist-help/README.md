# kaist-help

A tool to convert KAIST urban dataset to rosbag, and recover the `ring` and `time` filed of LiDAR pointcloud for [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) / [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) using.

---



## How to use

1. Download desired database files from the [website](https://sites.google.com/view/complex-urban-dataset).

2. Extract `.gz.tar` files into their folders.

   ```
   find . -name 'urban28*.tar.gz' -execdir tar -xzvf '{}' \;
   ```

3. Make ros workspace and build.

   ```
   mkdir -p ~/kaist-help-ws/src
   cp -r kaist-help ~/kaist-help-ws/src
   cd ~/kaist-help-ws
   catkin build
   ```

4. Edit the `kaist2bag/config/config.yaml` with path.

   ```
   # directory of the raw KAIST dataset
   dataset: "/mnt/F/DataSet/KAIST_Urban_dataset/urban26/urban26-dongtan"
   # directory of the rosbag file to save
   save_to: "/mnt/F/DataSet/KAIST_Urban_dataset/urban26/urban26-dongtan"
   ```


5. Create a rosbag file for each sensor type.

   ```
   source devel/setup.bash
   roslaunch kaist2bag kaist2bag.launch
   ```

6. Merge all bags into a single one (if desired).

   ```
   rosrun kaist2bag mergebag.py merged.bag <bag_file_1> ... <bag_file_8>
   ```

---




## Acknowledge

- The main package is from [kaist2bag](https://github.com/tsyxyz/kaist2bag).
- The code of recovering `ring` and `time` filed is adapted from [clins](https://github.com/APRIL-ZJU/clins).



