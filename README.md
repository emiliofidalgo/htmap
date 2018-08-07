# HTMap: Hierarchical Topological Mapping

HTMap is an appearance-based approach for topological mapping based on a hierarchical decomposition of the environment. In HTMap, images with similar visual properties are grouped together in nodes, which are represented by means of an average global descriptor (PHOG) and an index of binary features based on an incremental bag-of-binary-words approach ([OBIndex](http://github.com/emiliofidalgo/obindex)). Each image is represented by means of a global descriptor and a set of local features, and this information is used in a two-level loop closure approach, where, first, global descriptors are employed to obtain the most likely nodes of the map and then binary image features are used to retrieve the most likely images inside these nodes. This hierarchical scheme allows us to reduce the search space when recognizing places, maintaining high accuracy when creating a map. The algorithm can be used to detect loop closures in a unknown environment, without the need of a training stage, as it is usual in BoW schemes.

This repository contains the code used in our IEEE-TRO [publication](http://ieeexplore.ieee.org/document/7938750/) called *Hierarchical Place Recognition for Topological Mapping*, but adapted to be used with OpenCV 3.x. Given the differentes between OpenCV versions, it might be difficult to reproduce *exactly* the same results of our paper. This code has been released for illustration purposes.

HTMap is released as a ROS package, and relies on [OpenCV 3.x](http://opencv.org), [OBIndex](http://github.com/emiliofidalgo/obindex) and [Boost](http://www.boost.org) libraries. Note that HTMap is research code. The authors are not responsible for any errors it may contain. **Use it at your own risk!**

# Conditions of use

HTMap is distributed under the terms of the [GPL3 License](http://github.com/emiliofidalgo/htmap/blob/master/LICENSE).

# Related publication

The details of the algorithm are explained in the [following publication](http://ieeexplore.ieee.org/document/7938750/):

**Hierarchical Place Recognition for Topological Mapping**<br/>
Emilio Garcia-Fidalgo and Alberto Ortiz<br/>
IEEE Transactions on Robotics, Vol. 33, No. 5, Pgs. 1061-1074 (Oct. 2017)<br/>

If you use this code, please cite:
```
@article{Garcia-Fidalgo2017,
  author={Emilio Garcia-Fidalgo and Alberto Ortiz},
  journal={IEEE Transactions on Robotics},
  title={Hierarchical Place Recognition for Topological Mapping},
  year={2017},
  volume={33},
  number={5},
  pages={1061-1074},
  doi={10.1109/TRO.2017.2704598},
  month={Oct}
}
```

# Installation

1. First of all, you need to install [OBIndex](http://github.com/emiliofidalgo/obindex) dependencies:
  ```
  sudo apt-get install libflann-dev libboost-system-dev libboost-filesystem-dev
  ```

2. Clone OBIndex in your workspace and change the current branch to *kinetic*:
  ```
  cd ~/your_workspace/src
  git clone http://github.com/emiliofidalgo/obindex.git
  cd obindex
  git checkout kinetic
  ```

3. Clone HTMap in your workspace:
  ```
  cd ~/your_workspace/src
  git clone http://github.com/emiliofidalgo/htmap.git
  ```

4. Compile the packages using, as usual, the `catkin_make` command:
  ```
  cd ..
  catkin_make -DCMAKE_BUILD_TYPE=Release
  ```

5. As a final step, HTMap requires a writable directory where it stores results and serializes some data. Therefore, we recommend to create one for this purpose at this point:
  ```
  mkdir -p ~/Desktop/htmap
  ```

# Usage

- To run HTMap, use the provided launch file:
  ```
  roslaunch htmap htmap.launch image_dir:="/image/dir" working_dir:="~/Desktop/htmap"
  ```
  where *image_dir* is a directory containing the sequence of images to be processed and *working_dir* is the directory where the algorithm will store their results and serialization data. This might be the directory we created during the installation procedure.

  There exist several execution parameters that can be configured when running HTMap. See next section for a description of each of them.

- During the execution, screen will show information about the algorithm, indicating when a loop closure is found.

- After a succesful execution, HTMap generates, as output, several text files. These files are stored in the *working_dir/results* directory. To speed up the mapping process, some of them are empty, since the corresponding parts of the code are commented. They can be uncommented if needed. The most important files are:

    - *htmap_loops_ninliers.txt*, which is a binary matrix where entry (*i, j*) is 1 if image *i* and image *j* can be considered as the same place, and 0 otherwise. Therefore, the *i-th* row represents the possible loop closures for image *i*. The name *ninliers* refers to the value of the parameter used as the minimum number of inliers to accept a loop. See the paper for further details.

    - *htmap_img2loc_ninliers.txt*, where each line contains the identifier of each image and the location to which that image was assigned during the process.

- We provide several Matlab scripts to process the resulting files. Along with these scripts, we also provide ground truth files for the datasets used in the paper. If you want to use your own dataset, you should generate your own ground truth file using the same format.

- In order to obtain a precision / recall value, open Matlab and select the provided *matlab* directory as your working directory. Next, use the following sequence of commands, adapting, of course, the paths (*CC* stands for the *City Centre* dataset):
```
loop_file = '~/Desktop/htmap/results/htmap_loops_20.txt';
gt_file = 'gtruths/groundtruth_CC.mat';
PR(loop_file, gt_file, true);
```

- You can also generate a precision / recall curve after obtaining a set of precision / recall values. In our original paper, these curves were generated modifying the minimum number of inliers to accept a loop. To do that:
    - Set the parameter *batch* to *true*.
    - Set the parameter *inliers_begin* to the number of inliers that you want to use at the first execution of HTMap.
    - Set the parameter *inliers_end* to the number of inliers where the process will stop.
    - Set the parameter *inliers_step* to the number of inliers to increment between each execution of HTMap.


- Running HTMap under these conditions will execute the algorithm several times modifying the number of inliers and storing the corresponding files in the *results* directory. The precision / recall curve can be obtained using the following sequence of commands, adapting, again, the paths (*CC* stands for the *City Centre* dataset):
```
res_dir = '~/Desktop/htmap/results/';
gt_file = 'gtruths/groundtruth_CC.mat';
PR_curve(res_dir, gt_file, true);
```

- Contact the authors for more information about the other figures in the article.

# Contact

If you have problems or questions using this code, please contact the author (emilio.garcia@uib.es). [Feature requests](http://github.com/emiliofidalgo/htmap/issues) and [contributions](http://github.com/emiliofidalgo/htmap/pulls) are totally welcome.
