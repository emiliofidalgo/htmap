# HTMap: Hierarchical Topological Mapping

HTMap is an appearance-based approach for topological mapping based on a hierarchical decomposition of the environment. In HTMap, images with similar visual properties are grouped together in nodes, which are represented by means of an average global descriptor (PHOG) and an index of binary features based on an incremental bag-of-binary-words approach ([OBIndex](http://github.com/emiliofidalgo/obindex)). Each image is represented by means of a global descriptor and a set of local features, and this information is used in a two-level loop closure approach, where, first, global descriptors are employed to obtain the most likely nodes of the map and then binary image features are used to retrieve the most likely images inside these nodes. This hierarchical scheme allows us to reduce the search space when recognizing places, maintaining high accuracy when creating a map. The algorithm can be used to detect loop closures in a unknown environment, without the need of a training stage, as it is usual in BoW schemes.

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

5. As a final step, HTMap requires a writable directory where it stores results and serializes some data. Therefore, we recommend to create one for this purpose:
  ```
  mkdir -p ~/Desktop/htmap
  ```