# HTMap: Hierarchical Topological Mapping

HTMap is an appearance-based approach for topological mapping based on a hierarchical decomposition of the environment. In HTMap, images with similar visual properties are grouped together in nodes, which are represented by means of an average global descriptor (PHOG) and an index of binary features based on a bag-of-words online approach [(OBIndex)](http://github.com/emiliofidalgo/obindex). Each image is represented by means of a global descriptor and a set of local features, and this information is used in a two-level loop closure approach, where first global descriptors are employed to obtain the most likely nodes of the map and then binary image features are used to retrieve the most likely images inside these nodes. This hierarchical scheme allows us to reduce the search space when recognizing places, maintaining high accuracy when creating a map.

The algorithm can be used to detect loop closures in a unknown environment, without the need of a previous training stage.

HTMap is released as a ROS package, and relies on ([OpenCV](http://opencv.org), [OBIndex](http://github.com/emiliofidalgo/obindex) and [Boost](http://www.boost.org) libraries. Note that HTMap is research code. The authors are not responsible for any errors it may contain. **Use it at your own risk!**
