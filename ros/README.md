# Singularity Container for HTMap

Singularity container for HTMap. Contains ros with opencv and boost.

## Usage
Use `singularity build ros.sif ros.def` to get the singularity image file.

Once the .sif is built, you will need to use the container for [installing](https://github.com/emiliofidalgo/htmap#installation) `htmap` and `obindex`:

```
singularity exec ros.sif catkin_make -DCMAKE_BUILD_TYPE=Release
```

Then to [execute](https://github.com/emiliofidalgo/htmap#installation) htmap, source the :

```
singularity shell ros.sif
> . devel/setup.sh
> roslaunch htmap htmap.launch image_dir:="/image/dir" working_dir:="~/Desktop/htmap" other_options:="value"
```
