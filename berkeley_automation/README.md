# How to install [DQ-CNN](https://berkeleyautomation.github.io/gqcnn/install/install.html#ros-installation) and friends

The following procedure can approximatively work (some dependencies can be missing). In case of problems see [below](#Problems)

```
cd visualization && nano visualization/__init__.py 
sudo pip install --upgrade -e .

cd ../autolab_core && sudo pip install --upgrade  -e .

# Install good version of TF
sudo pip install --upgrade tensorflow==1.3.0

# Check the version
python -c “import tensorflow”

cd ../perception && sudo pip install --upgrade -e .

cd ../gqcnn && git checkout merge-jeff

sudo pip install --upgrade -e .

cd ../../ && catkin_make 
```


## Download model weights:
From https://berkeley.app.box.com/s/p85ov4dx7vbq6y1l02gzrnsexg6yyayb/folder/27403912897
Download file: GQCNN-3.0.zip

## Edit configurations 

In ~/gqcnn_ws/src/gqcnn/cfg/ros_nodes/grasp_planner_node.yaml 

Modify: 
```
policy:
   gqcnn_model: 

metric:
   gqcnn_model: /path/to/folder/GQ-Suction
```

## Run benchmark

```
cd src/gqcnn && sh scripts/evaluate_dex-net_4.0_suction_network.sh /home/aivengers/gqcnn_ws/src/gqcnn/gqcnn/GQ-Suction
```


## Run ros service 

```
cd gqcnn_ws && src && roslaunch gqcnn gqcnn.launch
```

## Problems

### Problem with `trimesh`

Comment the line : from .visualizer3d import Visualizer3D
and reinstall the package

### Problem to upgrade pyserial 

If fails to upgrade pyserial => remove package
```
sudo apt-get remove python-serial 
```

### Problem with `cv_bridge`

```
sudo apt-get install python-cv-bridge
```


