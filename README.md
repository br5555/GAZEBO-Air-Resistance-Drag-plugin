# GAZEBO-Air-Resistance-Drag-plugin

Add to ~/.bachrc line 
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:<PATH TO GENERATED .so file>
```
example ( gazebo_air_resistance_plugin_collision.so is in ~/gazebo_model_plugin/build) 
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_model_plugin/build
```
