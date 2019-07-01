# GAZEBO-Air-Resistance-Drag-plugin



## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

 * CMAKE
 * C++ compiler
 * [GAZEBO](http://gazebosim.org/) 

### Installing

Add to ~/.bachrc line 
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:<PATH TO GENERATED .so file>
```
example ( gazebo_air_resistance_plugin_collision.so is in ~/gazebo_model_plugin/build) 
```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_model_plugin/build
```

## License

This project is licensed under the GNU License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Drag for every kind of fluids
