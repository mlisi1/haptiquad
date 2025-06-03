# HaptiQuad
## Description
HaptiQuad is a C++ library implementing the work done in the paper [Residual-based contacts estimation for humanoid robots](https://ieeexplore.ieee.org/document/7803308). 
The purpose of this library is to produce residuals calculations and provide a way to estimate external forces based on those residuals.
The main change in the theory behind it is that, differently from the paper, this library does not expects a F/T sensor, thus estimating ground reaction forces.\\
It can be used with any floating base model, as long as all the informations are in the correct format. An example of its usage, as well as its ROS2 wrapper can be foound in [haptiquad_ros2](https://github.com/mlisi1/haptiquad_ros2/tree/main).
The library is mostly based on [Pinocchio](https://github.com/stack-of-tasks/pinocchio) and [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page).

## Installation
To install the library, the [`colcon`](https://colcon.readthedocs.io/en/released/) building tool is necessary, as the library is mostly thought to be used along ROS2, however it is also possible to use it as a standard library.
```bash
mkdir src && cd src
git clone git@github.com:mlisi1/haptiquad.git
cd haptiquad
git submodule init --recursive
cd ../..
colcon build
source install/setup.bash
```
After this, the library can be used normally.

## Usage

The main implementations are in `momentum_observer.hpp` and `force_estimator.hpp`, which respectively handle the residual calculation and GRFs estimation. To use both, a Pinocchio model of the floating base robot is needed; this has to be modified to add a 6DOFs joint between the floating base and a virtual added fixed base. The usual way to do this is to have an URDF description of the robot and add these lines:
```XML
<link name="fixed_base"></link>
<joint name="virtual" type="floating">
  <parent link="fixed_base"/>
  <child link="BASE_LINK_NAME"/>
  <origin rpy="0 0 0" xyz="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```
All the implementation is based on Spatial Algebra used in the Rigid Body Dynamic Algorithms book by Roy Featherstone, thus using 6D Vectors (spatial), using the convention linear before angular. A spatial force, for example will be a 6D vector, where the first 3 components are the force, while the la 3 components are the torque.

### Momentum Observer:
The momentum observer library is used to calculate the external and internal residuals, following the work presented in the paper cited in the Description section. The data needed to obtain the residuals are the following:\\
+ joints position, velocity and torques in the form of a `std::map<std::string, double>`. The map should contain the name of the joint and its value; this is to ensure consistency between the internal model and the data pased\\
+ the spatial velocity of the floating base\\
+ a quaternion expressing the orientation of the floating base\\

Beside these, which are data available on most robots, the interval of time (dt) needs to be specified, and the momentum observers' gains need to be specified.\\
The external residual will have a dimension of 6, and will be tied to the floating base dynamics, while the internal residual will have the same dimension of the DOFs of the robot. The residual, approximate respectively the external forces in the floating base frame, and the joint torques due to external forces.\\
The library also offers a time scaling option, useful when the dt is not fixed, and friction torque subtraction with a simple Coloumb-Viscous model.

### Force Estimator:
The force estimator, as the name implies, exploits the residuals to estimate the ground reaction forces. This estimation is made under the assumption that the time and point of the collision is known. The library, requires the feet frames (which need to be defined in its URDF) to be specified, as well as the number of contacts expected at most. The first ones can also be deducted from the joint states' names, but this only works if there is a consistency between joint prefix and feet frames. For example, ANYmal C has the front right leg's joints named with the prefix "FR" and its foot frame is "FR_FOOT". A model with this structure will work correctly. A `std::map<std::string, bool>` is needed to be passed to specify which foot, indicated by its frame name, is on the ground or not.\\
The actual force calculation is divided into two functions:
+ updateJacobians - requires the joints position as a `std::map<std::string, double>`, and the F and IC matrices (obtainable from Momentum Observer) and updates the jacobian states
+ calculateForces - requires both residuals and the quaternion expressing the orientation of the floating base; it returns a `std::map` which maps the feet frame names to their estimated forces

It is worth noting that the calculated forces are always the maximum number of contacts. If a leg is not on the ground, its spatial force will simply be zero.
