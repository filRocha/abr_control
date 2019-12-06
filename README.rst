***********
ABR Control
***********

ABR_Control: The ABR_Control library is a python library for the control and
path planning of robotic arms in various simulation enviornments. ABR_Control
currently supports and provides API's for Mujoco, VREP, and Pygame. Simple
one-joint, two-joint, and three-joint models are provided, in addition to a
more detailed model of the Kinova Jaco 2. There is also an API and config
available for controlling a real Jaco 2 with the same controllers and path
planners, which can be found in our `ABR_Jaco2 <https://github.com/abr/abr_jaco2/>`_ Repository

Installation
============

The ABR_Control library depends on NumPy, SymPy, SciPy, CloudPickle,
Cython, SetupTools, Nengo, and Matplotlib. We recommend using
`Anaconda <https://store.continuum.io/cshop/anaconda/>`_.
Note that installing in a clean environment will require compiling of the
dependent libraries, and will take a few minutes.

To install ABR_Control, clone this repository and run::

    sudo apt-get install g++
    sudo apt-get install python-dev
    sudo apt-get install libfreetype6-dev
    conda activate your_environment
    python setup.py install
    python setup.py develop

ABR_Control is tested to work on Python 3.6+, Python 2 is not supported.

Optional Installation
=====================

Mujoco
------
If you would like to use the Mujoco API you will need to install a
`forked version <https://github.com/studywolf/mujoco-py/>`_ where we have added
hooks for exitting out of simulations with the ESC key. This also opens the
API up for other keyboard interfacing through glfw. To use the mujoco API,
make sure you are in your anaconda environment and run::

    git clone https://github.com/studywolf/mujoco-py.git
    cd mujoco-py
    pip install -e .
    pip install glfw=>1.8.3
    pip install requests

Pygame
------
If you would like to use the Pygame API, from you anaconda environment run::

    pip install pygame

Vrep
----
We support up to Vrep 4.0. You will need to `download Vrep <http://coppeliarobotics.com/previousVersions/>`_
and follow the installation instructions.

PyDMPs
------
Some of the path planners work through the use of dynamic movement primitives (dmps).
DMPs allow for a stable way of representing complex motor actions without the need
to fine tune parameters. These path planners are appended with 'dmp' in their filename
and will require the installation of the pydmps repository. Again, from you Anaconda
environment run::

    pip install pydmps


Usage
=====

The ABR_Control repo is comprised of three parts: 1) arms, 2) controllers, and
3) interfaces.

1a) Arms: Using VREP, Pygame, or a real arm
---------------------------------------

All of the required information about an arm model is kept in that arm's
config file. To use the ABR_Control library with a new arm, the user must
provide the transformation matrices (written using SymPy expressions) from
the robot's origin reference frame to each link's centre-of-mass (COM) and
joints. These are specified sequentially, e.g.  origin -> link0 COM,
link0 COM -> joint0, joint0 -> link1 COM, etc. Additionally, the arm models
or simulation code is kept in the arm's folder.

The ABR_Control configuration base class uses the SymPy transform matrices
to provide functions that will calculate the transforms, Jacobian, Jacobian
derivative, inertia matrices, gravity forces, and centripetal and Coriolis
effects for each joint and COM. These can be accessed::

    from abr_control.arms import jaco2

    robot_config = jaco2.Config()
    # calculate the following given the arm state at joint_angles
    robot_config.Tx('joint3', joint_angles)  # the (x, y, z) position of joint3
    robot_config.M(joint_angles)  # calculate the inertia matrix in joint space
    robot_config.J('EE', joint_angles)  # the Jacobian of the end-effector

By default, the ``use_cython`` parameter is set to ``True`` to allow for
real-time control by generating optimized Cython code for each of the robot
configuration functions. This can take a little bit of time to generate these
functions, but they are saved in `~.cache/abr_control/arm_name/saved_functions`
where they will be loaded from for future runs. Note that a hash is saved
for the config, so if any changes are made the functions will be regenerated
during the next use. The cython optimization can be turned off on instantiation::

    from abr_control.arms import ur5

    robot_config = ur5.Config(use_cython=False)

Below are results from running the operational space controller with different
controllers using the Python and Cython config functions.

.. image:: examples/timing.png

1b) Arms: Using Mujoco
----------------------

When using Mujoco the process is a bit different. Mujoco handles the
generation of transformation matrices and only requires an xml config be made
describing the kinematic chain. The `Mujoco API <http://www.mujoco.org/book/modeling.html/>`_
is quite heavily documented, including the modelling process. To summarize,
the mujoco arm config is written in xml where the kinematic chain is defined
with ``<body>``, ``<geom>`` and ``<joint>`` tags. A ``<geom>`` defines a shape, whereas
a ``<body>`` can be a collection of ``<geom>``'s. ``<body>``'s contain
``<joint>``'s, which can be actuated by linking the ``<joint>`` name to an ``<actuator>``.
The dynamic properties of a ``<body>`` are defined in the ``<inertial>`` tag.

More detailed models can be created by importing stl files and using the ``mesh``
object type in the ``<geom>`` tag. For an example see ``abr_control/arms/jaco2/jaco2.xml``.
If using stl files, either specify their location with the ``folder``
parameter in the Mujococonfig, or place them in the arm's folder in a
``meshes`` folder. For more details, please refer to the Mujoco documentation linked
above and use the xml files in this repository as examples.

2) Controllers
--------------

The controllers make use of the robot configuration files to generate
control signals that drive the robot to a target. The ABR_Control library
provides implementations of several primary controllers, including operational
space, joint, sliding, and floating control.

There are also several implementations of secondary controllers designed to
operate in the null space of the operational space controller. These controllers
achieve secondary goals such as avoiding joint limits and obstacles, damping
movement, or maintaining a configuration near a specified resting state.

In the ``path_planners`` folder there are several path planners that can be used in
conjunction with the controllers. There are two straight-line filters on input targets,
linear and second order, which can be used to prevent the target from suddenly warping and
causing large spikes in generated torque. There is also an inverse kinematics
planner, which takes in a target for the end-effector and returns a joint angle
trajectory. An arc path planner is also provided that creates an arcing path
which can be useful when the arm has to reach over itself. This can help prevent
self-collisions and odd arm configurations.

Each path planner also has the ability
to generate a planned orientation path with the
``path_plannner.generate_orientation_path()`` function. This uses spherical linear
interpolation (SLERP) to generate a set of orientations from a start to a target
quaternion. The time profile will match that of the path planner instantiated
(ie: a linear path planner will have a linear step in orientation over time, with
a constant change in orientation, whereas a second order path planner will have a
bell shaped profile with the largest steps occuring during the middle of the movement,
with an acceleration and deceleration at the start and end, respectively)

In addition to filters, there is a second order implementation using dynamic
movement primitives. This allows the target to be adjusted on the fly without
needing to fine tune parameters. (See Optional Installation above)

Finally, there is an implementation of nonlinear adaptive control in the
``signals`` folder, as well as examples in Mujoco, PyGame and VREP showing how this
class can be used to overcome unexpected forces acting on the arm.

3) Interfaces
-------------

For communications to and from the system under control, an interface class
is used. The functions available in each class vary depending on the specific
system, but must provide ``connect``, ``disconnect``, ``send_forces`` and
``get_feedback`` methods.

Putting everything together
---------------------------
A control loop using these three files looks like::

    from abr_control.arms import jaco2
    from abr_control.controllers import OSC
    from abr_control.interfaces import VREP

    robot_config = jaco2.Config()
    ctrlr = OSC(robot_config, kp=20,
                # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
                ctrlr_dof=[True, True, True, False, False, False])
    interface = VREP(robot_config)

    interface.connect()

    target_xyz = [.2, .2, .5]  # in metres
    target_orientation = [0, 0, 0]  # Euler angles, relevant when controlled
    for ii in range(1000)
        feedback = interface.get_feedback()  # returns a dictionary with q, dq
        u = ctrlr.generate(
            q=feedback['q'],
            dq=feedback['dq'],
            target=np.hstack([target_xyz, target_orientation]))
        interface.send_forces(u)  # send forces and step VREP sim forward

    interface.disconnect()

**NOTE** that when using the Mujoco interface you will need to instantiate and
connect the interface before instantiating the controller. Some parameters
only get parsed from the xml once the arm config is linked to the mujoco
interface, which happens upon connection. If you try to instantiate your
controller before connecting to the interface you will receive an error
informing you to connect to your interface first. The correct order would
look like ::

    from abr_control.arms import MujocoConfig
    from abr_control.interfaces import Mujoco
    from abr_control.controllers import OSC

    robot_config = MujocoConfig('jaco2')
    interface = Mujoco(robot_config, dt=0.001)
    # connect to our interface first to gain access to config parameters
    interface.connect()
    ctrlr = OSC(robot_config, kp=20,
                # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
                ctrlr_dof=[True, True, True, False, False, False])

    # MAIN CONTROL LOOP STARTS HERE


Examples
========

The ABR_Control repo comes with several examples that demonstrate the use of
the different interfaces and controllers.

By default all of the PyGame examples run with the three-link MapleSim arm.
You can also run the examples using the two-link Python arm by changing the
import statement at the top of the example scripts.

To run the VREP examples, have the most recent VREP version open.
By default, the VREP examples all run with the UR5 or Jaco2 arm model.
To change this, change which arm folder is imported at the top of the example
script. The first time you run an example you will be promted to download
the arm model. Simply select ``yes`` to download the file and the simulation
will start once the download completes.

To run the Mujoco examples, you will be promted to download any mesh or texture
files, if they are used in the xml config, similarly to the VREP arm model.
Once the download completes the simulation will start. If you are using the
forked Mujoco-Py repository (See Optional Installation above) you can exit
the simulation with the ESC key.
