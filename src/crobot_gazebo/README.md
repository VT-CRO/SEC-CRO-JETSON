# crobot_gazebo

This package contains nodes for launching a robot simulation in gazebo. Also includes a mesh and URDF of the game field in `models`. All external models used should be placed in this folder.

## Launch Files
- `launch_sim.launch.py` - Launches gazebo with a simulated robot

## How To Use

### Using External Models
External models can be in either .dae, .stl, or .obj format. First create a folder for your model and create a new .sdf file for your model containing the following:

```
<?xml version="1.0"?>
<sdf version="1.5">
  <model name="[your model name]">

    <static>true</static>

    <link name="link">
    
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>[relative path to your collision model]</uri>
          </mesh>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <mesh>
            <uri>[relative path to your visual model]</uri>
          </mesh>
        </geometry>
      </visual>

    </link>
  </model>
</sdf>
```

The collision and visual models need not be the same.

*Note*: When importing .dae files from OnShape, you might find some meshes missing, particularly with instances of a mesh. To fix this, install Blender and import your .dae file.
At this point you can also easily make modifications to your model here with custom materials (colors). Now export the model as a .dae file with the parameter...

[TODO: Blender parameters]

### Saving Worlds
A Gazebo world allows you to "save" a scene in Gazebo (contains all models, lights, settings, etc.) that can be loaded later. To create a world, first load in all the models you need and move them to your desired positions.
If you have any models located in the `models` folder, you can find them in the... as shown in the image below:

[TODO: Show image]

Then you can save the world (shown in the image below) and load it later. It is recommended to savee worlds in the `worlds` folder of this package, but it is not necessary. The world can be lmanually oaded from gazebo as shown in the image below.

[TODO: Show image]


A world can also be loaded from the `launch_sim.launch.py` with the `world` parameter as shown in the example below:

```
ros2 launch crobot_gazebo launch_sim.launch.py world:=/absolute/path/to/your/world/file
```