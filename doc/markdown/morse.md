# Using OMPL with Morse {#morse}

[TOC]

## Installation {#morseInstallation}

First, you'll need to [install MORSE (version 1.0 or later)](http://www.openrobots.org/morse/doc/stable/user/installation.html). After doing so, you should have a working copy of both Blender and MORSE. You can check if this was successful by running

    morse check

Second, follow the general instructions for installing OMPL with the Python bindings. Since Blender and MORSE require Python 3, it is essential you also use Python 3 for the OMPL python bindings:

    cmake -DPYTHON_EXEC=/usr/bin/python3 .

You may also need to install the Python3 libraries and headers if they aren't already (for example, on Ubuntu this is the python3-dev package). You should also take care to set the variables Boost_PYTHON_LIBRARY_DEBUG and Boost_PYTHON_LIBRARY_RELEASE to the exact Python 3 version of these libraries that is used by Morse. This version is included in the output of `morse check`.

At the moment, there is one Python script that should be installed as a Blender add-on. This is not done automatically. You'll find the add-on located at ${PREFIX}/share/ompl/addons/ompl_addon.py (by default, ${PREFIX} is equal to /usr/local). To install it, open Blender, navigate to File > User Preferences > Addons > Install addon... and select the ompl_addon.py file. Then enable this addon by clicking its check box. Finally, you'll want to Save As Default so that it will always be enabled when Blender starts.

## Quick Start {#morseQuickstart}

Modeling and solving a planning problem is a multi-step process. The next several sections will take you through this process from start to finish; however, you may wish to jump in a little further down so you can see some results right away. If you would like to use a pre-designed demo environment instead of modeling your own the first time, start at the section \ref morseAddingRobots, with ompl/demos/morse/rampjump_starter.blend as your environment file. If you would like to get to the planning phase immediately, you can start at the section \ref morsePlanning, with ompl/demos/morse/rampjump_complete.blend as your environment file.

## Setting up an environment {#morseSettingUpAnEnvironment}

The environment is the 3D model in which a planning problem is defined. We'll use Blender to construct the environment. To get started, open Blender and save the fresh file under a name of your choice. This will be called the environment file, and has the file extension *.blend. You can add and manipulate objects as you like to build, e.g., a course for a robot to move around in.

First, here's a crash course in using Blender:

- Blender has a search menu which is activated by pressing `Space`; you can then type the (partial) name of an action and select it from the list; move the mouse away from the menu to cancel.
- Changing the viewing angle of the 3D environment is done by clicking and dragging the middle mouse button; also try experimenting with holding down `Ctrl`, `Shift`, or `Alt` while doing this to perform different kinds of changes to the view.
- Selecting/deselecting an object is done by right clicking on it; use `Shift` + right click to select multiple objects.
- Hotkeys are very important in learning to use Blender efficiently; among the commonly used are `g` for grab/move, `r` for rotate, `s` for scale, and `x`, `y`, or `z` during one of those operations to lock motion to a specific axis.
- Objects are given automatic names, and duplicated names are given the suffixes *.001, *.002, etc.; they can be renamed in the Properties menu on the far right, under the 'Object' tab.

As a very simple example, we can use the cube already in the scene as an item that a robot can interact with (by pushing it, e.g.). We'll also need a surface for a robot to drive around on. The steps to accomplish this are outlined below:

1. Add a plane
   - invoke Blender's search menu by pressing `Space`
   - type 'Add plane', select this action from the menu; there should now be a new plane object in the scene
2. Position the plane
   - press `s` (hotkey for ‘scale’) and drag the mouse to enlarge the plane to several times its original size.
   - press `g` (hotkey for ‘grab/move’), then press `z` to lock the motion to the vertical axis, and drag the mouse to position the plane just beneath the cube
3. Enable appropriate physics for participating objects
   - look in the top menu bar for the words ‘Blender Render’ and change this option to ‘Blender Game’; this allows use to define settings regarding objects' behavior in the physics engine.
   - select the cube (right click) and look in the 'Properties' pane on the far right for an icon showing a bouncing ball; clicking it should open the physics settings for the cube (you may have to scroll right in the menu of buttons to find it).
   - set the 'Physics Type' to 'Rigid Body'; this allows the cube to participate in the physics simulation as a movable, collidable object; also check the 'Collision Bounds' setting, and make sure it's using the 'Box' bounds (for more complicated objects, it may be appropriate to use 'Convex Hull' or 'Triangle Mesh').
   - now select the plane; its physics type should already be 'Static' by default; leave it as such since the plane should be immovable, but still collidable.

## Adding robot(s) {#morseAddingRobots}

MORSE requires special knowledge of the robots in a scene in order to control them, so rather than build them in as part of the environment, we provide a tool for adding robots into a scene. It is located in the OMPL menu; navigate to Game -> OMPL -> Add Robot... (Note that you will not be able to find this menu unless you have changed from the 'Blender Render' engine to the 'Blender Game' engine in the top menu bar.)

This brings up a dialog where you can select a robot type and a controller for it. Note that controllers may be designed to work only with specific robots, and vice versa. A simple choice is the SegwayRMP400 controlled by MotionVWDiff. This provides a car-like robot with 4 wheels and a controller with two inputs: desired velocity and desired turning speed. MORSE takes care of converted these inputs into appropriate forces to be applied to the wheels during simulation.

\note **Tip:** Some MORSE installations incorrectly identify their own resource folder. E.g., MORSE may look for robot models at '/usr/local/share/morse/' instead of '/usr/share/morse'. If that is the case, the robot model will not be found in this step, and MORSE will print an error to the terminal. Creating a symlink can rectify this.

After adding a robot to the scene, you can position it using the grab and rotate tools (`g`, and `r`). If you would like more than one robot of the same type, you can use the duplicate tool (`Shift` + `d`) instead of returning to the Add Robot dialog.

## Defining the goal {#morseGoal}

Goals are defined by specifying that a particular object should arrive at a pre-defined pose to within a certain tolerance or that the object should enter a pre-defined region. A goal requirement is conveyed to OMPL via the existence of an object that shares the same name as the object for which we are specifying the goal, but ends in the suffix '.goalPose', '.goalRegion', or '.goalRot'. To add a new pose or rotation goal specification for an object, navigate to Game -> OMPL -> Add Goal... Here you can choose which goal type you would like to add. Adding a region goal specification simply requires appropriately renaming an existing object representing the region.

**Goal Poses:** To specify that an object needs to reach a certain location and rotation, add a new Goal Pose for it. This "goal object" will look just like the actual object it corresponds to, but its appearance is irrelevant and you may change it. Position it into the desired pose for the object using the grab and rotate tools. In the Properties menu on the far right, under the Object tab, in the Custom Properties section, you will find locTol and rotTol settings. These control how closely the object must match its goal pose to count as satisfying the goal. The first is the location tolerance (Euclidean distance), and the second is rotation tolerance, where 0.0 means it should be identical, and 1.0 means the rotation is irrelevant.

**Goal Regions:** To specify that an object needs to enter a certain region, create a closed object (i.e., one that has a clear "inside" and "outside"), whose shape and location define the region that must be entered. You must name this Goal Region object correctly for it to work: it should have the same name as the object for which you are specify a goal, but with the suffix '.goalRegion'.

**Goal Rotations:** To accompany a goal region, you may wish also to specify a desired rotation (independent of location since that is accounted for by the goal region). Setting up this kind of goal is the same as for a goal pose, exception there is no location tolerance setting, and the location of the goal object is irrelevant.

If you started from scratch with the section \ref morseSettingUpAnEnvironment, try adding a goal pose for the cube object requiring it to be pushed back by the robot. Don't set the tolerance too small, or the goal will not be reached in a reasonable time.

\note **Tip:** to keep your modeling environment clean and easy to work with, consider moving all these goal specification objects to another layer. Press `M` then `2`, for example, to move selected items to layer 2. You can then switch between layers by pressing `1` or `2`, and display multiple layers together with `Shift` + `1` or `Shift` + `2`.

Note that the goal specification objects will not participate in physics, so there is no need to worry about disabling physics for them.

## Bounds configuration and planning {#morsePlanning}

\note **Tip:** from this point on, it is very useful to have launched Blender from a terminal so that you can see the output it produces, especially if anything goes wrong.

When you have finished setting up the scene as you like, its time to configure the planning bounds. This includes limits on the locations and velocities of all rigid body objects, as well as limits on the inputs to the robots' controllers. Bring up the menu for these settings by navigating to Game -> OMPL -> Bounds configuration... It will take a moment to load and you will see MORSE start up temporarily as the program gathers information about the controllers.

On the left side of the menu, you will be able to set upper and lower bounds for location in the x, y, and z dimensions, and for linear and angular velocities. The 'Automatic position bounds' will ignore user-specified bounds on the location and will use the current locations of all objects to estimate how large the bounds should be. This is useful if you are in a hurry.

On the right side of the menu you will find settings for upper and lower bounds on each of the control inputs for each of the robots. They are automatically labeled as best as possible; however, you may need to consult the documentation for specific controllers to determine what each argument does. (E.g., in the MotionVWDiff controller, the first argument specifies desired velocity, and the second specifies desired turning speed.)

If you have been following along from the section \ref morseSettingUpAnEnvironment, a good choice is to use the automatic position bounds and leave the min/max velocities at their defaults. Also, for the robot set_speed control, use plus/minus 5.0 as the bounds for arg0 and plus/minus 2.0 as the bounds for arg1.

After you have configured the planning bounds, you are ready to plan. Navigate to Game -> OMPL -> Plan... Choose a (possibly new) file in which you would like OMPL to save the solution, like 'path.out'. On the terminal you should see MORSE starting up, followed by a list of rigid body and goal objects, followed by OMPL starting up. If everything was successful, the final message before planning should be "Starting with 1 states". If you have `wmctrl` installed, the planning window will be “shaded” to keep it out of the way, but you can unshade it to watch if you would like. To stop a plan before a solution is found, close the planning window (or with it unshaded, press `Esc`); the best approximate solution found so far will be saved. Otherwise, OMPL will plan as long as it takes to find a solution.

## Playback and animation saving {#morsePlayback}

After planning is completed, you'll want to review the solution path by playing it back. At the same time, the path will be saved into a format used for animation in Blender allowing convenient production of pretty videos. To choose the destination file for this, run Game -> OMPL -> Choose animation save file... This file will be a copy of your environment *.blend file, but with a record of the changes to the poses of all the rigid body objects throughout the solution path.

To play back the solution and record the animation to file, run Game -> OMPL -> Play..., selecting the same file you chose in the Plan phase, e.g. 'path.out'. This will start up MORSE much like in the Plan phase, but will only re-enact the solution path.

## Rendering animation videos {#morseRendering}

After playing back the solution path, you can open the new animation *.blend file that was recorded during playback. Now you can edit the precise movements made by all the objects in the scene to clean up or tweak the path if you wish. Also you can use Blender to its full extent in applying materials and textures to the objects. You can even alter the shape of objects without affecting any of the motion since it is coded into the *.blend and no longer depends on the results of a physics simulation.

\note **Tip:** Blender has a built-in screen layout designed to help with the animation process. To access it, first open Blender, then open the animation file, being sure to uncheck the 'Load UI' option on the left. Then you can choose the 'Animation' screen layout instead of 'Default' in the top menu bar.

\note **Tip:** if you want to completely swap out an existing object with a fancier, prettier one, first add the new mesh to the scene, then select the old object, and in the Properties menu on the far right, go to the Object Data tab. Next to the name of the mesh is a 'Browse Mesh Data to be linked button which can be used to select the alternate mesh as a replacement.

You'll need to switch back from the 'Blender Game' engine to the 'Blender Render' engine in the top menu bar to allow rendering your animation without the physics engine. There are many useful settings in the Properties menu on the far right under the Render tab. Because the animation was created with a 60 Hz clock, make sure the frame rate setting is either 60 fps, or 30 fps with a frame step of 2 to skip alternate frames; otherwise the speed of the video will be incorrect. Also ensure that end frame is the last frame in which there is saved animation data. You can determine this by looking at the Timeline at the bottom of the Blender window; yellow lines indicate frames in which there is animation data. You should then change the output format to AVI JPEG or some other video format. If all the other settings are to your liking, you can press Render Animation to create the video.

## Advanced tweaks {#morseAdvancedTweaks}

There is some interesting functionality that is not exposed via the OMPL addon interface, since it requires a little bit of coding. Here are some pointers to get you started if you want to try out something a little fancier.

1. You can change the planner used by OMPL. See scripts/morse/planner.py, lines 66ff. Right now, it uses RRT. An alternative is giving in a block comment that uses KPIECE1; this is one of the planners that requires a projection. See the projection code at scripts/morse/environment.py, lines 264ff.
2. You can change the minimum and maximum control durations, as well as the control step size. See scripts/morse/environment.py, lines 70ff. Note that the playback process also uses the control step size setting.
3. You can modify the goal code to explain a complicated requirement to the system. See scripts/morse/environment.py, lines 318ff., especially the isSatisfied_Py() function. For exmample, you could require the distance between to specific objects to be greater than some value. Tip: when the simulation prints "Gathering list of rigid bodies and goal criteria," note that the rigid bodies are numbered; this number tells you which components of the state corresponds to each object. In particular, the state space is organized as a list containing the 4 vectors (position, linear velocity, angular velocity, and orientation) of the first rigid body, then the second, etc., so that, e.g., state[4*7 + 2] gives the angular velocity vector for rigid body number 7.
4. MORSE allows you to make your own robot models and write actuators (controllers) for them. You can even write new actuators for existing robots. This could be helpful in increasing the number of robots in the MORSE library that are usable by this planning process. See comments at scripts/morse/ompl_addon.py.in, lines 178ff., 205ff.
5. The planning process is quite slow right now, caused by an inability to speed up the Blender Game Engine. It was designed for live simulation, and though the underlying computations are extremely quick, it maintains a uniform pace instead of progressing the simulation as fast as possible. The MORSE developers are hoping to introduce faster-than-realtime simulation soon. In the meantime, there's a very minimal change you can make to the Blender source that multiplies the speed by a modest factor. Five times as fast seems to be the limit, presumably because Blender refuses to perform more than 5 physics updates during the same tick to keep up with the time. The speed hack simply entails multiplying the elapsed time returned by Blender's internal clock by 5 (anything higher will not have much affect). If you've built Blender from source, just open up blender/source/gameengine/BlenderRoutines/KX_BlenderSystem.cpp, multiply the value returned by GetTimeInSeconds() by 5, and recompile. Just remember to revert back to the normal speed before doing a playback, or else divide the playback durations by 5 (dur[i] in scripts/morse/player.py, line 66).
