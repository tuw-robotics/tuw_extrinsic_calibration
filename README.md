#tuw\_extricsic\_calibration

This software features a tool for semi-automatic extrinsic camera calibration implemented as an rviz plugin, designed for robots with a mounted camera and laser scan. It is therefore expected that the robot is equipped with a laser scanner which must be already calibrated.

##Details

The calibration plugin allows to connect a known tf, for instance /base_link, to the unknown camera tf that is estimated.
Internally, the pnp algorithm is used to determine the position of the camera with respect to the base frame. 

In order to solve pnp, reference points in the image must be given, which is done manually via the gui. Additionally,
the corresponding position in base link coordinates of the image points is inferred from the laser scan, mounted on the robot. 
Then the algorithm computes the TF. These steps can be repeated if a more precise estimation with more measurements are needed.

##Usage tutorial

First, start rqt_gui.

``rosrun rqt_gui rqt_gui``

Second, select the plugin via Visualization->Extrinsic Camera Calibration.
Play your bag file or start a live ros session with the robot.

After the topics arrived at least once, they can be selected in the combo box. For the base frame any TF can be selected (that has a link to the laser TF). Afterwards, the camera image along with the laser scan are displayed as seen in the Figure below. 
![GUI View](https://github.com/tuw-robotics/tuw_extrinsic_calibration/tree/master/tuw_extrinsic_camera/images/gui_0.png)

The workflow for using this plugin is as follows.

0. (Optional) freeze image & laser.
1. Select the valid laser scan range.
2. Mark the 4 (!) corners of the object.
3. Click use range & publish to compute the pose and publish it.
4. If needed, click refine and repeat steps 1-3.

In order to select the valid laser scan range lying on the object, the upper sliders can be used to restrict the angular range. The lower sliders can be used to select the distance. If the distance slider is modified the algorithm will use this corrected distance for pose estimation. This is useful when dealing with unrelieable laser scan readings.
For a live setting it is recommended to tick the freeze image and freeze laser checkboxes, as the images are updated with every incoming message. This allows for a more finegrained selection. Note that rosbag play allows pausing by pressing space, therefore the freeze buttons may not been used here.  
