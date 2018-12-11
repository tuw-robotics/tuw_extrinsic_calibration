# tuw_extrinsic_calibration

This software features a tool for semi-automatic extrinsic camera calibration implemented as an rviz plugin, designed for robots with a mounted camera and laser scan. It is therefore expected that the robot is equipped with a laser scanner which must be already calibrated. In addition, it assumes that rectangular objects such as doors are visible within the image.

## Details

The calibration plugin allows to connect a known tf, for instance /base_link, to the unknown camera tf that is estimated.
Internally, the pnp algorithm is used to determine the position of the camera with respect to the base frame. 

In order to solve pnp, reference points in the image must be given, which is done manually via the gui. Additionally,
the corresponding position in base link coordinates of the image points is inferred from the laser scan, mounted on the robot. 
Then the algorithm computes the TF. These steps can be repeated if a more precise estimation with more measurements are needed.

## Usage tutorial

First, start rqt_gui.

``rosrun rqt_gui rqt_gui``

Second, select the plugin via Visualization->Extrinsic Camera Calibration.
Play your bag file or start a live ros session with the robot.

After the topics arrived at least once, they can be selected in the combo box. For the base frame any TF can be selected (that has a link to the laser TF). Afterwards, the camera image along with the laser scan are displayed as seen in the following image.
![GUI intro view](https://github.com/tuw-robotics/tuw_extrinsic_calibration/raw/master/tuw_extrinsic_camera/images/gui_0.png).

The workflow for using this plugin is as follows.

0. (Optional) freeze image & laser.
1. Select the valid laser scan range.
2. Mark the 4 (!) corners of the object.
3. Click use range & publish to compute the pose and publish it.
4. If needed, click refine and repeat steps 1-3.

In order to select the valid laser scan range lying on the object, the upper sliders can be used to restrict the angular range. The lower sliders can be used to select the distance. If the distance slider is modified the algorithm will use this corrected distance for pose estimation. This is useful when dealing with unrelieable laser scan readings.
For a live setting it is recommended to tick the freeze image and freeze laser checkboxes, as the images are updated with every incoming message. This allows for a more finegrained selection. 

After step 2 the GUI will look like the image below (note that you can also zoom into the image). At the current moment you must (!) specify the points along the object in a clockwise fashion starting with the bottom left corner. The assumption is that the object is of rectangular shape and its dimensions are currently hardcoded. This is still work in progress  
![second](https://github.com/tuw-robotics/tuw_extrinsic_calibration/raw/master/tuw_extrinsic_camera/images/gui_1.png).

After the estimation, the laser scan will be projected into the image, giving the user a feeling of the estimations accuracy.
![third](https://github.com/tuw-robotics/tuw_extrinsic_calibration/raw/master/tuw_extrinsic_camera/images/gui_2.png)
The calibration depicted here is not good enough (see projected laserscan on the bottom right), since a planar object is used. However, one should then refine the estimation by driving to a different spot and selecting the same object again (steps 1-3) or selecting a different object in view. Hereby, all points from the previous selection(s) and the current selection are used for PnP. If you made a mistake click the undo button, the last 4 points will be deleted and the previous TF will be restored. This can be done multiple times until all previous data is cleared.