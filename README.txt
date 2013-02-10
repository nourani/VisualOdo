# Summary
A demonstration of visual odometry for car-like vehicles using a downward looking camera. Please refer to the publications:


[1] Navid Nourani-Vatani and Paulo VK Borges, Correlation-based Visual Odometry for Car-like Vehicles, Journal of Field Robotics, September 2011

[2] Navid Nourani-Vatani, Jonathan Roberts and Mandyam V Srinivasan, Practical Visual Odometry for Car-like Vehicles, IEEE International conference on Robotics and Automation, May 2009

[3] Navid Nourani-Vatani, Jonathan Roberts and Mandyam V Srinivasan, IMU-aided Visual Odometry for Car-like Vehicles, Australasian conference on Robotics and Automation, Dec 2008

I would love to hear from you if you are using this software and finding it useful or finding bugs (They say ~10 for every 1000 lines of code, so there are probably about 15-17 of these in there!)

You can contact me at n.nourani-vatani@acfr.usyd.edu.au

Enjoy
Navid

-------------------------------------------------------------------------------
# License
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

There is a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.


-------------------------------------------------------------------------------
# Getting started
The code relies on OpenCV 2.0 and is compatible with both Mac OS X and Linux.

Run "cmake .; make; make install" to install in the standard install location (/usr/local/)

Execute by running ./VisualOdoGroundCam


NOTES ON TEST DATA:
There is a folder containing 1000 images which you can use to test the algorithm on. This folder has been compressed to save space but it is still pretty large (132 MB). Uncompress it and make sure the "string fileDir" is pointing to the folder.


NOTES ON SETTINGS:
There are several parameters you SHOULD set. These include the camera location and the camera parameters. Set these in the "main()" function. There are other paramers regarding the template matching, TQM etc that you can change also.


NOTES ON FORWARD PREDICTION:
In the above mentioned publication, we employ a forward predicting FIR filter.  In this code, however, I am using Kalman Filters. The main reason is to limit the dependencies on other libraries. The code for using an FIR filter is included but commented out. You can easily swap to using the FIR by installing the libmusic library.


NOTES ON TEMPLATE QUALITY MEASURES:
I have also commented out the auto-correlation template quality measure for the same reason. To enable it you need to install the CImg header files.


NOTES ON IMU:
I have removed the dependency on IMU in this code. It is however quite straight forward to incorporate the IMU readings to generate 3D odometry estimates. Look through the code to find the location to read the IMU and to add the roll and pitch data.

