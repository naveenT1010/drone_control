## Following are some important points for this project

1. The off board mode of mavros changed a bit in the service call for state change to offboard. 
2. geographic data has to be installed for mavros to work. The scripts folder has a script which can install all the required data sets.
3. undefined refernce errors are due to not adding packages in CMake file of ros node.
4. This link is life saver since tracking api wasn't being detected by ros: https://answers.ros.org/question/272840/install-opencv_contrib-tracking-package/
5. This is also useful: https://answers.ros.org/question/109926/undefined-reference-to-cvfeature2dcompute/
