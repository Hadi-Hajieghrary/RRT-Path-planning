cd build

cmake ..

make


./pathplanning --image-file "../images/path_planning_challenge_image_1.png" --start-position "(50,50)" --goal-position "(900,900)"


This program implemented Rapidly-exploring Random Tree. In finding the new nodes the kinematics/dynamics of the robot is not considered. The result could be smoothed based according to the Kinematic and Dynamic constraines of the robot post-process, or the kinematics and dynamics of the robot can be considered when to chose the next step of the robot while sampling the space. A predominant method is using spline estimation interpolated from the waypoints. 

Here is one nice read on this subject:

Ravankar, Abhijeet & Ravankar, Ankit & Kobayashi, Yukinori & Hoshino, Yohei & Peng, Chao-Chung. (2018). Path Smoothing Techniques in Robot Navigation: State-of-the-Art, Current and Future Challenges. Sensors. 18. 3170. 10.3390/s18093170. 


