For the vacuum movement we wanted to develop in phases. 

Dumb_Vacuum (Attempt 1)
The first phase is just to try to get something moving, we did that successfully with dumb_vacuum.py. All it does is ensure that the minimum distance from the lidar is above some threshold, if so moves forward at some velocity until that's not the case, then spins until it's facing a new direction without occlusions. 

This was fine, but it had a tendency to hug walls, and then just traverse the edge over and over and over again 

Dumb_vacuum (Attempt 2) 
The goal is to create something that's probablistically complete, this second attempt (currently in progress) is trying to make that the case. We will find a maximum sliding window (pick arbitrarily if multiple) and then traverse in that direction after aligning to that heading with a pid controller.