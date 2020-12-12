Project Title: Smart Efficient Line Follower (SELF)
Authors: Rachel Lese, Tiff Lyman


![robotics map image](https://user-images.githubusercontent.com/43554898/101966009-9139d200-3be4-11eb-9920-9bbf6bd21282.jpg)


OVERVIEW:

This project uses reinforcement learning to direct a robot to follow a line.
It uses the same imaging as the original line follower, except instead of PID
it has a set of possible angular and linear velocities with a weight attributed
to each one. The velocity with the highest weight is what the robot uses.
These weights change over time as the robot takes in more data to influence the 
weights.

Instructions to Run:

Clone Repo: https://github.com/campusrover/SELF.git
 - Navigate to src/ in both the code editor and your desktop
 - Launch learnLine.launch
 - run Final_Project.py in the code editor

Once opened, you can watch the robot run on the map that we have created. Feel free to create another map and play with our algorithm!

We hope you enjoy reading through our code and playing with our algorithm!

Rachel Lese and Tiff Lyman
