# Gaze control  
The goal of this task is to implement gaze controller for the iCub robot that is able to follow a ball moving on a table.  
The task is simplified to 2D case.
The task will be run with GUI off.

## Task
Implement gaze() function in `hw2.py` that will control the gaze of the robot to follow the ball.  
   - the function takes one argument:
     - client - instance of pycub class that controls the simulation
   - the function should control the robot to follow the ball
     - the move **must** non-blocking, i.e., parameter wait=False  
   - you **should not** call `update_simulation()` in this function

## Scoring
 - the ball will be moving for N seconds and each step the error in degrees will be calculated  
 - The score is calculated based on head-view direction and head-to-ball direction
 - maximum number of points is M and you will **lose** points based on the following:
   - if the mean absolute error is:
     - less than 0.55 degree - 0% of points
     - more than 0.55 and less than 1 degree - 50% of points
     - more than 1 and less than 5 degrees - 75% of points
     - more than 5 - 100% of points
   - if the max error is:
     - less than 1.25 degrees - 0% of points
     - more than 1.25 and less than 5 degrees - 25% of points
     - more than 5 and less than 10 degrees - 50% of points
     - more than 10 - 100% of points
 - the test will be done **11** times and the final score will be the average of **9** best results
 - If you have all 11 correct, you will get 1 bonus point

## Requirements
   - do not create new client instance, use the one that is passed as an argument
     - 0 points be will be given otherwise 
   - do not rename the function or file
     - 0 points be will be given otherwise 
   - do not call `update_simulation()` in the function
     - 0 points be will be given otherwise 
   - use non-blocking move **must**, i.e., parameter wait=False  
     - 0 points be will be given otherwise 
   - have "nice" coding style
     - use understandable variable names
     - use comments to explain more complicated things
       - -1 points otherwise
   - Have a general code, i.e., no if/else (switch) statements every possible case that can happen
     - you can of course still use if/else when needed, but should not be used for example based on every possible position of the ball etc.
     - -1 points otherwise