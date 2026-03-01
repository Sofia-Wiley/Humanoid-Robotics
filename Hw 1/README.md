# Push the Ball
The goal is to hit the ball to push it as far as possible from **any part** of the table.
The ball will always spawn at the same place. The robot can be moved with position or cartesian control.
The trajectories should be collision free and max allowed velocity is 10 (rad/s).  

The resulting distance is checked 1 second after returning from the movement function. GUI will be off.

## Task
Implement `push_the_ball` function in `hw1.py` that will push the ball as far as possible from the table.  
  - the function takes one argument:
    - client - instance of pycub class that controls the simulation

## Scoring
 - max 5 points
 - points for distance are computed as: min(5, distance*2)
   - i.e., you get max 5 points if you push the ball 2.5m away
 - the best three of you get 3/2/1 bonus points
 - The code will be evaluated by TA at the end. There is no requirement for code quality. However, you can loose
   points from the automatic evaluation if violate any of the requirements below.
   - TA will check the runs visually and assess the possible violations. 

## Requirements:
  - do not create new client instance, use the one that is passed as an argument
    - -5 points
  - do not rename the function or file
    - -5 points 
  - Trajectories must be collision free with the table (self-collisions of the robot are allowed)
    - -1 point 
  - Max allowed velocity is 10 (rad/s)
    - -3 points 
  - **Do not** introduce artificial delays in the code, e.g., sleep() or using update_simulation() after the ball leave table
    - You can get as creative, e.g., throw or kick the ball. You have unlimited time  to do so, but 
      only until the ball leave the table. After that if you spend time doing unnecessary stuff you will lose points.
    - -5 points 
  - **Do not** turn of gravity
    - -5 points 