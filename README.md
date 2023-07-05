# ball-platform
A robot project made to balance a ball on a platform. The platform is controlled by two servo motors, and the ball's position is detected using a camera. A Raspberry Pi is the brain of the project, and calculates the incline of the plane needed to get the ball to the desired position. 

## Forward kinematics
This is the exact dimensions of the robot:
![image](https://github.com/Martin-Ansteensen/ball-platform/assets/50178947/55fe8bb5-64f0-4437-876a-041d446361d8)
To calculate the forward kinematics, I simplified it to the following:
![calculations](https://github.com/Martin-Ansteensen/ball-platform/assets/50178947/a285fe4d-40dd-4438-9d43-28b1787acfb3)
C is the center of the disk, and J1 is the center of rotation of the servo. J2 is where the servo-arm (L1) connects to the arm with ball-joints (L2). J3 is the location where the ball joint attaches to the disk. There are mainly two simplifications made in the model: in reality the disk pivots around a point that is offset from the disk and J3 is conncted to the disk at an offset.

Given a fixed position of J2, we want to find the angle of the plane (v). We know that J3 must be a fixed distance from C (lie on the circle perimeter in blue). We also know that J2 and J3 are at a fixed distance from each other (L2).
![calculation](https://github.com/Martin-Ansteensen/ball-platform/assets/50178947/4d6f0c7e-48a7-43a8-8252-d65dc4c1ec01)
Where J2 is given by 
```math
 (x, y) = (L1\cdot cos(u) + x(J1), L\cdot sin(u) + y(J1))
```
where u is the angle of the servo located at J1, and x(J1) and y(J1) is the x- and y-position of J1 relative to C.

## Inverse kinematics
As the function used to calculate the forward kinematics is monotonic (at least with the dimensions of my robot), I did not bother calculting the inverse kinematics. Instead, I perform a binary search on the forward kinematics function (shown below, where the inputs will range from -1.05 to 0.82)
![function](https://github.com/Martin-Ansteensen/ball-platform/assets/50178947/0bd60cb9-3e6c-4774-ab28-4dd2aff8223a)

## Robot in action
[![Watch the video](https://img.youtube.com/vi/sYU_3XsPKic/0.jpg)](https://www.youtube.com/watch?v=sYU_3XsPKic "TITLE")

[![Watch the video](https://img.youtube.com/vi/nO7r-7b6NJc/0.jpg)](https://www.youtube.com/watch?v=nO7r-7b6NJc "TITLE")

