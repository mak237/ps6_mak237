// "Copyright 2015 <Marc Krumbein>"  [legal/copyright]
// plane_fitter: program to illustrate creation and use of a ROS library
//

#include<ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <my_interesting_moves/my_interesting_moves.h>

// using namespace std;


SetGoals::SetGoals()
{
}

void SetGoals::set_goal_one(Vectorq7x1 &q_ps6_pose_1)
{
    q_ps6_pose_1<< -0.7, -0.08,   1.86,    1.43,    -1.5,   2.23,  -2.4;
}

void SetGoals::set_goal_two(Vectorq7x1 &q_ps6_pose_2)
{
    q_ps6_pose_2<<  0.5,  0.5,      1.5,     1,       -1.7,     1,     -1.8;
}

void SetGoals::set_goal_three(Vectorq7x1 &q_ps6_pose_3)
{
    q_ps6_pose_3<< -0.1, -0.28,   3.06,    0.63,    -2.3,   0.43,  -1.6;
}

void SetGoals::set_goal_four(Vectorq7x1 &q_ps6_pose_4)
{
    q_ps6_pose_4<< -0.7, -0.08,   1.86,    1.43,    -1.5,   2.23,  -2.4;
}
