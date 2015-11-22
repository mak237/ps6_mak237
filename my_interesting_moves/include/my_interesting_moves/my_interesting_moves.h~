// "Copyright 2015 <Marc Krumbein>"  [legal/copyright]
// my_interesting_moves.h header file //
/// mak237; Nov, 2015.
/// Include this file in "my_interesting_moves.cpp", and in any main that uses this library.
/// This class provides four functions to return predefined goal vectors to send to Baxter
///

#ifndef MY_INTERESTING_MOVES_MY_INTERESTING_MOVES_H
#define MY_INTERESTING_MOVES_MY_INTERESTING_MOVES_H

#include <ros/ros.h>
#include <ros/init.h>

#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cwru_srv/simple_bool_service_message.h>

typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

// define a class, including a constructor, member variables and member functions
class SetGoals
{
public:
    SetGoals();

     /** Call this function to output the first predefined goal position for the right arm of the Baxter robot.
     * The values returned were created experimentally and are used to demonstrate robot manipulation.
     * @param q_ps6_pose_1 output: An experimentally found set of points to be used to manipulate Baxter
     */
    void set_goal_one(Vectorq7x1 &q_ps6_pose_1);

     /** Call this function to output the second predefined goal position for the right arm of the Baxter robot.
     * The values returned were created experimentally and are used to demonstrate robot manipulation.
     * @param q_ps6_pose_2 output: An experimentally found set of points to be used to manipulate Baxter
     */
    void set_goal_two(Vectorq7x1 &q_ps6_pose_2);

     /** Call this function to output the third predefined goal position for the right arm of the Baxter robot.
     * The values returned were created experimentally and are used to demonstrate robot manipulation.
     * @param q_ps6_pose_3 output: An experimentally found set of points to be used to manipulate Baxter
     */
    void set_goal_three(Vectorq7x1 &q_ps6_pose_3);

     /** Call this function to output the fourth predefined goal position for the right arm of the Baxter robot.
     * The values returned were created experimentally and are used to demonstrate robot manipulation.
     * @param q_ps6_pose_4 output: An experimentally found set of points to be used to manipulate Baxter
     */
    void set_goal_four(Vectorq7x1 &q_ps6_pose_4);


private:
};

// this closes the header-include trick...ALWAYS need one of these to match #ifndef
#endif  // MY_INTERESTING_MOVES_MY_INTERESTING_MOVES_H
