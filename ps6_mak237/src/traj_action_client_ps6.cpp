// traj_action_client_ps6: 
// mak237
// This node moves Baxter's right arm to a set of predefined positions found in the 'my_interesting_moves' library

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <my_interesting_moves/my_interesting_moves.h>

#include<cwru_action/trajAction.h>
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

// This function will be called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
        const cwru_action::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d",result->return_val);
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "traj_action_client_node"); // name this node 
        ros::NodeHandle nh; //standard ros node handle        
        int g_count = 0;
                int ans;
    Vectorq7x1 q_ps6_pose_1, q_ps6_pose_2, q_ps6_pose_3, q_ps6_pose_4;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;
       
  
       std::vector<Eigen::VectorXd> des_path;
        

        trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
        cout<<"instantiating a traj streamer"<<endl; // enter 1:";
        //cin>>ans;
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here

// Use the functions defined in the 'my_interesting_moves' library to set goals and push them to be used on Baxter

SetGoals SetGoals;

SetGoals.set_goal_one(q_ps6_pose_1);
q_in_vecxd = q_ps6_pose_1; 
des_path.push_back(q_in_vecxd);

SetGoals.set_goal_two(q_ps6_pose_2);
q_in_vecxd = q_ps6_pose_2; 
des_path.push_back(q_in_vecxd);

SetGoals.set_goal_three(q_ps6_pose_3);
q_in_vecxd = q_ps6_pose_3; 
des_path.push_back(q_in_vecxd);

SetGoals.set_goal_four(q_ps6_pose_4);
q_in_vecxd = q_ps6_pose_4; 
des_path.push_back(q_in_vecxd);


    cout << "stuffing traj: " << endl;
    baxter_traj_streamer.stuff_trajectory(des_path, des_trajectory); //convert from vector of 7dof poses to trajectory message        
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        cwru_action::trajGoal goal; 
        //  copy traj to goal:
        goal.trajectory = des_trajectory;
        //cout<<"ready to connect to action server; enter 1: ";
        //cin>>ans;
        // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
        actionlib::SimpleActionClient<cwru_action::trajAction> action_client("trajActionServer", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


        if (!server_exists) {
            ROS_WARN("could not connect to server; will wait forever");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        server_exists = action_client.waitForServer(); //wait forever 
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        //while(true) {
        // stuff a goal message:
        //g_count++;
        //goal.traj_id = g_count; // this merely sequentially numbers the goals sent
        //ROS_INFO("sending traj_id %d",g_count);
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        
	// commented out due to changes to traj.action - traj_id is no longer available
	// traj_id is used to check for timeouts on individual actions - without it we can only check timing 
	// for total action duration, not individual action durattion
        if (!finished_before_timeout) {
	    
            //ROS_WARN("giving up waiting on result for goal number %d",g_count); 
            return 0;
        }
        else {
            ROS_INFO("finished before timeout");
        }
        
        //}

    return 0;
}

