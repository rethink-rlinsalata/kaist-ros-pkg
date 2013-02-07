/*
 * IKFast Plugin Template for moveit
 *
 * You should run create_ikfast_moveit_plugin.py to generate your own plugin.
 *
 *
 * Creates a kinematics plugin using the output of IKFast from OpenRAVE.
 * This plugin and the move_group node can be used as a general 
 * kinematics service, from within the moveit planning environment, or in 
 * your own ROS node.
 * 
 * Tested on ROS Groovy
 * with code generated by IKFast61 from OpenRAVE 0.8.2
 * using a 6 DOF manipulator
 * 
 * 
 * Author: David Butterworth, KAIST & Jeremy Zoss, SwRI
 *         Based on original plugin by unknown author
 * Date: January 2013
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Auto-generated by create_ikfast_moveit_plugin.py in arm_kinematics_tools

#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <urdf/model.h>
#include <tf_conversions/tf_kdl.h>

// Need a floating point tolerance when checking joint limits, in case the joint starts at limit
const double LIMIT_TOLERANCE = .0000001;

namespace _ROBOT_NAME___GROUP_NAME__kinematics
{
#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
// Code generated by IKFast56/61
#include "_ROBOT_NAME___GROUP_NAME__ikfast_solver.cpp"

  class IKFastKinematicsPlugin : public kinematics::KinematicsBase
  {
    std::vector<std::string> joint_names_;
    std::vector<double> joint_min_vector_;
    std::vector<double> joint_max_vector_;
    std::vector<bool> joint_has_limits_vector_;
    std::vector<std::string> link_names_;
    size_t num_joints_;
    std::vector<int> free_params_;

    const std::vector<std::string>& getJointNames() const { return joint_names_; }
    const std::vector<std::string>& getLinkNames() const { return link_names_; }

    public:

      /** @class
       *  @brief Interface for an IKFast kinematics plugin
       */
      IKFastKinematicsPlugin() {}

      /**
       * @brief Given a set of joint angles and a set of links, compute their pose
       * @param link_names  - set of links for which poses are to be computed
       * @param joint_angles - current joint angles
       *          the response contains stamped pose information for all the requested links
       * @return True if a valid solution was found, false otherwise
       */
      // This FK routine is only used if 'use_plugin_fk' is set in the 'arm_kinematics_constraint_aware' node,
      // otherwise ROS TF is used to calculate the forward kinematics
      bool getPositionFK(const std::vector<std::string> &link_names,
                         const std::vector<double> &joint_angles, 
                         std::vector<geometry_msgs::Pose> &poses) const;
    
      /**
       * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
      // Returns the first IK solution that is within joint limits,
      // this is called by get_ik() service
      bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                         const std::vector<double> &ik_seed_state,
                         std::vector<double> &solution,
                         moveit_msgs::MoveItErrorCodes &error_code) const;

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
      bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          moveit_msgs::MoveItErrorCodes &error_code) const;

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @param the distance that the redundancy can be from the current position 
       * @return True if a valid solution was found, false otherwise
       */
      bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                            const std::vector<double> &ik_seed_state,
                            double timeout,
                            const std::vector<double> &consistency_limits,
                            std::vector<double> &solution,
                            moveit_msgs::MoveItErrorCodes &error_code) const;

      /**
       * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
       * This particular method is intended for "searching" for a solutions by stepping through the redundancy
       * (or other numerical routines).
       * @param ik_pose the desired pose of the link
       * @param ik_seed_state an initial guess solution for the inverse kinematics
       * @return True if a valid solution was found, false otherwise
       */
      // searchPositionIK #3 - used by planning scene warehouse
      bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,
                                                                                               moveit_msgs::MoveItErrorCodes &error_code)> &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code) const;

        /**
         * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
         * This particular method is intended for "searching" for a solutions by stepping through the redundancy
         * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
         * around those specified in the seed state are admissible and need to be searched.
         * @param ik_pose the desired pose of the link
         * @param ik_seed_state an initial guess solution for the inverse kinematics
         * @param consistency_limit the distance that the redundancy can be from the current position 
         * @return True if a valid solution was found, false otherwise
         */
        bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_seed_state,
                              double timeout,
                              const std::vector<double> &consistency_limits,
                              std::vector<double> &solution,
                              const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,
                                                                                                  moveit_msgs::MoveItErrorCodes &error_code)> &solution_callback,
                              moveit_msgs::MoveItErrorCodes &error_code) const;

    private:

      bool initialize(const std::string& group_name, const std::string& base_name, const std::string& tip_name, double search_discretization);

      /**
       * @brief Calls the IK solver from IKFast
       * @return The number of solutions found
       */
      int solve(KDL::Frame &pose_frame, const std::vector<double> &vfree, IkSolutionList<IkReal> &solutions) const;

      /**
       * @brief Gets a specific solution from the set
       */
      void getSolution(const IkSolutionList<IkReal> &solutions, int i, std::vector<double>& solution) const;

      double harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const;
      double harmonize_old(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const;
      //void getOrderedSolutions(const std::vector<double> &ik_seed_state, std::vector<std::vector<double> >& solslist);
      void getClosestSolution(const IkSolutionList<IkReal> &solutions, const std::vector<double> &ik_seed_state, std::vector<double> &solution) const;
      void fillFreeParams(int count, int *array);
      bool getCount(int &count, const int &max_count, const int &min_count) const;

  }; // end class

  bool IKFastKinematicsPlugin::initialize(const std::string& group_name,
                                          const std::string& base_name,
                                          const std::string& tip_name,
                                          double search_discretization) 
  {
    setValues(group_name, base_name, tip_name,search_discretization);

    ros::NodeHandle node_handle("~/"+group_name);

    std::string robot;
    node_handle.param("robot",robot,std::string());
    
    // IKFast56/61
    fillFreeParams( GetNumFreeParameters(), GetFreeParameters() );
    num_joints_ = GetNumJoints();

    if(free_params_.size()>1){
      ROS_FATAL("Only one free joint paramter supported!");
      return false;
    }
      
    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,std::string("robot_description"));
    node_handle.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG("Reading xml file from parameter server\n");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
    {
      ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
      return false;
    }

    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    boost::shared_ptr<urdf::Link> link = boost::const_pointer_cast<urdf::Link>(robot_model.getLink(tip_frame_));
    while(link->name != base_frame_ && joint_names_.size() <= num_joints_){
      //	ROS_INFO("link %s",link->name.c_str());
      link_names_.push_back(link->name);
      boost::shared_ptr<urdf::Joint> joint = link->parent_joint;
      if(joint){
        if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
	  joint_names_.push_back(joint->name);
          float lower, upper;
          int hasLimits;
          if ( joint->type != urdf::Joint::CONTINUOUS ) {
            if(joint->safety) {
              lower = joint->safety->soft_lower_limit; 
              upper = joint->safety->soft_upper_limit;
            } else {
              lower = joint->limits->lower;
              upper = joint->limits->upper;
            }
            hasLimits = 1;
          } else {
            lower = -M_PI;
            upper = M_PI;
            hasLimits = 0;
          }
          if(hasLimits) {
            joint_has_limits_vector_.push_back(true);
            joint_min_vector_.push_back(lower);
            joint_max_vector_.push_back(upper);
          } else {
            joint_has_limits_vector_.push_back(false);
            joint_min_vector_.push_back(-M_PI);
            joint_max_vector_.push_back(M_PI);
          }
        }
      } else{
        ROS_WARN("no joint corresponding to %s",link->name.c_str());
      }
      link = link->getParent();
    }
    
    if(joint_names_.size() != num_joints_){
      ROS_FATAL("Joints number mismatch.");
      return false;
    }
      
    std::reverse(link_names_.begin(),link_names_.end());
    std::reverse(joint_names_.begin(),joint_names_.end());
    std::reverse(joint_min_vector_.begin(),joint_min_vector_.end());
    std::reverse(joint_max_vector_.begin(),joint_max_vector_.end());
    std::reverse(joint_has_limits_vector_.begin(), joint_has_limits_vector_.end());

    for(size_t i=0; i <num_joints_; ++i)
      ROS_INFO_STREAM(joint_names_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i] << " " << joint_has_limits_vector_[i]);

    return true;
  }

  int IKFastKinematicsPlugin::solve(KDL::Frame &pose_frame, const std::vector<double> &vfree, IkSolutionList<IkReal> &solutions) const
  {
    // IKFast56/61
    solutions.Clear();

    //KDL::Rotation rot = KDL::Rotation::RotY(M_PI/2);
    KDL::Rotation orig = pose_frame.M;
    KDL::Rotation mult = orig;//*rot;

    double vals[9];
    vals[0] = mult(0,0);
    vals[1] = mult(0,1);
    vals[2] = mult(0,2);
    vals[3] = mult(1,0);
    vals[4] = mult(1,1);
    vals[5] = mult(1,2);
    vals[6] = mult(2,0);
    vals[7] = mult(2,1);
    vals[8] = mult(2,2);

    double trans[3];
    trans[0] = pose_frame.p[0];//-.18;
    trans[1] = pose_frame.p[1];
    trans[2] = pose_frame.p[2];

    // IKFast56/61
    ComputeIk(trans, vals, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
    return solutions.GetNumSolutions();
  }

  void IKFastKinematicsPlugin::getSolution(const IkSolutionList<IkReal> &solutions, int i, std::vector<double>& solution) const
  {
    solution.clear();
    solution.resize(num_joints_);

    // IKFast56/61
    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
    std::vector<IkReal> vsolfree( sol.GetFree().size() );
    sol.GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);

    // std::cout << "solution " << i << ":" ;
    // for(int j=0;j<num_joints_; ++j)
    //   std::cout << " " << solution[j];
    // std::cout << std::endl;
	  
    //ROS_ERROR("%f %d",solution[2],vsolfree.size());
  }

  double IKFastKinematicsPlugin::harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const
  {
    double dist_sqr = 0;
    std::vector<double> ss = ik_seed_state;
    for(size_t i=0; i< ik_seed_state.size(); ++i)
    {
      while(ss[i] > 2*M_PI) {
        ss[i] -= 2*M_PI;
      }
      while(ss[i] < 2*M_PI) {
        ss[i] += 2*M_PI;
      }
      while(solution[i] > 2*M_PI) {
        solution[i] -= 2*M_PI;
      }
      while(solution[i] < 2*M_PI) {
        solution[i] += 2*M_PI;
      }
      dist_sqr += fabs(ik_seed_state[i] - solution[i]);
    }
    return dist_sqr;
  }

  double IKFastKinematicsPlugin::harmonize_old(const std::vector<double> &ik_seed_state, std::vector<double> &solution) const
  {
    double dist_sqr = 0;
    for(size_t i=0; i< ik_seed_state.size(); ++i){
      double diff = ik_seed_state[i] - solution[i];
      if( diff > M_PI ) solution[i]+=2*M_PI; 
      else if (diff < -M_PI) solution[i]-=2*M_PI;
      diff = ik_seed_state[i] - solution[i];
      dist_sqr += fabs(diff);
    }
    return dist_sqr;
  }
  
  // void IKFastKinematicsPlugin::getOrderedSolutions(const std::vector<double> &ik_seed_state, 
  //                                  std::vector<std::vector<double> >& solslist)
  // {
  //   std::vector<double> 
  //   double mindist = 0;
  //   int minindex = -1;
  //   std::vector<double> sol;
  //   for(size_t i=0;i<solslist.size();++i){
  //     getSolution(i,sol);
  //     double dist = harmonize(ik_seed_state, sol);
  //     //std::cout << "dist[" << i << "]= " << dist << std::endl;
  //     if(minindex == -1 || dist<mindist){
  //       minindex = i;
  //       mindist = dist;
  //     }
  //   }
  //   if(minindex >= 0){
  //     getSolution(minindex,solution);
  //     harmonize(ik_seed_state, solution);
  //     index = minindex;
  //   }
  // }

  void IKFastKinematicsPlugin::getClosestSolution(const IkSolutionList<IkReal> &solutions, const std::vector<double> &ik_seed_state, std::vector<double> &solution) const
  {
    double mindist = DBL_MAX;
    int minindex = -1;
    std::vector<double> sol;

    // IKFast56/61
    for(size_t i=0; i < solutions.GetNumSolutions(); ++i)
    {
      getSolution(solutions, i,sol);
      double dist = harmonize(ik_seed_state, sol);
      ROS_INFO_STREAM("Dist " << i << " dist " << dist);
      //std::cout << "dist[" << i << "]= " << dist << std::endl;
      if(minindex == -1 || dist<mindist){
        minindex = i;
        mindist = dist;
      }
    }
    if(minindex >= 0){
      getSolution(solutions, minindex,solution);
      harmonize(ik_seed_state, solution);
    }
  }

  void IKFastKinematicsPlugin::fillFreeParams(int count, int *array)
  { 
    free_params_.clear(); 
    for(int i=0; i<count;++i) free_params_.push_back(array[i]); 
  }
  
  bool IKFastKinematicsPlugin::getCount(int &count, const int &max_count, const int &min_count) const
  {
    if(count > 0)
    {
      if(-count >= min_count)
      {   
        count = -count;
        return true;
      }
      else if(count+1 <= max_count)
      {
        count = count+1;
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      if(1-count <= max_count)
      {
        count = 1-count;
        return true;
      }
      else if(count-1 >= min_count)
      {
        count = count -1;
        return true;
      }
      else
        return false;
    }
  }

  bool IKFastKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                             const std::vector<double> &joint_angles, 
                                             std::vector<geometry_msgs::Pose> &poses) const
  {
    KDL::Frame p_out;
    if(link_names.size() == 0) {
      ROS_WARN_STREAM("Link names with nothing");
      return false;
    }

    if(link_names.size()!=1 || link_names[0]!=tip_frame_){
      ROS_ERROR("Can compute FK for %s only",tip_frame_.c_str());
      return false;
    }
	
    bool valid = true;

    IkReal eerot[9],eetrans[3];
    IkReal angles[joint_angles.size()];
    for (unsigned char i=0; i < joint_angles.size(); i++) angles[i] = joint_angles[i];
  
    // IKFast56/61
    ComputeFk(angles,eetrans,eerot);

    for(int i=0; i<3;++i) p_out.p.data[i] = eetrans[i];
    for(int i=0; i<9;++i) p_out.M.data[i] = eerot[i];
    poses.resize(1);
    tf::PoseKDLToMsg(p_out,poses[0]);	

    return valid;
  }  

  bool IKFastKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                             const std::vector<double> &ik_seed_state,
                                             std::vector<double> &solution,
                                             moveit_msgs::MoveItErrorCodes &error_code) const
  {
    std::vector<double> vfree(free_params_.size());
    for(std::size_t i = 0; i < free_params_.size(); ++i){
      int p = free_params_[i];
      // ROS_ERROR("%u is %f",p,ik_seed_state[p]);
      vfree[i] = ik_seed_state[p];
    }

    KDL::Frame frame;
    tf::PoseMsgToKDL(ik_pose,frame);

    IkSolutionList<IkReal> solutions;
    int numsol = solve(frame,vfree,solutions);
		
    if(numsol){
      for(int s = 0; s < numsol; ++s)
      {
        std::vector<double> sol;
        getSolution(solutions,s,sol);
        //printf("Sol %d: %e   %e   %e   %e   %e   %e    \n", s, sol[0], sol[1], sol[2], sol[3], sol[4], sol[5] );

        bool obeys_limits = true;
        for(unsigned int i = 0; i < sol.size(); i++) {
          // Add tolerance to limit check
          if(joint_has_limits_vector_[i] && ( (sol[i] < (joint_min_vector_[i]-LIMIT_TOLERANCE)) || 
                                                                     (sol[i] > (joint_max_vector_[i]+LIMIT_TOLERANCE)) ) ) 
          {
            // One element of solution is not within limits
            obeys_limits = false;
            //ROS_INFO_STREAM("      Num " << i << " value " << sol[i] << " has limit: " << joint_has_limits_vector_[i] << "  being  " << joint_min_vector_[i] << " to " << joint_max_vector_[i] << "\n");
            break;
          }
        }
        if(obeys_limits) {
          // All elements of solution obey limits
          getSolution(solutions,s,solution);
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
          //printf("obeys limits \n\n");
          return true;
        }
      }
    }else{
      //printf("No IK solution \n");
    }
	
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
    return false;
  }

  // searchPositionIK #1
  bool IKFastKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                const std::vector<double> &ik_seed_state,
                                                double timeout,
                                                std::vector<double> &solution,
                                                moveit_msgs::MoveItErrorCodes &error_code) const
  {
    if(free_params_.size()==0){
      return getPositionIK(ik_pose, ik_seed_state,solution, error_code);
    }
	
    KDL::Frame frame;
    tf::PoseMsgToKDL(ik_pose,frame);

    std::vector<double> vfree(free_params_.size());

    ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
    int counter = 0;

    double initial_guess = ik_seed_state[free_params_[0]];
    vfree[0] = initial_guess;

    int num_positive_increments = (joint_max_vector_[free_params_[0]]-initial_guess)/search_discretization_;
    int num_negative_increments = (initial_guess-joint_min_vector_[free_params_[0]])/search_discretization_;

    //ROS_INFO_STREAM("Free param is " << free_params_[0] << " initial guess is " << initial_guess << " " << num_positive_increments << " " << num_negative_increments);

    while(1) {
      IkSolutionList<IkReal> solutions; 
      int numsol = solve(frame,vfree, solutions);
      //ROS_INFO_STREAM("Solutions number is " << numsol);
      //ROS_INFO("%f",vfree[0]);
	    
      if(numsol > 0){
        for(int s = 0; s < numsol; ++s){
          std::vector<double> sol;
          getSolution(solutions,s,sol);

          bool obeys_limits = true;
          for(unsigned int i = 0; i < sol.size(); i++) {
            if(joint_has_limits_vector_[i] && (sol[i] < joint_min_vector_[i] || sol[i] > joint_max_vector_[i])) {
              obeys_limits = false;
              break;
            }
            //ROS_INFO_STREAM("Num " << i << " value " << sol[i] << " has limits " << joint_has_limits_vector_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i]);
          }
          if(obeys_limits) {
            getSolution(solutions,s,solution);
            error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
            return true;
          }
        }
      }
      
      // if(numsol > 0){
      //   for(unsigned int i = 0; i < sol.size(); i++) {
      //     if(i == 0) {
      //       //ik_solver_->getClosestSolution(ik_seed_state,solution);
      //       getClosestSolution(ik_seed_state,solution);
      //     } else {
      //       //ik_solver_->getSolution(s,sol);  
      //       getSolution(s,sol);          
      //     }
      //   }
      //   bool obeys_limits = true;
      //   for(unsigned int i = 0; i < solution.size(); i++) {
      //     if(joint_has_limits_vector_[i] && (solution[i] < joint_min_vector_[i] || solution[i] > joint_max_vector_[i])) {
      //       obeys_limits = false;
      //       break;
      //     }
      //     //ROS_INFO_STREAM("Num " << i << " value " << sol[i] << " has limits " << joint_has_limits_vector_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i]);
      //   }
      //   if(obeys_limits) {
      //     error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      //     return true;
      //   }
      // }
      if(!getCount(counter, num_positive_increments, num_negative_increments)) {
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
        return false;
      }
      
      vfree[0] = initial_guess+search_discretization_*counter;
      ROS_DEBUG_STREAM(counter << " " << vfree[0]);
    }
    //shouldn't ever get here
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
    return false;
  }      

  // searchPositionIK #2
  bool IKFastKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                const std::vector<double> &ik_seed_state,
                                                double timeout,
                                                const std::vector<double> &consistency_limits,
                                                std::vector<double> &solution,
                                               moveit_msgs::MoveItErrorCodes &error_code) const
  {
    if(free_params_.size()==0){
      //TODO - how to check consistency when there are no free params?
      return getPositionIK(ik_pose, ik_seed_state,solution, error_code);
      ROS_WARN_STREAM("No free parameters, so can't search");
    }
	
    KDL::Frame frame;
    tf::PoseMsgToKDL(ik_pose,frame);

    std::vector<double> vfree(free_params_.size());

    ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
    int counter = 0;

    double initial_guess = ik_seed_state[free_params_[0]];
    vfree[0] = initial_guess;

    // replaced consistency_limit (scalar) w/ consistency_limits (vector).
    // Assume [0]th free_params element for now.  Probably wrong.
    double max_limit = fmin(joint_max_vector_[free_params_[0]], initial_guess+consistency_limits[free_params_[0]]);
    double min_limit = fmax(joint_min_vector_[free_params_[0]], initial_guess-consistency_limits[free_params_[0]]);

    int num_positive_increments = (int)((max_limit-initial_guess)/search_discretization_);
    int num_negative_increments = (int)((initial_guess-min_limit)/search_discretization_);

    ROS_DEBUG_STREAM("Free param is " << free_params_[0] << " initial guess is " << initial_guess << " " << num_positive_increments << " " << num_negative_increments);

    while(1) {
      IkSolutionList<IkReal> solutions;
      int numsol = solve(frame,vfree, solutions);
      //ROS_INFO_STREAM("Solutions number is " << numsol);
      //ROS_INFO("%f",vfree[0]);
	    
      if(numsol > 0){
        for(int s = 0; s < numsol; ++s){
          std::vector<double> sol;
          //ik_solver_->getSolution(s,sol);
          getSolution(solutions, s,sol);
          bool obeys_limits = true;
          for(unsigned int i = 0; i < sol.size(); i++) {
            if(joint_has_limits_vector_[i] && (sol[i] < joint_min_vector_[i] || sol[i] > joint_max_vector_[i])) {
              obeys_limits = false;
              break;
            }
            //ROS_INFO_STREAM("Num " << i << " value " << sol[i] << " has limits " << joint_has_limits_vector_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i]);
          }
          if(obeys_limits) {
            //ik_solver_->getSolution(s,solution);
            getSolution(solutions, s,solution);
            error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
            return true;
          }
        }
      }
      
      // if(numsol > 0){
      //   for(unsigned int i = 0; i < sol.size(); i++) {
      //     if(i == 0) {
      //       //ik_solver_->getClosestSolution(ik_seed_state,solution);
      //       getClosestSolution(ik_seed_state,solution);
      //     } else {
      //       //ik_solver_->getSolution(s,sol);      
      //        getSolution(s,sol);     
      //     }
      //   }
      //   bool obeys_limits = true;
      //   for(unsigned int i = 0; i < solution.size(); i++) {
      //     if(joint_has_limits_vector_[i] && (solution[i] < joint_min_vector_[i] || solution[i] > joint_max_vector_[i])) {
      //       obeys_limits = false;
      //       break;
      //     }
      //     //ROS_INFO_STREAM("Num " << i << " value " << sol[i] << " has limits " << joint_has_limits_vector_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i]);
      //   }
      //   if(obeys_limits) {
      //     error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      //     return true;
      //   }
      // }
      if(!getCount(counter, num_positive_increments, num_negative_increments)) {
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
        return false;
      }
      
      vfree[0] = initial_guess+search_discretization_*counter;
      ROS_DEBUG_STREAM(counter << " " << vfree[0]);
    }
    //shouldn't ever get here
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
    return false;
  }      

  // searchPositionIK #3
  // this is used by planning scene warehouse
  bool IKFastKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                const std::vector<double> &ik_seed_state,
                                                double timeout,
                                                std::vector<double> &solution,
                                                const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,
                                                                                                                 moveit_msgs::MoveItErrorCodes &error_code)> &solution_callback,
                                                moveit_msgs::MoveItErrorCodes &error_code) const
  {
    // If manipulator has no free links
    if(free_params_.size()==0){
      // Find first IK solution, within joint limits
      if(!getPositionIK(ik_pose, ik_seed_state,solution, error_code)) {
        ROS_DEBUG_STREAM("No solution whatsoever");
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
        return false;
      } 
      // check for collisions
      solution_callback(ik_pose,solution,error_code);
      if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_DEBUG_STREAM("Solution passes");
        return true;
      } else {
        ROS_DEBUG_STREAM("Solution has error code " << error_code);
        return false;
      }
    }

    KDL::Frame frame;
    tf::PoseMsgToKDL(ik_pose,frame);

    std::vector<double> vfree(free_params_.size());

    ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
    int counter = 0;

    double initial_guess = ik_seed_state[free_params_[0]];
    vfree[0] = initial_guess;

    int num_positive_increments = (joint_max_vector_[free_params_[0]]-initial_guess)/search_discretization_;
    int num_negative_increments = (initial_guess - joint_min_vector_[free_params_[0]])/search_discretization_;

    ROS_DEBUG_STREAM("Free param is " << free_params_[0] << " initial guess is " << initial_guess << " " << num_positive_increments << " " << num_negative_increments);

    unsigned int solvecount = 0;
    unsigned int countsol = 0;

    ros::WallTime start = ros::WallTime::now();

    std::vector<double> sol;
    while(1) {
      IkSolutionList<IkReal> solutions;
      int numsol = solve(frame,vfree,solutions);

      if(solvecount == 0) {
        if(numsol == 0) {
          ROS_DEBUG_STREAM("Bad solve time is " << ros::WallTime::now()-start);
        } else {
          ROS_DEBUG_STREAM("Good solve time is " << ros::WallTime::now()-start);
        }
      }
      solvecount++;
      if(numsol > 0){
        if(solution_callback.empty()){
          getClosestSolution(solutions, ik_seed_state,solution);
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
          return true;
        }
        
        for(int s = 0; s < numsol; ++s){
          getSolution(solutions, s,sol);
          countsol++;
          bool obeys_limits = true;
          for(unsigned int i = 0; i < sol.size(); i++) {
            if(joint_has_limits_vector_[i] && (sol[i] < joint_min_vector_[i] || sol[i] > joint_max_vector_[i])) {
              obeys_limits = false;
              break;
            }
          }
          if(obeys_limits) {
            solution_callback(ik_pose,sol,error_code);
            if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
              solution = sol;
              ROS_DEBUG_STREAM("Took " << (ros::WallTime::now() - start) << " to return true " << countsol << " " << solvecount);
              return true;
            }
          }
        }
      }
      if(!getCount(counter, num_positive_increments, num_negative_increments)) {
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
        ROS_DEBUG_STREAM("Took " << (ros::WallTime::now() - start) << " to return false " << countsol << " " << solvecount);
        return false;
      }
      vfree[0] = initial_guess+search_discretization_*counter;
      ROS_DEBUG_STREAM(counter << " " << vfree[0]);
    }
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
    return false;
  }      

  // searchPositionIK #4
  bool IKFastKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                const std::vector<double> &ik_seed_state,
                                                double timeout,
                                                const std::vector<double> &consistency_limits,
                                                std::vector<double> &solution,
                                                const boost::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,
                                                                                                                 moveit_msgs::MoveItErrorCodes &error_code)> &solution_callback,
                                                moveit_msgs::MoveItErrorCodes &error_code) const
  {
    if(free_params_.size()==0){
      if(!getPositionIK(ik_pose, ik_seed_state,solution, error_code)) {
        ROS_DEBUG_STREAM("No solution whatsoever");
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
        return false;
      } 
      solution_callback(ik_pose,solution,error_code);
      if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_DEBUG_STREAM("Solution passes");
        return true;
      } else {
        ROS_DEBUG_STREAM("Solution has error code " << error_code);
        return false;
      }
    }

    KDL::Frame frame;
    tf::PoseMsgToKDL(ik_pose,frame);

    std::vector<double> vfree(free_params_.size());

    ros::Time maxTime = ros::Time::now() + ros::Duration(timeout);
    int counter = 0;

    double initial_guess = ik_seed_state[free_params_[0]];
    vfree[0] = initial_guess;

    // moveit replaced consistency_limit (scalar) w/ consistency_limits (vector)
    // Assuming [0]th free_params element.  Probably wrong.
    double max_limit = fmin(joint_max_vector_[free_params_[0]], initial_guess+consistency_limits[free_params_[0]]);
    double min_limit = fmax(joint_min_vector_[free_params_[0]], initial_guess-consistency_limits[free_params_[0]]);

    int num_positive_increments = (int)((max_limit-initial_guess)/search_discretization_);
    int num_negative_increments = (int)((initial_guess-min_limit)/search_discretization_);

    ROS_DEBUG_STREAM("Free param is " << free_params_[0] << " initial guess is " << initial_guess << " " << num_positive_increments << " " << num_negative_increments);

    unsigned int solvecount = 0;
    unsigned int countsol = 0;

    ros::WallTime start = ros::WallTime::now();

    std::vector<double> sol;
    while(1) {
      IkSolutionList<IkReal> solutions;
      int numsol = solve(frame,vfree,solutions);
      if(solvecount == 0) {
        if(numsol == 0) {
          ROS_DEBUG_STREAM("Bad solve time is " << ros::WallTime::now()-start);
        } else {
          ROS_DEBUG_STREAM("Good solve time is " << ros::WallTime::now()-start);
        }
      }
      solvecount++;
      if(numsol > 0){
        if(solution_callback.empty()){
          getClosestSolution(solutions, ik_seed_state,solution);
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
          return true;
        }
        
        for(int s = 0; s < numsol; ++s){
          getSolution(solutions, s,sol);
          countsol++;
          bool obeys_limits = true;
          for(unsigned int i = 0; i < sol.size(); i++) {
            if(joint_has_limits_vector_[i] && (sol[i] < joint_min_vector_[i] || sol[i] > joint_max_vector_[i])) {
              obeys_limits = false;
              break;
            }
          }
          if(obeys_limits) {
            solution_callback(ik_pose,sol,error_code);
            if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
              solution = sol;
              ROS_DEBUG_STREAM("Took " << (ros::WallTime::now() - start) << " to return true " << countsol << " " << solvecount);
              return true;
            }
          }
        }
      }
      if(!getCount(counter, num_positive_increments, num_negative_increments)) {
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
        ROS_DEBUG_STREAM("Took " << (ros::WallTime::now() - start) << " to return false " << countsol << " " << solvecount);
        return false;
      }
      vfree[0] = initial_guess+search_discretization_*counter;
      ROS_DEBUG_STREAM(counter << " " << vfree[0]);
    }
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION; 
    return false;
  }      

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(_ROBOT_NAME___GROUP_NAME__kinematics::IKFastKinematicsPlugin, kinematics::KinematicsBase);

