
#ifndef _MYPANTILT2_PLUGIN_HH_
#define _MYPANTILT2_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "geometry_msgs/Twist.h"
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo/rendering/rendering.hh>


namespace gazebo
{
    class mypantilt2_plugin: public ModelPlugin
    {
        public: mypantilt2_plugin(){}
        public: ~mypantilt2_plugin(){}
        

        private: physics::ModelPtr model;
        private: physics::JointPtr joint0;
        private: physics::JointPtr joint1;
        private: physics::JointPtr joint2;
        private: physics::JointControllerPtr jc0;
        //private: common::PID pid;

        private: std::unique_ptr<ros::NodeHandle> rosNode;
        private: ros::Subscriber rosSub;
        private: ros::CallbackQueue rosQueue;
        private: std::thread rosQueueThread;
       


        public: virtual void Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
                {
                      

                    if(_model->GetJointCount()==0)
                    {
                        std::cerr<< "Invalid joint count, myrobot plugin not loaded\n";
                        return;
                    }

                                
                    this->model=_model;
                    this->jc0=model->GetJointController();  
                    this->joint0=_model->GetJoints()[0];
                    this->joint1=_model->GetJoints()[1];
                    this->joint2=_model->GetJoints()[2];

/*
                    this->pid=common::PID(0.1,0,0);
                    this->model->GetJointController()->SetVelocityPID(
                            this->joint0->GetScopedName(),this->pid);
                    this->model->GetJointController()->SetVelocityPID(
                            this->joint1->GetScopedName(),this->pid);
                    this->model->GetJointController()->SetVelocityPID(
                            this->joint2->GetScopedName(),this->pid);
*/
                      if(!ros::isInitialized())
                    {
                        int argc=0;
                        char **argv=NULL;

                        ros::init(argc,argv,"gazebo_client",
                                ros::init_options::NoSigintHandler);


                    }

 this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

                    ros::SubscribeOptions so=
                        ros::SubscribeOptions::create<geometry_msgs::Twist>(
                                "/"+this->model->GetName()+"/angle_cmd",
                                1,
                                boost::bind(&mypantilt2_plugin::OnRosMsg,this,_1),
                                ros::VoidPtr(),&this->rosQueue);
                    this->rosSub=this->rosNode->subscribe(so);

                    this->rosQueueThread=
                        std::thread(std::bind(&mypantilt2_plugin::QueueThread,this));
                }

        public: void setPosition0(const float &_angle)
                {
                            jc0->SetJointPosition(joint0,_angle);
                    std::cerr<<"joint name is "<<this->joint0->GetScopedName()<<std::endl;
                }

        public:  void setPosition1(const float &_angle){
                    jc0->SetJointPosition(joint1,_angle);
                    std::cerr<<"joint name is "<<this->joint1->GetScopedName()<<std::endl;
                }

       public: void setPosition2(const float &_angle){

                    jc0->SetJointPosition(joint2,_angle);
                   std::cerr<<"joint name is "<<this->joint2->GetScopedName()<<std::endl;
               }

        public: void OnRosMsg(const geometry_msgs::TwistConstPtr& _msg){

            std::cerr<<"OnrosMsg process"<<std::endl;
            if((_msg->angular.x)!=0){
                    this->setPosition0(_msg->angular.x);
            }
             if((_msg->angular.y)!=0){
                    this->setPosition1(_msg->angular.y);
            }
            if((_msg->angular.z)!=0){
                    this->setPosition2(_msg->angular.z);
            }      
                      
                }

        private: void QueueThread(){
                     static const double timeout=0.01;
                     while (this->rosNode->ok()){
                         this->rosQueue.callAvailable(ros::WallDuration(timeout));
                     }
                 }
    };
    GZ_REGISTER_MODEL_PLUGIN(mypantilt2_plugin)
}
#endif