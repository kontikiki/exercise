#ifndef _MYROBOT_PLUGIN_HH_
#define _MYROBOT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"


namespace gazebo
{
    class myrobot_plugin: public ModelPlugin
    {
        public: myrobot_plugin(){}
        private: physics::ModelPtr model;
        private: physics::JointPtr joint0;
        private: physics::JointPtr joint1;
        private: physics::JointPtr joint2;
        private: common::PID pid;

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
                    this->joint0=_model->GetJoints()[0];
                    this->joint1=_model->GetJoints()[1];
                    this->joint2=_model->GetJoints()[2];
                    this->pid=common::PID(0.1,0,0);
                    this->model->GetJointController()->SetVelocityPID(
                            this->joint0->GetScopedName(),this->pid);
                    this->model->GetJointController()->SetVelocityPID(
                            this->joint1->GetScopedName(),this->pid);
                    this->model->GetJointController()->SetVelocityPID(
                            this->joint2->GetScopedName(),this->pid);

                    double velocity=0;
                    
                    if(!ros::isInitialized())
                    {
                        int argc=0;
                        char **argv=NULL;

                        ros::init(argc,argv,"gazebo_client",
                                ros::init_options::NoSigintHandler);

                    }
            
                    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

                    ros::SubscribeOptions so=
                        ros::SubscribeOptions::create<geometry_msgs::Vector3>(
                                "/"+this->model->GetName()+"/vel_cmd",
                                1,
                                boost::bind(&myrobot_plugin::OnRosMsg,this,_1),
                                ros::VoidPtr(),&this->rosQueue);
                    this->rosSub=this->rosNode->subscribe(so);

                    this->rosQueueThread=
                        std::thread(std::bind(&myrobot_plugin::QueueThread,this));
                }

        public: void setVelocity0(const float &_vel)
                {
                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint0->GetScopedName(),_vel);

                    std::cerr<<"joint name is "<<this->joint0->GetScopedName()<<std::endl;
                }

        public:  void setVelocity1(const float &_vel){
                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint1->GetScopedName(),_vel);
                    std::cerr<<"joint name is "<<this->joint1->GetScopedName()<<std::endl;
                }

       public: void setVelocity2(const float &_vel){
                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint2->GetScopedName(),_vel);
                   std::cerr<<"joint name is "<<this->joint2->GetScopedName()<<std::endl;
               }

        public: void OnRosMsg(const geometry_msgs::Vector3ConstPtr &_msg){
                   if((_msg->x)>0){
                    this->setVelocity0(_msg->x);
            }
             if((_msg->y)>0){
                    this->setVelocity1(_msg->y);
            }
            if((_msg->z)>0){
                    this->setVelocity2(_msg->z);
            } 
                }

        private: void QueueThread(){
                     static const double timeout=0.01;
                     while (this->rosNode->ok()){
                         this->rosQueue.callAvailable(ros::WallDuration(timeout));
                     }
                 }
    };
    GZ_REGISTER_MODEL_PLUGIN(myrobot_plugin)
}
#endif
