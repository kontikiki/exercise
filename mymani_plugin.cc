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
    class mymani_plugin: public ModelPlugin
    {
        public: mymani_plugin(){}
        private: physics::ModelPtr model;

        private: physics::JointPtr joint0;
        private: physics::JointPtr joint1;
        private: physics::JointPtr joint2;
        private: physics::JointPtr joint3;
        private: physics::JointPtr joint4;
        private: physics::JointPtr joint5;
        private: physics::JointPtr joint6;
       private: physics::JointPtr joint7;
       
        private: common::PID pid;

        private: std::unique_ptr<ros::NodeHandle> rosNode;
        private: ros::Subscriber rosSub;
        private: ros::CallbackQueue rosQueue;
        private: std::thread rosQueueThread;


        public: virtual void Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
                {
                    if(_model->GetJointCount()==0)
                    {
                        std::cerr<< "Invalid joint count, mymani plugin not loaded\n";
                        return;
                    }

                    this->model=_model;
                    this->joint0=_model->GetJoints()[0];
                    this->joint1=_model->GetJoints()[1];
                    this->joint2=_model->GetJoints()[2];
                    this->joint3=_model->GetJoints()[3];
                    this->joint4=_model->GetJoints()[4];
                    /*
                    this->joint5=_model->GetJoints()[5];
                    this->joint6=_model->GetJoints()[6];
                    this->joint7=_model->GetJoints()[7];
                    */
                    this->pid=common::PID(0,0,0);

                    this->model->GetJointController()->SetVelocityPID(
                            this->joint0->GetScopedName(),this->pid);
                    std::cerr<<"joint name is "<<this->joint0->GetScopedName()<<std::endl;

                    this->model->GetJointController()->SetVelocityPID(
                            this->joint1->GetScopedName(),this->pid);
                    std::cerr<<"joint name is "<<this->joint1->GetScopedName()<<std::endl;

                    this->model->GetJointController()->SetVelocityPID(
                            this->joint2->GetScopedName(),this->pid);
                    std::cerr<<"joint name is "<<this->joint2->GetScopedName()<<std::endl;

                    this->model->GetJointController()->SetVelocityPID(
                            this->joint3->GetScopedName(),this->pid);
                    std::cerr<<"joint name is "<<this->joint3->GetScopedName()<<std::endl;

                    this->model->GetJointController()->SetVelocityPID(
                            this->joint4->GetScopedName(),this->pid);
                    std::cerr<<"joint name is "<<this->joint4->GetScopedName()<<std::endl;

                    this->model->GetJointController()->SetVelocityPID(
                            this->joint5->GetScopedName(),this->pid);
                    std::cerr<<"joint name is "<<this->joint5->GetScopedName()<<std::endl;

                    this->model->GetJointController()->SetVelocityPID(
                            this->joint6->GetScopedName(),this->pid);
                    std::cerr<<"joint name is "<<this->joint6->GetScopedName()<<std::endl;

                    this->model->GetJointController()->SetVelocityPID(
                            this->joint7->GetScopedName(),this->pid);
                    std::cerr<<"joint name is "<<this->joint7->GetScopedName()<<std::endl;

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
                                boost::bind(&mymani_plugin::OnRosMsg,this,_1),
                                ros::VoidPtr(),&this->rosQueue);
                                std::cerr<<"model name:"<< this->model->GetName()<<std::endl;
                    this->rosSub=this->rosNode->subscribe(so);
                                boost::bind(&mymani_plugin::OnRosMsg,this,_1),

                    this->rosQueueThread=
                        std::thread(std::bind(&mymani_plugin::QueueThread,this));
                }

        public: void setVelocity04(const float &_vel)
                {
                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint0->GetScopedName(),_vel);

             //      std::cerr<<"joint name is "<<this->joint0->GetScopedName()<<std::endl;

                    
                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint1->GetScopedName(),_vel);

              //      std::cerr<<"joint name is "<<this->joint1->GetScopedName()<<std::endl;


                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint2->GetScopedName(),_vel);

                //    std::cerr<<"joint name is "<<this->joint2->GetScopedName()<<std::endl;



                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint3->GetScopedName(),_vel);

                //    std::cerr<<"joint name is "<<this->joint3->GetScopedName()<<std::endl;


                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint4->GetScopedName(),_vel);

                //    std::cerr<<"joint name is "<<this->joint4->GetScopedName()<<std::endl;

                }

        public:  void setVelocity5(const float &_vel){
                    this->model->GetJointController()->SetVelocityTarget(
                            this->joint5->GetScopedName(),_vel);
                  //  std::cerr<<"joint name is "<<this->joint5->GetScopedName()<<std::endl;
                }

        public: void setVelocity6(const float &_vel){
                   this->model->GetJointController()->SetVelocityTarget(
                            this->joint6->GetScopedName(),_vel);
                //   std::cerr<<"joint name is "<<this->joint6->GetScopedName()<<std::endl;
                }
                

        public: void OnRosMsg(const geometry_msgs::Vector3ConstPtr &_msg){
                    this->setVelocity04(_msg->x);
                    this->setVelocity5(_msg->y);
                    this->setVelocity6(_msg->z);
                }

        private: void QueueThread(){
                     static const double timeout=0.01;
                     while (this->rosNode->ok()){
                         this->rosQueue.callAvailable(ros::WallDuration(timeout));
                     }
                 }
    };
    GZ_REGISTER_MODEL_PLUGIN(mymani_plugin)
}
#endif
