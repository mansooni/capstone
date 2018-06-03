#include <functional>
#include <gazebo/gazebo.hh>

#include <sdf/sdf.hh>
#include "gazebo/msgs/msgs.hh"

#include <boost/bind.hpp>
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Joint.hh"

#include "gazebo/common/common.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

namespace gazebo
{
    class Blocking : public ModelPlugin
    {

        // Pointer to the model
        private: physics::ModelPtr model;
        private: sdf::ElementPtr sdf;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        public:
            transport::NodePtr node;
            std::string topicName = "/gazebo/vrc_task_1/blockingbar";
            transport::SubscriberPtr topicSub;
            physics::JointPtr blockingBarJoint;
            bool isFirst = false;
            bool isBlock = false;
            int topic = 0;
            float blockVel = 0.0;
            common::Time time;

        private: void TopicSubscriber(ConstGzStringPtr &_msg)
        {

            try
            {
                this->topic = std::stoi(_msg->data());
                if (not this->isFirst){
                    this->isFirst = true;
                }
            }
            catch(...)
            {
                gzerr << "Unable to process elevator message[" << _msg->data() << "]\n";
            }
        }

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            printf("Load Plugin is started!! \n");
            GZ_ASSERT(_parent, "ElevatorPlugin model pointer is NULL");
            GZ_ASSERT(_sdf, "ElevatorPlugin sdf pointer is NULL");

            this->model = _parent;
            this->sdf = _sdf;

            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetName());

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Blocking::OnUpdate, this, _1));

            this->topicSub = this->node->Subscribe(this->topicName,&Blocking::TopicSubscriber, this);
            std::string JointName = "blocking_bar_joint";
            this->blockingBarJoint = this->model->GetJoint(JointName);
        }

        public: void OnUpdate(const common::UpdateInfo & /*_info*/)
        {
            // Apply a small linear velocity to the model.
            this->blockingBarJoint->SetVelocity(0,this->blockVel);
            ControlBlockingBar();
        }
        public: void ControlBlockingBar(){
            if(this->isFirst and not this->isBlock){
                this->time = common::Time::GetWallTime() + 10;
                this->isBlock = true;
            }
            if(this->isFirst and this->isBlock and common::Time::GetWallTime() <= this->time){
                this->blockVel = 0.5;
            }
            else if(this->isFirst and this->isFirst and this->isBlock and common::Time::GetWallTime() > this->time){
                this->isBlock = false;
                this->blockVel = -0.5;
                this->isFirst = false;
            }
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Blocking)
}
