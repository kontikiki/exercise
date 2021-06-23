#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
    class ContactPlugin : public SensorPlugin
    {
        public: ContactPlugin();
        public: virtual ~ContactPlugin();

        public: virtual void Load(sensors::SensorPtr _sensor,sdf::ElementPtr _sdf);

        private: virtual void OnUpdate();

        private: sensors::ContactSensorPtr parentSensor;

        private: event::ConnectionPtr updateConnection;
    };
}

#endif
