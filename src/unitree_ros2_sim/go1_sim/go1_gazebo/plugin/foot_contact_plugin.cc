#include <string>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace gazebo
{
    class UnitreeFootContactPlugin : public SensorPlugin
    {
    public:
        UnitreeFootContactPlugin() : SensorPlugin() {}
        ~UnitreeFootContactPlugin() {}

        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
            // Ensure the parent sensor is a ContactSensor
            this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
            if (!this->parentSensor) {
                gzerr << "UnitreeFootContactPlugin requires a ContactSensor.\n";
                return;
            }

            // Initialize ROS 2 node
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }
            this->ros_node = rclcpp::Node::make_shared("unitree_foot_contact_plugin");

            // Create publisher
            this->force_pub = this->ros_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
                "/visual/" + _sensor->Name() + "/the_force", 100);

            // Connect to the sensor update event
            this->update_connection = this->parentSensor->ConnectUpdated(
                std::bind(&UnitreeFootContactPlugin::OnUpdate, this));

            this->parentSensor->SetActive(true); // Ensure the sensor is active

            count = 0;
            Fx = 0;
            Fy = 0;
            Fz = 0;

            RCLCPP_INFO(this->ros_node->get_logger(), "Loaded %s plugin.", _sensor->Name().c_str());
        }

    private:
        void OnUpdate()
        {
            msgs::Contacts contacts = this->parentSensor->Contacts();
            count = contacts.contact_size();

            // Process contacts
            for (int i = 0; i < count; ++i) {
                if (contacts.contact(i).position_size() != 1) {
                    RCLCPP_ERROR(this->ros_node->get_logger(), "Contact count isn't correct!!!!");
                }
                for (int j = 0; j < contacts.contact(i).position_size(); ++j) {
                    Fx += contacts.contact(i).wrench(0).body_1_wrench().force().x(); // Local coordinates
                    Fy += contacts.contact(i).wrench(0).body_1_wrench().force().y();
                    Fz += contacts.contact(i).wrench(0).body_1_wrench().force().z();
                }
            }

            // Compute average force and publish
            geometry_msgs::msg::WrenchStamped force_msg;
            if (count != 0) {
                force_msg.wrench.force.x = Fx / static_cast<double>(count);
                force_msg.wrench.force.y = Fy / static_cast<double>(count);
                force_msg.wrench.force.z = Fz / static_cast<double>(count);
                count = 0;
                Fx = 0;
                Fy = 0;
                Fz = 0;
            } else {
                force_msg.wrench.force.x = 0;
                force_msg.wrench.force.y = 0;
                force_msg.wrench.force.z = 0;
            }

            this->force_pub->publish(force_msg);
        }

    private:
        rclcpp::Node::SharedPtr ros_node;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub;
        event::ConnectionPtr update_connection;
        sensors::ContactSensorPtr parentSensor;
        int count = 0;
        double Fx = 0, Fy = 0, Fz = 0;
    };

    GZ_REGISTER_SENSOR_PLUGIN(UnitreeFootContactPlugin)
}
