//
// Created by osboxes on 3/29/19.
//

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo {

    constexpr double kAirDensity = 1.24;
    constexpr double kBoxDragCoefficient = 1.05;

    class AirResistanceBox : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {


            // Store the pointer to the model
            this->model = _parent;
            this->my_sdf = _sdf;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&AirResistanceBox::OnUpdate, this));
        }

        // Called by the world update start event
    public:
        void OnUpdate() {

            for (auto &link : this->model->GetLinks()) {
                auto size_box = link->GetCollisionBoundingBox().GetSize();

                auto velocity_of_link = link->GetRelativeLinearVel();

                gazebo::math::Vector3 drag_force(
                        0.5 * kAirDensity * -sgn(velocity_of_link.x) * velocity_of_link.x * velocity_of_link.x *
                        kBoxDragCoefficient * size_box.y * size_box.z,
                        0.5 * kAirDensity * -sgn(velocity_of_link.y) * velocity_of_link.y * velocity_of_link.y *
                        kBoxDragCoefficient * size_box.z * size_box.x,
                        0.5 * kAirDensity * -sgn(velocity_of_link.z) * velocity_of_link.z * velocity_of_link.z *
                        kBoxDragCoefficient * size_box.x * size_box.y);


                link->AddRelativeForce(drag_force);
            }

        }

        // Pointer to the model
    private:
        physics::ModelPtr model;


        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
        sdf::ElementPtr my_sdf;

        template<typename T>
        int sgn(T val) {
            return (T(0) < val) - (val < T(0));
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(AirResistanceBox)
}
