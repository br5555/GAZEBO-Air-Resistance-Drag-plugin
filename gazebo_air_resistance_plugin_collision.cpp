//
// Created by osboxes on 3/29/19.
//

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo {


    constexpr double kBoxDragCoefficient = 1.05;

    class AirResistanceBox : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {


            // Store the pointer to the model
            this->model = _parent;
            this->my_sdf = _sdf;


            if(my_sdf->HasElement("my_air_density"))
                kAirDensity =  std::stod( my_sdf->GetElement("my_air_density")->GetValue()->GetAsString() );

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&AirResistanceBox::OnUpdate, this));

            m_angular_drag_coeff = 5.0;
        }

        // Called by the world update start event
    public:
        void OnUpdate() {

            for (auto &link : this->model->GetLinks()) {
                auto size_box = link->GetCollisionBoundingBox().GetSize();

                auto velocity_of_link = link->GetRelativeLinearVel();

                ignition::math::Vector3d drag_force(
                        0.5 * kAirDensity * -sgn(velocity_of_link.x) * velocity_of_link.x * velocity_of_link.x *
                        kBoxDragCoefficient * size_box.y * size_box.z,
                        0.5 * kAirDensity * -sgn(velocity_of_link.y) * velocity_of_link.y * velocity_of_link.y *
                        kBoxDragCoefficient * size_box.z * size_box.x,
                        0.5 * kAirDensity * -sgn(velocity_of_link.z) * velocity_of_link.z * velocity_of_link.z *
                        kBoxDragCoefficient * size_box.x * size_box.y);


                link->AddRelativeForce(drag_force);


                auto angular_vel = link->GetRelativeAngularVel();
                auto link_collision_box = link->GetCollisionBoundingBox().GetSize();


                ignition::math::Vector3d drag_torque(
                        ( pow(angular_vel.x, 2.0) * kAirDensity * kBoxDragCoefficient
                          *link_collision_box.x * (sgn(drag_force.Y())*pow(link_collision_box.y,4.0)  - sgn(drag_force.Z())* pow(link_collision_box.z,4.0)) ) /128.0 ,

                        (pow(angular_vel.y, 2.0) * kAirDensity * kBoxDragCoefficient
                         *link_collision_box.y * (sgn(drag_force.Z())*pow(link_collision_box.z,4.0)  - sgn(drag_force.X())*pow(link_collision_box.x,4.0)) ) /128.0 ,

                        (pow(angular_vel.z, 2.0) * kAirDensity * kBoxDragCoefficient
                         *link_collision_box.z * (sgn(drag_force.X())*pow(link_collision_box.x,4.0)  - sgn(drag_force.Y())*pow(link_collision_box.y,4.0)) ) /128.0 );


                link->AddRelativeTorque(drag_torque);


            }

        }

        // Pointer to the model
    private:
        physics::ModelPtr model;


        // Pointer to the update event connection
    private:
        double m_angular_drag_coeff;
        event::ConnectionPtr updateConnection;
        sdf::ElementPtr my_sdf;
        double kAirDensity = 1.24;

        template<typename T>
        int sgn(T val) {
            return (T(0) < val) - (val < T(0));
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(AirResistanceBox)
}
