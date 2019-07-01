#define _USE_MATH_DEFINES

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <cmath>
#include <vector>


namespace gazebo {


    static std::map<std::string, double> dragCoefficients = {{"box",                    1.05},
                                                             {"cylinder mantle",        1.2},
                                                             {"cylinder base",          1.12},
                                                             {"sphere",                 0.45},
                                                             {"square cylinder mantle", 2.0}};


    class AirResistance : public ModelPlugin {
    public:

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {


            // Store the pointer to the model
            this->model = _parent;
            this->my_sdf = _sdf;


            if (my_sdf->HasElement("my_air_density"))
                kAirDensity = std::stod(my_sdf->GetElement("my_air_density")->GetValue()->GetAsString());

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&AirResistance::OnUpdate, this));
            m_angular_drag_coeff = 5.0;

        }

        // Called by the world update start event
    public:
        void OnUpdate() {

            for (auto &link : this->model->GetLinks()) {

                if (cacheLinksInfo.find(link->GetName()) == cacheLinksInfo.end()) {
                    newLink(link);
                }


                auto velocity_of_link = link->GetRelativeLinearVel();
                auto cacheLink = cacheLinksInfo[link->GetName()];

                ignition::math::Vector3d drag_force(
                        -sgn(velocity_of_link.x) * velocity_of_link.x * velocity_of_link.x * cacheLink.X(),
                        -sgn(velocity_of_link.y) * velocity_of_link.y * velocity_of_link.y * cacheLink.Y(),
                        -sgn(velocity_of_link.z) * velocity_of_link.z * velocity_of_link.z * cacheLink.Z());


                link->AddRelativeForce(drag_force);



               auto angular_vel = link->GetRelativeAngularVel();
                auto cacheLinkAng = cacheLinksInfoAngular[link->GetName()];


                ignition::math::Vector3d drag_torque(
                         -sgn(angular_vel.x) * pow(angular_vel.x, 2.0) *  cacheLinkAng.X(),

                         -sgn(angular_vel.y) * pow(angular_vel.y, 2.0) *  cacheLinkAng.Y(),

                         -sgn(angular_vel.z) * pow(angular_vel.z, 2.0) *  cacheLinkAng.Z() ) ;


                link->AddRelativeTorque(drag_torque);


            }


        }


        inline double cylinder_mantle_area(double height, double radius) {
            return 2.0 * radius * height;
        }

        inline double cylinder_base_area(double radius) {
            return (pow(2.0 * radius, 2.0) * M_PI) / 4.0;
        }

        inline double sphere_area(double radius) {
            return (pow(2.0 * radius, 2.0) * M_PI) / 4.0;
        }

        inline double box_area(double sideA, double sideB) {
            return sideA * sideB;
        }

        void newLink(const boost::shared_ptr<gazebo::physics::Link> &link) {

            auto coll_box = link->GetCollisionBoundingBox().GetSize();

            cacheLinksInfo[link->GetName()] = ignition::math::Vector3d(
                    0.5 * kAirDensity * dragCoefficients["box"] * box_area(coll_box.y, coll_box.z),
                    0.5 * kAirDensity * dragCoefficients["box"] * box_area(coll_box.z, coll_box.x),
                    0.5 * kAirDensity * dragCoefficients["box"] * box_area(coll_box.x, coll_box.y));


            cacheLinksInfoAngular[link->GetName()] = ignition::math::Vector3d(
                     kAirDensity * dragCoefficients["box"]  *  (1/8.0) * coll_box.x *  (pow(coll_box.z,3.0) +  pow(coll_box.y,3.0) ),

                     kAirDensity * dragCoefficients["box"]  *   (1/8.0) * coll_box.y *  (pow(coll_box.x,3.0) +  pow(coll_box.z,3.0) ) ,

                     kAirDensity * dragCoefficients["box"]  *  (1/8.0) * coll_box.z *  (pow(coll_box.y,3.0) +  pow(coll_box.x,3.0) ) ) ;


            if (link->GetSDF()->HasElement("visual")) {
                if (link->GetSDF()->GetElement("visual")->HasElement("geometry")) {
                    if (link->GetSDF()->GetElement("visual")->GetElement("geometry")->HasElement("cylinder")) {
                        double radius = std::stod(
                                link->GetSDF()->GetElement("visual")->GetElement("geometry")->GetElement(
                                        "cylinder")->GetElement("radius")->GetValue()->GetAsString());
                        double length = std::stod(
                                link->GetSDF()->GetElement("visual")->GetElement("geometry")->GetElement(
                                        "cylinder")->GetElement("length")->GetValue()->GetAsString());

                        double cyl_mantle =
                                0.5 * kAirDensity * dragCoefficients["cylinder mantle"] *
                                cylinder_mantle_area(length, radius);
                        double cyl_base =
                                0.5 * kAirDensity * dragCoefficients["cylinder base"] * cylinder_base_area(radius);

                        cacheLinksInfo[link->GetName()] = ignition::math::Vector3d(cyl_mantle, cyl_mantle, cyl_base);

                        double cyl_mantle_ang_z = M_PI * dragCoefficients["cylinder mantle"] * kAirDensity * pow(radius ,4.0) * length;

                        double cyl_base_ang_z = M_PI *  kAirDensity * dragCoefficients["cylinder base"] * (1.0/5.0) * pow(radius ,5.0);


                        double cyl_mantle_ang_xy = M_PI * dragCoefficients["cylinder mantle"] * kAirDensity * radius  * pow(length, 3.0) *(1.0/24.0);

                        double cyl_base_ang_xy = M_PI *  kAirDensity * dragCoefficients["cylinder base"] * (1.0/8.0) * pow(radius ,2.0) * pow(length ,2.0);

                        cacheLinksInfoAngular[link->GetName()] = ignition::math::Vector3d(cyl_mantle_ang_xy + 2.0*cyl_base_ang_xy, cyl_mantle_ang_xy + 2.0*cyl_base_ang_xy, cyl_mantle_ang_z + 2.0*cyl_base_ang_z);



                    } else if (link->GetSDF()->GetElement("visual")->GetElement("geometry")->HasElement("box")) {
                        std::vector<std::string> sizes = split(
                                link->GetSDF()->GetElement("visual")->GetElement("geometry")->GetElement(
                                        "box")->GetElement(
                                        "size")->GetValue()->GetAsString());
                        double x = std::stod(sizes[0]);
                        double y = std::stod(sizes[1]);
                        double z = std::stod(sizes[2]);

                        cacheLinksInfo[link->GetName()] = ignition::math::Vector3d(
                                0.5 * kAirDensity * dragCoefficients["box"] * box_area(y, z),
                                0.5 * kAirDensity * dragCoefficients["box"] * box_area(z, x),
                                0.5 * kAirDensity * dragCoefficients["box"] * box_area(x, y));


                        cacheLinksInfoAngular[link->GetName()] = ignition::math::Vector3d(
                                kAirDensity * dragCoefficients["box"]  *  (1/8.0) * x *  (pow(z,3.0) +  pow(y,3.0) ),

                                kAirDensity * dragCoefficients["box"]  *   (1/8.0) * y *  (pow(x,3.0) +  pow(z,3.0) ) ,

                                kAirDensity * dragCoefficients["box"]  *  (1/8.0) * z *  (pow(y,3.0) +  pow(x,3.0) ) ) ;

                    } else if (link->GetSDF()->GetElement("visual")->GetElement("geometry")->HasElement("sphere")) {
                        double radius = std::stod(
                                link->GetSDF()->GetElement("visual")->GetElement("geometry")->GetElement(
                                        "sphere")->GetElement(
                                        "radius")->GetValue()->GetAsString());

                        double cacheCoeff = 0.5 * kAirDensity * dragCoefficients["sphere"] * sphere_area(radius);
                        cacheLinksInfo[link->GetName()] = ignition::math::Vector3d(cacheCoeff, cacheCoeff, cacheCoeff);

                        double cacheCoeffAng =  0.5 * kAirDensity * dragCoefficients["sphere"] *(8.0/3.0) * M_PI * pow(radius,4)  ;

                        cacheLinksInfoAngular[link->GetName()] = ignition::math::Vector3d(cacheCoeffAng, cacheCoeffAng, cacheCoeffAng);


                    }
                }
            }


        }

        // Pointer to the model
    private:
        physics::ModelPtr model;
        std::map<std::string, ignition::math::Vector3d> cacheLinksInfo;
        std::map<std::string, ignition::math::Vector3d> cacheLinksInfoAngular;

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

        std::vector<std::string> split(std::string const &input) {
            std::istringstream buffer(input);
            std::vector<std::string> ret{std::istream_iterator<std::string>(buffer),
                                         std::istream_iterator<std::string>()};
            return ret;
        }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(AirResistance)
}
