#define _USE_MATH_DEFINES

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


#include <map>
#include <memory>
#include <sstream>

#include <cmath>

#include <boost/regex.hpp>
#include <boost/algorithm/string_regex.hpp>
#include <vector>


namespace gazebo {

    constexpr double kAirDensity = 1.24;


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

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&AirResistance::OnUpdate, this));
        }

        // Called by the world update start event
    public:
        void OnUpdate() {

            for (auto &link : this->model->GetLinks()) {

                if (cacheLinksInfo.find(link->GetName()) != cacheLinksInfo.end()) {
                    newLink(link);
                }

                auto velocity_of_link = link->GetRelativeLinearVel();
                auto cachLink = cacheLinksInfo[link->GetName()];


                link->AddRelativeForce(
                        {-sgn(velocity_of_link.x) * velocity_of_link.x * velocity_of_link.x * cachLink.x,
                         -sgn(velocity_of_link.y) * velocity_of_link.y * velocity_of_link.y * cachLink.y,
                         -sgn(velocity_of_link.z) * velocity_of_link.z * velocity_of_link.z * cachLink.z
                        });


            }


        }


        double parseCylinderSize(std::string &line) {
            std::size_t pos = line.find(">");
            line = line.substr(pos + 1);
            pos = line.find("/");
            line = line.substr(0, pos - 1);
            std::string::size_type sz;

            return std::stod(line, &sz);
        }

        void parseBoxSizes(std::string line, double &x, double &y, double &z) {
            std::size_t pos = line.find(">");
            line = line.substr(pos + 1);
            pos = line.find("/");
            line = line.substr(0, pos - 1);
            std::vector<std::string> words;
            boost::algorithm::split_regex(words, line, boost::regex("\\s+"), boost::match_default);

            std::string::size_type sz;     // alias of size_t


            x = std::stod(words[0], &sz);
            y = std::stod(words[1], &sz);
            z = std::stod(words[2], &sz);
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

            auto my_prefix = "";

            std::string geometry_ = link->GetSDF()->GetElement("visual")->GetElement("geometry")->ToString(
                    my_prefix);
            std::istringstream f(geometry_);
            std::string line;
            std::getline(f, line); //this is <geometry> line  doesn't care

            std::getline(f, line);

            if (line.find("<box") != std::string::npos) {
                std::getline(f, line);
                double x, y, z;
                parseBoxSizes(line, x, y, z);

                cacheLinksInfo[link->GetName()] = gazebo::math::Vector3(
                        0.5 * kAirDensity * dragCoefficients["box"] * box_area(y, z),
                        0.5 * kAirDensity * dragCoefficients["box"] * box_area(z, x),
                        0.5 * kAirDensity * dragCoefficients["box"] * box_area(x, y));


            } else if (line.find("<cyl") != std::string::npos) {
                std::getline(f, line);
                double length = parseCylinderSize(line);
                std::getline(f, line);
                double radius = parseCylinderSize(line);


                double cyl_mantle =
                        0.5 * kAirDensity * dragCoefficients["cylinder mantle"] * cylinder_mantle_area(length, radius);
                double cyl_base = 0.5 * kAirDensity * dragCoefficients["cylinder base"] * cylinder_base_area(radius);

                cacheLinksInfo[link->GetName()] = gazebo::math::Vector3(cyl_mantle, cyl_mantle, cyl_base);


            } else if (line.find("<sph") != std::string::npos) {

                std::getline(f, line);
                double radius = parseCylinderSize(line);
                double cacheCoeff = 0.5 * kAirDensity * dragCoefficients["sphere"] * sphere_area(radius);
                cacheLinksInfo[link->GetName()] = gazebo::math::Vector3(cacheCoeff, cacheCoeff, cacheCoeff);

            } else {
                auto coll_box = link->GetCollisionBoundingBox().GetSize();

                cacheLinksInfo[link->GetName()] = gazebo::math::Vector3(
                        0.5 * kAirDensity * dragCoefficients["box"] * box_area(coll_box.y, coll_box.z),
                        0.5 * kAirDensity * dragCoefficients["box"] * box_area(coll_box.z, coll_box.x),
                        0.5 * kAirDensity * dragCoefficients["box"] * box_area(coll_box.x, coll_box.y));
            }


        }

        // Pointer to the model
    private:
        physics::ModelPtr model;
        std::map<std::string, gazebo::math::Vector3> cacheLinksInfo;

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
    GZ_REGISTER_MODEL_PLUGIN(AirResistance)
}
