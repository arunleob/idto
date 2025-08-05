#include "examples/example_base.h"
#include <drake/common/find_resource.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(rotate, false,
            "whether to solve the problem where the plate is rotated 45 degrees");
DEFINE_bool(floating, false,
            "whether to solve the problem where the plate is rotated 45 degrees using a floating base");

namespace idto {
namespace examples {
namespace waiter {

using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::Cylinder;
using drake::geometry::ProximityProperties;
using drake::geometry::Rgba;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using Eigen::Vector3d;
using utils::FindIdtoResource;

class WaiterExample : public TrajOptExample {
 public:
  WaiterExample() {
  }

 private:

  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    if (!FLAGS_floating) {
      std::string urdf_file =
          FindIdtoResource("idto/models/waiter_sphere.urdf");
      ModelInstanceIndex sphere = Parser(plant).AddModels(urdf_file)[0];
      plant->set_gravity_enabled(sphere, false);

      urdf_file =
          FindIdtoResource("idto/models/waiter.urdf");
      Parser(plant).AddModels(urdf_file);
    }
    else {
      std::string urdf_file =
          FindIdtoResource("idto/models/waiter_rotate_sphere.urdf");
      ModelInstanceIndex sphere = Parser(plant).AddModels(urdf_file)[0];
      plant->set_gravity_enabled(sphere, false);

      urdf_file =
          FindIdtoResource("idto/models/waiter_floating.urdf");
      Parser(plant).AddModels(urdf_file);
    }

  }
};

}  // namespace waiter
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::waiter::WaiterExample example;
  std::string yaml_file;
  if (FLAGS_floating) {
     yaml_file = "idto/examples/waiter/waiter_floating.yaml";
  } else {
     yaml_file = "idto/examples/waiter/waiter.yaml";
  }
  example.RunExample(yaml_file);

  return 0;
}
