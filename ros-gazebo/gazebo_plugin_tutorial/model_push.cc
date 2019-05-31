#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

class ModelPush : public ModelPlugin {
 public:
  void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/) {
    // Store the pointer to the model
    model_ = parent;

    // Listen to the update event. This event is broadcast every simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPush::OnUpdate, this));
  }

  // Called by the world update start event
  void OnUpdate() {
    // Apply a small linear velocity to the model.
    model_->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
  }

 private:
  physics::ModelPtr model_;  // Pointer to the model
  event::ConnectionPtr updateConnection_;  // Pointer to the update event connection
};

GZ_REGISTER_MODEL_PLUGIN(ModelPush)  // Register this plugin with the simulator

}  // namespace gazebo
