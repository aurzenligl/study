#include <gazebo/gazebo.hh>

namespace gazebo {

class WorldPluginTutorial : public WorldPlugin {
 public:
  WorldPluginTutorial() : WorldPlugin() {
    printf("Hello World!\n");
  }

  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) {}
};

GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)

}  // namespace gazebo
