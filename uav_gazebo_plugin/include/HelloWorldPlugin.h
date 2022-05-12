#include <gazebo/gazebo.hh>

namespace gazebo {
class HelloWorldPlugin : public WorldPlugin {
public:
    HelloWorldPlugin()
        : WorldPlugin()
    {
        printf("Hello World!\n");
    }

public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) { }
};

}