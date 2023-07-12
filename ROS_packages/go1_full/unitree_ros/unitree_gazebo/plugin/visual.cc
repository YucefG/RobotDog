#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Visual.hh>

namespace gazebo
{
  class VisualPlugin : public VisualPlugin
  {
    public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  
      // Set the visual size and color
      const double size = 0.1;
      const common::Color red(1.0, 0.0, 0.0, 1.0);

      // Create a sphere visual
      rendering::SpherePtr sphere = _parent->CreateSphere();
      sphere->SetRadius(size);
      sphere->SetAmbient(red);
      sphere->SetDiffuse(red);

      // Set the visual pose to be the origin of the world
      ignition::math::Pose3d pose(0, 0, 0, 0, 0, 0);
      sphere->SetPose(pose);
    }
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_VISUAL_PLUGIN(VisualPlugin)
}