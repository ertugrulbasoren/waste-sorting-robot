#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <string>
#include <functional>

namespace gazebo
{
class ConveyorMotionPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();

    belt_center_ = ignition::math::Vector3d(0.0, 0.0, 0.18);
    belt_size_   = ignition::math::Vector3d(2.4, 0.6, 0.12);
    belt_speed_  = 0.22;
    top_margin_  = 0.20;

    if (sdf->HasElement("belt_center"))
      belt_center_ = sdf->Get<ignition::math::Vector3d>("belt_center");

    if (sdf->HasElement("belt_size"))
      belt_size_ = sdf->Get<ignition::math::Vector3d>("belt_size");

    if (sdf->HasElement("belt_speed"))
      belt_speed_ = sdf->Get<double>("belt_speed");

    if (sdf->HasElement("top_margin"))
      top_margin_ = sdf->Get<double>("top_margin");

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ConveyorMotionPlugin::OnUpdate, this));

    gzmsg << "[ConveyorMotionPlugin] Loaded. speed=" << belt_speed_ << "\n";
  }

private:
  bool Ignore(const std::string &name) const
  {
    if (name == model_->GetName()) return true;
    if (name.find("trash_bin") != std::string::npos) return true;
    if (name.find("top_camera") != std::string::npos) return true;
    if (name.find("ground_plane") != std::string::npos) return true;
    return false;
  }

  void OnUpdate()
  {
    const double x_min = belt_center_.X() - belt_size_.X() * 0.5;
    const double x_max = belt_center_.X() + belt_size_.X() * 0.5;
    const double y_min = belt_center_.Y() - belt_size_.Y() * 0.5;
    const double y_max = belt_center_.Y() + belt_size_.Y() * 0.5;
    const double z_min = belt_center_.Z() - 0.06;
    const double z_max = belt_center_.Z() + top_margin_;

    for (auto const &m : world_->Models())
    {
      if (!m) continue;
      if (m->IsStatic()) continue;
      if (Ignore(m->GetName())) continue;

      ignition::math::Vector3d pos = m->WorldPose().Pos();
      if (pos.X() < x_min || pos.X() > x_max) continue;
      if (pos.Y() < y_min || pos.Y() > y_max) continue;
      if (pos.Z() < z_min || pos.Z() > z_max) continue;

      ignition::math::Vector3d vel = m->WorldLinearVel();
      ignition::math::Vector3d ang = m->WorldAngularVel();

      vel.X(belt_speed_);
      vel.Y(vel.Y() * 0.9);
      if (vel.Z() < -0.1) vel.Z(-0.1);

      ang.X(ang.X() * 0.8);
      ang.Y(ang.Y() * 0.8);
      ang.Z(ang.Z() * 0.8);

      m->SetLinearVel(vel);
      m->SetAngularVel(ang);
    }
  }

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
  ignition::math::Vector3d belt_center_;
  ignition::math::Vector3d belt_size_;
  double belt_speed_;
  double top_margin_;
};

GZ_REGISTER_MODEL_PLUGIN(ConveyorMotionPlugin)
}
