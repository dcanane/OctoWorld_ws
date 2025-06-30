#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>

class OctomapSaver : public rclcpp::Node
{
public:
  OctomapSaver() : Node("octomap_bt_saver")
  {
    this->declare_parameter<std::string>("output_path", "/tmp/mapa.bt");

    sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/rtabmap/octomap_binary", 10,
      std::bind(&OctomapSaver::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "A ouvir em /rtabmap/octomap_binary...");
  }

private:
  void callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::string output_path;
    this->get_parameter("output_path", output_path);

    std::shared_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*msg));
    if (!tree) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao converter mensagem Octomap!");
      return;
    }

    auto* color_tree = dynamic_cast<octomap::ColorOcTree*>(tree.get());
    if (!color_tree) {
      RCLCPP_ERROR(this->get_logger(), "A mensagem não contém um ColorOcTree válido.");
      return;
    }

    if (color_tree->writeBinary(output_path)) {
      RCLCPP_INFO(this->get_logger(), "Mapa guardado em: %s", output_path.c_str());
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Falha ao guardar o ficheiro .bt.");
    }
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
};
  
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapSaver>());
  rclcpp::shutdown();
  return 0;
}
