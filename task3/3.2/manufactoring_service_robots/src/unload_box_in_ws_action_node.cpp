#include <memory>
#include <algorithm>
#include<vector>
#include<string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class UnloadBoxInWs : public plansys2::ActionExecutorClient
{
public:
  UnloadBoxInWs()
  : plansys2::ActionExecutorClient("unloadboxinws", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  { std :: vector <std :: string > arguments = get_arguments () ;
    if (progress_ < 1.0) {
      progress_ += 0.2;
      send_feedback(progress_, "Robot "+arguments [0]+ " is emptying "+ 
            arguments [1]+ " in "+ arguments [2] + ", the box contained some "+
            arguments [3]+ " carring a "+
            arguments [4]);
    } else {
      finish(true, progress_, "Robot "+arguments [0]+ " is emptying "+ 
            arguments [1]+ " in "+ arguments [2] + ", the box contained some "+
            arguments [3]+ " carring a "+
            arguments [4]);

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Robot "+arguments [0]+ " is emptying "+ 
            arguments [1]+ " in "+ arguments [2] + ", the box contained some "+
            arguments [3]+ " carring a "+
            arguments [4]+" . . . [ " << std::min(100.0, progress_ * 100.0) << "% ]  " <<
            std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnloadBoxInWs>();

  node->set_parameter(rclcpp::Parameter("action_name", "UNLOADBOXINWS"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
