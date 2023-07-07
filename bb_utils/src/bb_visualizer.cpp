#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bb_interfaces/msg/message.hpp"
#include "bb_interfaces/srv/change_message.hpp"

#include "imgui_framework.hpp"
#include "imgui.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

/* A node that shows a GUI */

class BBVisualizer : public rclcpp::Node
{
  public:
    BBVisualizer()
    : Node("bb_visualizer"), count_(0)
    {
      publisher_ = this->create_publisher<bb_interfaces::msg::Message>("topic", 10);

      timer_ = this->create_wall_timer(
      500ms, std::bind(&BBVisualizer::timer_callback, this));

      ImGUI_f::init(1280, 720, "bb_visualizer");
      ImGUI_f::uploadFonts();

      imgui_timer_ = this->create_wall_timer(
      std::chrono::seconds(0), std::bind(&BBVisualizer::update_imgui, this));

      srv_ = this->create_service<bb_interfaces::srv::ChangeMessage>("change_message",  std::bind(&BBVisualizer::change_message, this, _1, _2)); 
      
    }

    void change_message(const std::shared_ptr<bb_interfaces::srv::ChangeMessage::Request> request,
          std::shared_ptr<bb_interfaces::srv::ChangeMessage::Response>       response) 
    {
      strcpy(this->message_, request->message_to_display.c_str());  
      printf(message_);                                   
      RCLCPP_INFO(this->get_logger(), "Incoming request\nmessage_to_display: %s",
                    request->message_to_display);
      spawnWord_ = true;
      spawnTimer_ = 0.0f;
    }

  private:
    void timer_callback()
    {
      auto message = bb_interfaces::msg::Message();
      message.mess = message_ + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.mess.c_str());
      publisher_->publish(message);

    }

    void update_imgui(){

      glfwPollEvents();

      if(glfwWindowShouldClose(ImGUI_f::window)){
        ImGUI_f::cleanup();
        rclcpp::shutdown();
      }
      
      ImGUI_f::resizeSwapChain();
      ImGUI_f::newFrame();

      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      // Real ImGui code
      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      static float f = 0.0f;
      static int counter = 0;
      ImGuiIO& io = ImGui::GetIO();
      ImGuiWindowFlags window_flags = 0;
      window_flags |= ImGuiWindowFlags_NoDocking;
      ImGui::Begin("Hello, world!", NULL, window_flags);                          // Create a window called "Hello, world!" and append into it.

      ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)

      ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
      ImGui::ColorEdit3("clear color", (float*)&ImGUI_f::clear_color); // Edit 3 floats representing a color

      if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
          counter++;
      ImGui::SameLine();
      ImGui::Text("counter = %d", counter);
      ImGui::InputText("message to publish", message_, IM_ARRAYSIZE(message_));

      if (spawnWord_ && spawnTimer_ < spawnDuration_) {
        ImGui::Text("received service call");
        spawnTimer_ += ImGui::GetIO().DeltaTime;
      }

      ImGui::ShowDemoWindow(&demo);
      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
      ImGui::End();

      ImGUI_f::render();

    }

    bool demo = true;
    bool spawnWord_ = false;
    float spawnTimer_ = 0.0f;
    const float spawnDuration_ = 3.0f; // 3 seconds


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr imgui_timer_;

    rclcpp::Publisher<bb_interfaces::msg::Message>::SharedPtr publisher_;
    rclcpp::Service<bb_interfaces::srv::ChangeMessage>::SharedPtr srv_;
    size_t count_;
    char message_[256] = "Hello, world!";
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BBVisualizer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}