#include <chrono>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"

// #include "bb_interfaces/msg/message.hpp"
// #include "bb_interfaces/srv/change_message.hpp"

#include "bb_interfaces/msg/stats.hpp"


#include "imgui_framework.hpp"
#include "imgui.h"
#include "implot.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace ImPlot{
// utility structure for realtime plot
struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};
}

/* A node that shows a GUI to visualize statistics*/

class BBVisualizer : public rclcpp::Node
{
  public:
    BBVisualizer()
    : Node("bb_visualizer")
    {
      // publisher_ = this->create_publisher<bb_interfaces::msg::Message>("topic", 10);

      // timer_ = this->create_wall_timer(
      // 500ms, std::bind(&BBVisualizer::timer_callback, this));

      ImGUI_f::init(1280, 720, "bb_visualizer");
      ImGUI_f::uploadFonts();

      imgui_timer_ = this->create_wall_timer(
      std::chrono::seconds(0), std::bind(&BBVisualizer::update_imgui, this));

      // srv_ = this->create_service<bb_interfaces::srv::ChangeMessage>("change_message",  std::bind(&BBVisualizer::change_message, this, _1, _2)); 
      _stats_sub = this->create_subscription<bb_interfaces::msg::Stats>(
      "benchmark/stats", 10, std::bind(&BBVisualizer::update_stats, this, _1));

      
    }

    // void change_message(const std::shared_ptr<bb_interfaces::srv::ChangeMessage::Request> request,
    //       std::shared_ptr<bb_interfaces::srv::ChangeMessage::Response>       response) 
    // {
    //   strcpy(this->message_, request->message_to_display.c_str());  
    //   printf(message_);                                   
    //   RCLCPP_INFO(this->get_logger(), "Incoming request\nmessage_to_display: %s",
    //                 request->message_to_display);
    //   spawnWord_ = true;
    //   spawnTimer_ = 0.0f;
    // }

  private:
    // void timer_callback()
    // {
    //   auto message = bb_interfaces::msg::Message();
    //   message.mess = message_ + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.mess.c_str());
    //   publisher_->publish(message);

    // }

    void update_stats(std::shared_ptr<bb_interfaces::msg::Stats> stats_message){
      _statistics_map = {
        {"Detection Accuracy",          std::to_string(stats_message->det_a)},
        {"Localization Accuracy",       std::to_string(stats_message->loc_a)},
        {"MOTP",                        std::to_string(stats_message->motp)},
        {"True Positive",               std::to_string(stats_message->true_positive)},
        {"False Positive",              std::to_string(stats_message->false_positive)},
        {"Missed Objects",              std::to_string(stats_message->missed)},
        {"Objects to Detect",           std::to_string(stats_message->objects_to_detect)},
        {"TOTAL Detection Accuracy",          std::to_string(stats_message->tot_det_a)},
        {"TOTAL Localization Accuracy",       std::to_string(stats_message->tot_loc_a)},
        {"TOTAL MOTP",                        std::to_string(stats_message->tot_motp)},
        {"TOTAL MOTA",                        std::to_string(stats_message->mota)},
        {"TOTAL True Positive",               std::to_string(stats_message->tot_true_positive)},
        {"TOTAL False Positive",              std::to_string(stats_message->tot_false_positive)},
        {"TOTAL Missed Objects",              std::to_string(stats_message->tot_missed)},
        {"TOTAL Association Mismatch",        std::to_string(stats_message->tot_ass_mismatch)},
        {"TOTAL Objects to Detect",           std::to_string(stats_message->tot_objects_to_detect)}
      };

      _detA =                  stats_message->det_a;           
      _locA =                  stats_message->loc_a;           
      _MOTP =                  stats_message->motp;            
      _true_positive =         stats_message->true_positive;   
      _false_positive =        stats_message->false_positive;   
      _missed =                stats_message->missed;          
      _objects_to_detect =     stats_message->objects_to_detect;   
      _tot_detA =              stats_message->tot_det_a;           
      _tot_locA =              stats_message->tot_loc_a;       
      _tot_MOTP =              stats_message->tot_motp;          
      _tot_true_positive =     stats_message->tot_true_positive; 
      _tot_false_positive =    stats_message->tot_false_positive;
      _tot_missed =            stats_message->tot_missed;          
      _tot_objects_to_detect = stats_message->tot_objects_to_detect;
      _MOTA =                  stats_message->mota;

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


      ImGuiIO& io = ImGui::GetIO();
      ImGuiWindowFlags window_flags = 0;
      window_flags |= ImGuiWindowFlags_NoDocking;

      visualizeStats();
      ImGui::Begin("Framerate", NULL, window_flags);


      // ImGui::InputText("message to publish", message_, IM_ARRAYSIZE(message_));

      // if (spawnWord_ && spawnTimer_ < spawnDuration_) {
      //   ImGui::Text("received service call");
      //   spawnTimer_ += ImGui::GetIO().DeltaTime;
      // }

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
      ImGui::End();

      if(ImGui::Button("See demo")){
        demo = !demo;
      }
      if(demo){
        ImGui::ShowDemoWindow(&demo);
      }

      ImGUI_f::render();

    }

    void visualizeStats(){
      ImGuiWindowFlags window_flags = 0;
      window_flags |= ImGuiWindowFlags_NoDocking;
      ImGui::Begin("Statistics", NULL, window_flags);

      if (ImGui::CollapsingHeader("Numeric")){
        writeStats();
      }

      if (ImGui::CollapsingHeader("Pie Graph")){
        plotPieGraphs(200);
      }

      if (ImGui::CollapsingHeader("History")){
        plotHistory();
      }

      
      if (ImGui::Button("Save data to file")){
        writeDataToFile();
      }
      ImGui::SameLine();
      ImGui::InputTextWithHint("##Input", "Notes...", _note_to_add, IM_ARRAYSIZE(_note_to_add));
      
      if (_saved_data){
        ImGui::Text("Saved");
      }

      if (ImGui::Button("Show previously saved data")){
        openDataFile();
      }

      ImGui::End();
    }

    void openDataFile(){
      // Use the 'xdg-open' command to open the file with the default text editor
      std::string openCommand = "xdg-open " + std::string(_data_file);

      // Execute the command
      int result = system(openCommand.c_str());

      if (result != 0) {
        std::cerr << "Failed to open the file." << std::endl;
      }
    }

    void writeDataToFile(){
      // Open the text file in append mode
      std::ofstream outputFile(_data_file, std::ios::app);

      if (!outputFile.is_open()) {
          std::cerr << "Failed to open the file." << std::endl;
          return;
      }

      // Get the current date and time
      auto currentTime = std::chrono::system_clock::now();
      std::time_t time = std::chrono::system_clock::to_time_t(currentTime);

      // Create a string to hold the formatted date and time
      std::string dateTime = std::ctime(&time);
      // Write the date and time to the file
      outputFile << "\nDate and Time: " << dateTime << "\n" << "Note: " << _note_to_add << "\n";

      // Write the map data to the file in a JSON-like format
      outputFile << "{\n";
      for (const auto& pair : _statistics_map) {
        if(pair.first.find("TOTAL") != std::string::npos) // There is TOTAL in the name
          outputFile << "  \"" << pair.first << "\": " << pair.second << ",\n";
      }
      outputFile << "}\n";

      // Close the file
      outputFile.close();

      _saved_data = true;
    }

    void plotHistory(){
      static ImPlot::ScrollingBuffer sdata1, sdata2, sdata3;
      static float t = 0;
      t += ImGui::GetIO().DeltaTime;
      sdata1.AddPoint(t, _detA);
      sdata2.AddPoint(t, _locA);
      sdata3.AddPoint(t, _MOTA);

      static float history = 10.0f;
      ImGui::SliderFloat("History",&history,1,30,"%.1f s");

      static ImPlotAxisFlags flags = ImPlotAxisFlags_None;

      if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,300))) {
          ImPlot::SetupAxes("Time", "Value", flags, flags);
          ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
          ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
          ImPlot::PlotLine("Detection Accuracy", &sdata1.Data[0].x, &sdata1.Data[0].y, sdata1.Data.size(), 0, sdata1.Offset, 2*sizeof(float));
          ImPlot::PlotLine("Localization Accuracy", &sdata2.Data[0].x, &sdata2.Data[0].y, sdata2.Data.size(), 0, sdata2.Offset, 2*sizeof(float));
          ImPlot::PlotLine("MOTA", &sdata3.Data[0].x, &sdata3.Data[0].y, sdata3.Data.size(), 0, sdata3.Offset, 2*sizeof(float));
          ImPlot::EndPlot();
      }
    }

    void writeStats(){
      if (ImGui::BeginTable("Statistics", 2))
      {
          for (auto pair : _statistics_map)
          {
              ImGui::TableNextRow();
              ImGui::TableSetColumnIndex(0);
              ImGui::Text(pair.first.c_str());
              ImGui::TableSetColumnIndex(1);
              ImGui::Text(pair.second.c_str());
          }
          ImGui::EndTable();
      }
    }

    void plotPieGraphs(int dim){
      static const char* labels1[]    = {"True Positive","False Positive"};
      int data1[]                     = {_true_positive,  _false_positive};
      static ImPlotPieChartFlags flags = 0;
      ImGui::SetNextItemWidth(dim);

      if (ImPlot::BeginPlot("##Pie1", ImVec2(dim,dim), ImPlotFlags_Equal | ImPlotFlags_NoMouseText)) {
          ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
          ImPlot::SetupAxesLimits(0, 1, 0, 1);
          ImPlot::PlotPieChart(labels1, data1, 2, 0.5, 0.5, 0.4, "%.0f", 90, flags);
          ImPlot::EndPlot();
      }

      ImGui::SameLine();

      static const char* labels2[]    = {"TOT True Positive","TOT False Positive"};
      int data2[]                     = {_tot_true_positive,  _tot_false_positive};
      ImGui::SetNextItemWidth(dim);

      if (ImPlot::BeginPlot("##Pie2", ImVec2(dim,dim), ImPlotFlags_Equal | ImPlotFlags_NoMouseText)) {
          ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
          ImPlot::SetupAxesLimits(0, 1, 0, 1);
          ImPlot::PlotPieChart(labels2, data2, 2, 0.5, 0.5, 0.4, "%.0f", 90, flags);
          ImPlot::EndPlot();
      }



      static const char* labels3[]    = {"Find","Missed"};
      int data3[]                     = {_true_positive,  _missed};
      ImGui::SetNextItemWidth(dim);

      if (ImPlot::BeginPlot("##Pie3", ImVec2(dim,dim), ImPlotFlags_Equal | ImPlotFlags_NoMouseText)) {
          ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
          ImPlot::SetupAxesLimits(0, 1, 0, 1);
          ImPlot::PlotPieChart(labels3, data3, 2, 0.5, 0.5, 0.4, "%.0f", 90, flags);
          ImPlot::EndPlot();
      }

      ImGui::SameLine();

      static const char* labels4[]    = {"TOT Find","TOT Missed"};
      int data4[]                     = {_tot_true_positive,  _tot_missed};
      ImGui::SetNextItemWidth(dim);

      if (ImPlot::BeginPlot("##Pie4", ImVec2(dim,dim), ImPlotFlags_Equal | ImPlotFlags_NoMouseText)) {
          ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoDecorations);
          ImPlot::SetupAxesLimits(0, 1, 0, 1);
          ImPlot::PlotPieChart(labels4, data4, 2, 0.5, 0.5, 0.4, "%.0f", 90, flags);
          ImPlot::EndPlot();
      }
    }


  private:

    // Set to true to enable the demo window to take inspiration for useful ImGui visualizations
    bool demo = false;

    std::string _data_file = "stats.txt";
    bool _saved_data = false;
    char _note_to_add[256];

    // bool spawnWord_ = false;
    // float spawnTimer_ = 0.0f;
    // const float spawnDuration_ = 3.0f; // 3 seconds

    // size_t count_;
    // char message_[256] = "Hello, world!";

    // rclcpp::TimerBase::SharedPtr timer_;

    // rclcpp::Publisher<bb_interfaces::msg::Message>::SharedPtr publisher_;
    // rclcpp::Service<bb_interfaces::srv::ChangeMessage>::SharedPtr srv_;


    // %%%%%%%%% Stats
    int _tot_false_positive, _tot_true_positive, _tot_missed, _tot_objects_to_detect;
    int _objects_to_detect, _false_positive, _true_positive, _missed;
    double _detA, _locA, _MOTP, _tot_detA, _tot_locA, _tot_MOTP, _MOTA;
    std::map<std::string, std::string> _statistics_map;

    rclcpp::TimerBase::SharedPtr imgui_timer_;


    rclcpp::Subscription<bb_interfaces::msg::Stats>::SharedPtr _stats_sub;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BBVisualizer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}