#include <chrono>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <unordered_map>
#include <X11/Xlib.h>

#include "rclcpp/rclcpp.hpp"

// #include "bb_interfaces/msg/message.hpp"
// #include "bb_interfaces/srv/change_message.hpp"

#include "bb_interfaces/msg/stats.hpp"
#include "bb_interfaces/msg/s_track_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

    ImVec2 getLast(){
        if (Data.size() == 0)
            return ImVec2(0,0);
        if (Offset == 0)
            return Data[Data.size() - 1];

        return Data[Offset-1];
    }

    ImVec2 getSecondLast(){
        if (Data.size() == 0)
            return ImVec2(0,0);
        if (Data.size() == 1)
            return getLast();
        if (Offset == 0)
            return Data[Data.size() - 2];
        if (Offset == 1)
            return Data[Data.size() - 1];

        return Data[Offset-2];
    }

    ImVec2 getNegativeIndex(int i){
        if(i>=0 || Data.size() <= -i)
            return ImVec2(0,0);
        int difference = Offset + i;
        if (difference < 0)
            return Data[Data.size() + difference];

        return Data[difference];
    }
};
}

/* A node that shows a GUI to visualize statistics*/

class BBVisualizer : public rclcpp::Node
{
  public:
    BBVisualizer();

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
    static const ImGuiTableSortSpecs* s_current_sort_specs;

  private:

    // Make the UI compact because there are so many fields
    static void PushStyleCompact();
    static void PopStyleCompact();

    void update_stats(std::shared_ptr<bb_interfaces::msg::Stats> stats_message);
    void update_stracks(std::shared_ptr<bb_interfaces::msg::STrackArray> strack_message);
    void update_gt(std::shared_ptr<visualization_msgs::msg::Marker> marker);
    void update_track(std::shared_ptr<visualization_msgs::msg::Marker> marker);

    void point_clicked(std::shared_ptr<geometry_msgs::msg::PointStamped> cp_message);

    void update_imgui();

    void visualizeTracks();
    void visualizeStats();
    void visualizeFocus();
    void writeStats();
    void plotHistory();
    void plotPieGraphs(int dim);

    void openDataFile();
    void writeDataToFile();

    void setParameters();
    void setROSParameter(std::string node_name, std::string paramName, bool value);


  private:

    // Set to true to enable the demo window to take inspiration for useful ImGui visualizations
    bool demo = false;

    std::string _data_file = "stats.txt";
    bool _saved_data = false;
    char _note_to_add[256];

    // rclcpp::TimerBase::SharedPtr timer_;

    // rclcpp::Publisher<bb_interfaces::msg::Message>::SharedPtr publisher_;
    // rclcpp::Service<bb_interfaces::srv::ChangeMessage>::SharedPtr srv_;

    // %%%%%%%%% Focus
    bool _focus_activated = false;
    std::shared_ptr<visualization_msgs::msg::Marker> _focus_gt;
    std::shared_ptr<visualization_msgs::msg::Marker> _focus_track;

    // %%%%%%%%% STrack
    bb_interfaces::msg::STrackArray _last_track_msg;

    // %%%%%%%%% Stats
    int _tot_false_positive, _tot_true_positive, _tot_missed, _tot_objects_to_detect;
    int _objects_to_detect, _false_positive, _true_positive, _missed;
    double _detA, _locA, _MOTP, _tot_detA, _tot_locA, _tot_MOTP, _MOTA;
    std::map<std::string, std::string> _statistics_map;

    //-----------------

    rclcpp::TimerBase::SharedPtr imgui_timer_;

    // %%%%%%%%% Subscribers
    rclcpp::Subscription<bb_interfaces::msg::Stats>::SharedPtr _stats_sub;
    rclcpp::Subscription<bb_interfaces::msg::STrackArray>::SharedPtr _strack_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _clicked_point_sub;

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr _focus_track_sub;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr _focus_gt_sub;
};