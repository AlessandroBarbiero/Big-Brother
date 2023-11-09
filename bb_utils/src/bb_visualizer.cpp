#include <bb_utils/bb_visualizer.hpp>

const ImGuiTableSortSpecs* BBVisualizer::s_current_sort_specs = NULL;
// Compare function to be used by sort()
bool CompareWithSortSpecs(const bb_interfaces::msg::STrack& a, const bb_interfaces::msg::STrack& b);

BBVisualizer::BBVisualizer()
: Node("bb_visualizer")
{
  ImGUI_f::init(1280, 720, "bb_visualizer");
  ImGUI_f::uploadFonts();

  imgui_timer_ = this->create_wall_timer(
  std::chrono::seconds(0), std::bind(&BBVisualizer::update_imgui, this));

  // srv_ = this->create_service<bb_interfaces::srv::ChangeMessage>("change_message",  std::bind(&BBVisualizer::change_message, this, _1, _2)); 
  _stats_sub = this->create_subscription<bb_interfaces::msg::Stats>(
  "benchmark/stats", 10, std::bind(&BBVisualizer::update_stats, this, _1));
  _strack_sub = this->create_subscription<bb_interfaces::msg::STrackArray>(
  "bytetrack/active_tracks_explicit", 10, std::bind(&BBVisualizer::update_stracks, this, _1));

}

// Make the UI compact because there are so many fields
void BBVisualizer::PushStyleCompact()
{
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(style.FramePadding.x, (float)(int)(style.FramePadding.y * 0.60f)));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(style.ItemSpacing.x, (float)(int)(style.ItemSpacing.y * 0.60f)));
}

void BBVisualizer::PopStyleCompact()
{
    ImGui::PopStyleVar(2);
}

void BBVisualizer::update_stats(std::shared_ptr<bb_interfaces::msg::Stats> stats_message){
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

void BBVisualizer::update_stracks(std::shared_ptr<bb_interfaces::msg::STrackArray> strack_message){
  _last_track_msg = *strack_message.get();
}

void BBVisualizer::update_imgui(){

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
  visualizeTracks();

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

void BBVisualizer::visualizeTracks(){
  // static float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;
  static float TEXT_BASE_HEIGHT = ImGui::GetTextLineHeightWithSpacing();

  ImGuiWindowFlags window_flags = 0;
  window_flags |= ImGuiWindowFlags_NoDocking;
  ImGui::Begin("Tracks", NULL, window_flags);

  static ImGuiTableFlags flags = // ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY | 
  ImGuiTableFlags_RowBg | 
  ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | 
  ImGuiTableFlags_Hideable | ImGuiTableFlags_Sortable | ImGuiTableFlags_SortMulti;
  static int freeze_cols = 1;
  static int freeze_rows = 1;

  PushStyleCompact();
  ImGui::SetNextItemWidth(ImGui::GetFrameHeight());
  ImGui::DragInt("freeze_cols", &freeze_cols, 0.2f, 0, 9, NULL, ImGuiSliderFlags_NoInput);
  ImGui::SetNextItemWidth(ImGui::GetFrameHeight());
  ImGui::DragInt("freeze_rows", &freeze_rows, 0.2f, 0, 9, NULL, ImGuiSliderFlags_NoInput);
  PopStyleCompact();

  ImGui::Spacing();

  static ImVector<int64_t> selection;
  static ImGuiTextFilter filter;
  filter.Draw("Filter by TrackID");

  ImVec2 outer_size = ImVec2(0.0f, TEXT_BASE_HEIGHT * 8);
  if (ImGui::BeginTable("tracks_table", 13, flags, outer_size))
  {
    ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
    ImGui::TableSetupColumn("TrackID", ImGuiTableColumnFlags_NoHide); // Make the first column not hideable to match our use of TableSetupScrollFreeze()
    ImGui::TableSetupColumn("IsActivated");
    ImGui::TableSetupColumn("State");
    ImGui::TableSetupColumn("Class");
    ImGui::TableSetupColumn("Score (%)");
    ImGui::TableSetupColumn("X (m)");
    ImGui::TableSetupColumn("Y (m)");
    ImGui::TableSetupColumn("Theta (rad)");
    ImGui::TableSetupColumn("Size x (m)");
    ImGui::TableSetupColumn("Size y (m)");
    ImGui::TableSetupColumn("Size z (m)");
    ImGui::TableSetupColumn("V (m/s)");
    ImGui::TableSetupColumn("W (rad/s)");

    // Sort our data
    ImGuiTableSortSpecs* sorts_specs = ImGui::TableGetSortSpecs();
    if (sorts_specs && _last_track_msg.tracks.size() > 1)
    {
        s_current_sort_specs = sorts_specs; // Store in variable accessible by the sort function.
        std::sort(_last_track_msg.tracks.begin(), _last_track_msg.tracks.end(), CompareWithSortSpecs);
    }


    ImGui::TableHeadersRow();

    for (size_t row = 0; row < this->_last_track_msg.tracks.size(); row++)
    {
      bb_interfaces::msg::STrack track = _last_track_msg.tracks[row];
      if (!filter.PassFilter(std::to_string(track.track_id).c_str()))
        continue;

      const bool item_is_selected = selection.contains(track.track_id);
      ImGui::PushID(track.track_id);
      ImGui::TableNextRow();


      ImGui::TableSetColumnIndex(0);   // TrackID
      char label[32];
      sprintf(label, "%ld", track.track_id);
      ImGuiSelectableFlags selectable_flags = ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowItemOverlap;
      if (ImGui::Selectable(label, item_is_selected, selectable_flags))
      {
          if (ImGui::GetIO().KeyCtrl)
          {
              if (item_is_selected)
                  selection.find_erase_unsorted(track.track_id);
              else
                  selection.push_back(track.track_id);
          }
          else
          {
              selection.clear();
              selection.push_back(track.track_id);
          }
      }

      if (ImGui::TableSetColumnIndex(1))   // IsActivated
        ImGui::Text("%s", track.is_activated ? "true" : "false");
      if (ImGui::TableSetColumnIndex(2))   // State
        ImGui::Text("%s", track.state.c_str());
      if (ImGui::TableSetColumnIndex(3))   // Class
        ImGui::Text("%s", track.class_name.c_str());
      if (ImGui::TableSetColumnIndex(4))   // Score
        ImGui::Text("%.2f", track.score);
      if (ImGui::TableSetColumnIndex(5))   // X
        ImGui::Text("%.2f", track.mean[0]);
      if (ImGui::TableSetColumnIndex(6))   // Y
        ImGui::Text("%.2f", track.mean[1]);
      if (ImGui::TableSetColumnIndex(7))   // Theta
        ImGui::Text("%.2f", track.mean[2]);
      if (ImGui::TableSetColumnIndex(8))   // Size x
        ImGui::Text("%.2f", track.mean[3]*track.mean[5]);
      if (ImGui::TableSetColumnIndex(9))   // Size y
        ImGui::Text("%.2f", track.mean[4]*track.mean[5]);
      if (ImGui::TableSetColumnIndex(10))   // Size z
        ImGui::Text("%.2f", track.mean[5]);
      if (ImGui::TableSetColumnIndex(11))   // V
        ImGui::Text("%.2f", track.mean[6]);
      if (ImGui::TableSetColumnIndex(12))   // W
        ImGui::Text("%.2f", track.mean[7]);

      ImGui::PopID();
    }
    ImGui::EndTable();
  }

  ImGui::End();
}

void BBVisualizer::visualizeStats(){
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

void BBVisualizer::openDataFile(){
  // Use the 'xdg-open' command to open the file with the default text editor
  std::string openCommand = "xdg-open " + std::string(_data_file);

  // Execute the command
  int result = system(openCommand.c_str());

  if (result != 0) {
    std::cerr << "Failed to open the file." << std::endl;
  }
}

void BBVisualizer::writeDataToFile(){
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

void BBVisualizer::plotHistory(){
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

void BBVisualizer::writeStats(){
  if (ImGui::BeginTable("Statistics", 2))
  {
      for (auto pair : _statistics_map)
      {
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::Text("%s", pair.first.c_str());
          ImGui::TableSetColumnIndex(1);
          ImGui::Text("%s", pair.second.c_str());
      }
      ImGui::EndTable();
  }
}

void BBVisualizer::plotPieGraphs(int dim){
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


bool CompareWithSortSpecs(const bb_interfaces::msg::STrack& a, const bb_interfaces::msg::STrack& b)
  {
    for (int n = 0; n < BBVisualizer::s_current_sort_specs->SpecsCount; n++)
    {
        const ImGuiTableColumnSortSpecs* sort_spec = &BBVisualizer::s_current_sort_specs->Specs[n];
        int delta = 0;
        switch (sort_spec->ColumnIndex)
        {
        case 0:    delta = (a.track_id - b.track_id);                              break;
        case 1:    delta = (a.is_activated ? 1 : -1);                               break;
        case 2:    delta = (strcmp(a.state.c_str(), b.state.c_str()));             break;
        case 3:    delta = (strcmp(a.class_name.c_str(), b.class_name.c_str()));   break;
        case 4:    delta = (a.score - b.score);                                    break;
        case 5:    delta = (a.mean[0] - b.mean[0]);                                break;
        case 6:    delta = (a.mean[1] - b.mean[1]);                                break;
        case 7:    delta = (a.mean[2] - b.mean[2]);                                break;
        case 8:    delta = (a.mean[3] - b.mean[3]);                                break;
        case 9:    delta = (a.mean[4] - b.mean[4]);                                break;
        case 10:   delta = (a.mean[5] - b.mean[5]);                                break;
        case 11:   delta = (a.mean[6] - b.mean[6]);                                break;
        case 12:   delta = (a.mean[7] - b.mean[7]);                                break;
        default: IM_ASSERT(0); break;
        }
        if (delta > 0)
            return (sort_spec->SortDirection == ImGuiSortDirection_Ascending);
        if (delta < 0)
            return (sort_spec->SortDirection != ImGuiSortDirection_Ascending);
      }

      // Always return a way to differenciate items.
      return (a.track_id - b.track_id)>0;
  }





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BBVisualizer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}