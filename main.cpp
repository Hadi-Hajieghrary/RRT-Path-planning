
#include <chrono>
#include <memory>
#include <string>
#include <yaml.h>

#include <opencv2/opencv.hpp>

#include "boost/program_options.hpp"

#include "RRT.hpp"

int main(int argc, char** argv) {

  
  
  const size_t terminate_search_cost{4000};
  const size_t max_sampling_num{10000};
  const double goal_tolerance{30.0};

  size_t world_size_x, world_size_y;
  size_t start_position_x, start_position_y;
  size_t goal_position_x, goal_position_y;

  std::string img_file_path;

  // Accept inputs from the commandline
  try{
    boost::program_options::options_description arg_description("This program will deploy RRT to find a path through am Emage from Start to Goal position.");
    auto add_opt = arg_description.add_options();
    add_opt = add_opt("help,h", "Help on the program and program options");
    add_opt = add_opt("image-file,f", boost::program_options::value<std::string>(), "Path to the image");
    add_opt = add_opt("start-position,s", boost::program_options::value<std::string>(), "Start Position: (x,y)");
    add_opt = add_opt("goal-position,g", boost::program_options::value<std::string>(), "Goal Position: (x,y)");
    
    boost::program_options::variables_map opt_map;
    auto cmd_line = boost::program_options::parse_command_line(argc, argv, arg_description);
    boost::program_options::store(cmd_line, opt_map);
    if (opt_map.count("help")) {
        std::cout << "This Program implements plain RRT." << std::endl;
        std::cout << arg_description << "\n";
        return 0;
    }

    if (opt_map.count("image-file") != 0) {
        img_file_path = opt_map["image-file"].as<std::string>();
    }else{
        throw std::runtime_error("No Image File is Provided!\n");
    }

    auto getElements = [](std::string str){size_t l = str.size(); size_t mid = str.find(',');return std::make_pair(std::stoi(str.substr(1,mid-1)),  std::stoi(str.substr(mid+1, l-mid-2)));};
    if (opt_map.count("start-position") != 0) {
      try{
        std::string input = opt_map["start-position"].as<std::string>();
        std::tie(start_position_x,start_position_y) = getElements(input);  
      }catch(...){
        std::cout<<"Please provide a valid Start Position: (x,y)"<<std::endl;
        return 1;
      }    
    }else{
        throw std::runtime_error("Start Position is to be provided!!\n");
    }      

    if (opt_map.count("goal-position") != 0) {
      try{
        std::string input = opt_map["goal-position"].as<std::string>();
        std::tie(goal_position_x,goal_position_y) = getElements(input);
      }catch(...){
        std::cout<<"Please provide a valid Goal Position: (x,y)"<<std::endl;
        return 1;
      }    
    }else{
        throw std::runtime_error("Goal Position is to be provided!!\n");
    }    
  }catch(std::exception& e){
      std::cout << e.what();
      return 1;
  }


  //Construct OpenCV Image Object
  cv::Mat world;

  try{
    world = cv::imread(img_file_path, CV_8UC1);
    world_size_x = world.cols;
    world_size_y = world.rows;
  }catch(...){
    std::cout<<"Can not open the File at this position: '" + img_file_path + " '"<<std::endl;
    return 1;
  }
  printf ("\nPlanning path from the Start Point: (%ld,%ld) to the Goal Point: (%ld,%ld)...\n", start_position_x, start_position_y, goal_position_x, goal_position_y);

  BoundingBox space;
  std::vector<Bound> bounds{Bound(0, world_size_x), Bound(0, world_size_y)};
  space.setBound(bounds);
  std::vector<bool> occupancy_map(world_size_x * world_size_y, 1);
  for(size_t i = 0; i< world_size_x * world_size_y; ++i){
    occupancy_map[i] = !(world.data[i]);
  }
// Construct the main object to implement RRT
  std::shared_ptr<GridConstraint> constraint = std::make_shared<GridConstraint>(space, occupancy_map, std::vector<int>{world_size_x,world_size_y});

  std::unique_ptr<RRT> planner = std::make_unique<RRT>(max_sampling_num, goal_tolerance);


  planner->setUp(constraint);
  planner->setSearchCostMaxLimit(terminate_search_cost);

  auto start_time = std::chrono::system_clock::now();
  bool status = planner->solve(State({start_position_x, start_position_y}), State({goal_position_x, goal_position_y}));
  auto end_time = std::chrono::system_clock::now();

  if (status) {
    // draw and output result
    auto node_list = planner->getNodeList();
    auto result = planner->getResult();

    cv::cvtColor(world, world, cv::COLOR_BGR2RGB);

    for (int yi = 0; yi < world.rows; yi++) {
      for (int xi = 0; xi < world.cols; xi++) {
        if (world.at<cv::Vec3b>(yi, xi) != cv::Vec3b(0, 0, 0)) {
          world.at<cv::Vec3b>(yi, xi) = cv::Vec3b(250, 250, 250);
        }
      }
    }

    auto leafs = node_list->searchLeafs();
    for (auto node : leafs) {
      while (node->_parent != nullptr) {
        cv::line(world, cv::Point(node->_state.coordinates[0], node->_state.coordinates[1]),
                  cv::Point(node->_parent->_state.coordinates[0], node->_parent->_state.coordinates[1]), cv::Vec3b(200, 150, 100), 1.0,
                  CV_AVX);
        node = node->_parent;
      }
    }

    auto prev_node_pos = cv::Point(result[0].coordinates[0], result[0].coordinates[1]);
    for (const auto &point : result) {
      cv::circle(world, cv::Point(point.coordinates[0], point.coordinates[1]), 3.0, cv::Vec3b(150, 150, 250), 1, CV_AVX);
      cv::line(world, cv::Point(point.coordinates[0], point.coordinates[1]), prev_node_pos, cv::Vec3b(150, 250, 150), 1, CV_AVX);
      prev_node_pos = cv::Point(point.coordinates[0], point.coordinates[1]);
    }

    cv::circle(world, cv::Point(result.front().coordinates[0], result.front().coordinates[1]), 6.0, cv::Vec3b(150, 250, 250), -1,
                CV_AVX);
    cv::putText(world, "Start", cv::Point(result.front().coordinates[0] + 10, result.front().coordinates[1]),
                cv::FONT_HERSHEY_COMPLEX_SMALL | cv::FONT_ITALIC, 1.0, cv::Scalar(150, 250, 250), 1, CV_AVX);

    cv::circle(world, cv::Point(result.back().coordinates[0], result.back().coordinates[1]), 6.0, cv::Vec3b(150, 250, 250), -1,
                CV_AVX);
    cv::putText(world, "Goal", cv::Point(result.back().coordinates[0] + 10, result.back().coordinates[1]),
                cv::FONT_HERSHEY_COMPLEX_SMALL | cv::FONT_ITALIC, 1.0, cv::Scalar(150, 250, 250), 1, CV_AVX);

    cv::putText(world, "Total Length:" + std::to_string(int(planner->getResultCost())), cv::Point( world_size_x - 400, world_size_y - 10),
                cv::FONT_HERSHEY_COMPLEX_SMALL | cv::FONT_ITALIC, 1.0, cv::Scalar(10, 10, 250), 1, CV_AVX);


    printf("--> List of Waypoints:\n");
    auto waypoint = result.begin();
    printf("    Start Point: (%.0f, %.0f)\n", waypoint->coordinates[0],  waypoint->coordinates[1]);
    int p = 1;
    for(; waypoint != result.end() - 1; ++waypoint){
      printf("    --> Waypoint %d: (%.0f, %.0f) \n", p++, waypoint->coordinates[0],  waypoint->coordinates[1]);
    }
    printf("   End Point: (%.0f, %.0f)\n", waypoint->coordinates[0],  waypoint->coordinates[1]);
    std::cout<<std::flush;
    cv::namedWindow("World", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::imshow("World", world);

    std::string& processed_omg_file_path = img_file_path.insert(img_file_path.size()-4,"_processed").erase(3, 7);
    cv::imwrite(processed_omg_file_path, world);
    printf("The Processed image (with planned path) is saved at: '%s'\n",processed_omg_file_path.c_str());
    std::cout << "Time Ellapsed : " << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << "milli-second"<< std::endl;
    std::cout<<std::flush;
    cv::waitKey(0);
    cv::destroyWindow("World");
    


  } else {
    std::cout << "No path could be found!" << std::endl;
    std::cout << "Time Ellapsed : " << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << "milli-second\n"<< std::endl;
  }

  

  return 0;
}
