#include <path_searching/dyn_a_star.h>
#include <iostream>

// Run with sentry_planner.yaml. No sensor/robot or executor is started.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scan_planner_node", "/sentry_sim");
  auto map = std::make_shared<GridMap>();
  map->initMap(node.get());
  int failures = 0;
  {
    AStar search;
    search.initGridMap(map, Eigen::Vector3i(100, 100, 100));
    for (double dz : {0.0, 0.20, -0.20, -0.35}) {
      Eigen::Vector3d start(0.0, 0.0, 1.0), end(0.0, 1.0, 1.0 + dz);
      const auto result = search.AstarSearch(0.05, start, end);
      const auto path = result == SUCCESS ? search.getPath() : std::vector<Eigen::Vector3d>{};
      const bool ok = !path.empty() && (path.back() - end).norm() <= 0.045;
      std::cout << "empty-map dz=" << dz << " result=" << result << " pass=" << ok << '\n';
      if (!ok) ++failures;
    }
  }
  map.reset();
  node.reset();
  rclcpp::shutdown();
  return failures ? 1 : 0;
}
