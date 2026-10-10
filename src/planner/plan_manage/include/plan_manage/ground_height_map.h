// 地面高度网格：从 prepare_terrain_map.py 生成的文本网格读取，提供双线性查询。
//
// 用途：让地面机器人的 z 跟随地形，并保证**规划碰撞检查的 z 与仿真运动学的 z
// 来自同一张地形网格**。地图里的地面点不能喂给 SCAN（占据栅格会把地面本身当成障碍），
// 所以地面以这个独立网格的形式提供给运动模拟器与规划器。
//
// 文件格式（见 scripts/prepare_terrain_map.py）：
//     # cell <c> x0 <x0> y0 <y0> nx <nx> ny <ny>
//     <z00> <z10> ... （共 ny 行，每行 nx 个值，x 递增、y 递增）
//
// 边界语义（重要）：
//   - contains(x, y) 判定查询点是否落在网格覆盖范围内。**越界不代表可行驶地面**，
//     调用方必须显式处理：规划器对越界点回退到线性插值，模拟器保持上一次有效高度并告警。
//   - heightAt(x, y) 对越界点做**连续坐标夹取后再插值**（不是先夹索引再拿原始小数部分），
//     否则在左/下边界会得到随距离反向变化的高度（已修正的缺陷）。

#ifndef PLAN_MANAGE_GROUND_HEIGHT_MAP_H
#define PLAN_MANAGE_GROUND_HEIGHT_MAP_H

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace scan_planner
{
class GroundHeightMap
{
public:
  bool load(const std::string &path)
  {
    std::ifstream file(path);
    if (!file.is_open())
      return false;

    values_.clear();
    std::string line;
    bool header_parsed = false;
    while (std::getline(file, line))
    {
      if (line.empty())
        continue;
      if (line[0] == '#')
      {
        // 只在第一行含 "cell" 的注释里解析头部。其它注释（例如
        // "each row nx values"）同样含 nx/ny 这样的词，参与解析会把 nx 读成 0。
        if (!header_parsed && line.find("cell") != std::string::npos)
        {
          std::istringstream stream(line);
          std::string token;
          while (stream >> token)
          {
            if (token == "cell") { if (!(stream >> cell_)) cell_ = 0.0; }
            else if (token == "x0") { if (!(stream >> x0_)) x0_ = 0.0; }
            else if (token == "y0") { if (!(stream >> y0_)) y0_ = 0.0; }
            else if (token == "nx") { if (!(stream >> nx_)) nx_ = 0; }
            else if (token == "ny") { if (!(stream >> ny_)) ny_ = 0; }
          }
          header_parsed = (nx_ > 0 && ny_ > 0 && cell_ > 0.0);
        }
        continue;
      }
      std::istringstream stream(line);
      double value = 0.0;
      while (stream >> value)
        values_.push_back(value);
    }

    if (!header_parsed)
      return false;
    if (values_.size() != static_cast<size_t>(nx_) * static_cast<size_t>(ny_))
      throw std::runtime_error("ground grid size mismatch in " + path + ": got " +
                               std::to_string(values_.size()) + ", expected " +
                               std::to_string(nx_ * ny_));
    loaded_ = true;
    return true;
  }

  bool loaded() const { return loaded_; }
  double cell() const { return cell_; }

  /// 网格覆盖范围（以格中心为界）。
  double minX() const { return x0_; }
  double maxX() const { return x0_ + (nx_ - 1) * cell_; }
  double minY() const { return y0_; }
  double maxY() const { return y0_ + (ny_ - 1) * cell_; }

  /// 查询点是否落在网格覆盖范围内（含半格余量）。越界时调用方必须自行决定回退策略，
  /// 不能默认当作可行驶地面。
  bool contains(double x, double y) const
  {
    if (!loaded_)
      return false;
    const double half = 0.5 * cell_;
    return x >= minX() - half && x <= maxX() + half &&
           y >= minY() - half && y <= maxY() + half;
  }

  /// 双线性插值。越界时先把**连续坐标**夹取到网格范围再插值，
  /// 因此边界外得到的是边界格的值（常量外推），不会出现反向变化或跳变。
  double heightAt(double x, double y) const
  {
    if (!loaded_)
      return 0.0;
    const double fx_raw = (x - x0_) / cell_;
    const double fy_raw = (y - y0_) / cell_;
    const double fx = std::clamp(fx_raw, 0.0, static_cast<double>(nx_ - 1));
    const double fy = std::clamp(fy_raw, 0.0, static_cast<double>(ny_ - 1));

    const int i0 = std::min(static_cast<int>(std::floor(fx)), nx_ - 1);
    const int j0 = std::min(static_cast<int>(std::floor(fy)), ny_ - 1);
    const int i1 = std::min(i0 + 1, nx_ - 1);
    const int j1 = std::min(j0 + 1, ny_ - 1);
    const double tx = fx - static_cast<double>(i0);
    const double ty = fy - static_cast<double>(j0);

    const double a = at(i0, j0) * (1.0 - tx) + at(i1, j0) * tx;
    const double b = at(i0, j1) * (1.0 - tx) + at(i1, j1) * tx;
    return a * (1.0 - ty) + b * ty;
  }

  double minHeight() const
  {
    return values_.empty() ? 0.0 : *std::min_element(values_.begin(), values_.end());
  }
  double maxHeight() const
  {
    return values_.empty() ? 0.0 : *std::max_element(values_.begin(), values_.end());
  }

private:
  double at(int i, int j) const
  {
    return values_[static_cast<size_t>(j) * static_cast<size_t>(nx_) + static_cast<size_t>(i)];
  }

  std::vector<double> values_;
  double cell_{0.0}, x0_{0.0}, y0_{0.0};
  int nx_{0}, ny_{0};
  bool loaded_{false};
};
}  // namespace scan_planner

#endif  // PLAN_MANAGE_GROUND_HEIGHT_MAP_H
