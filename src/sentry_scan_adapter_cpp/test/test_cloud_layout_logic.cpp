// 点云布局校验的确定性反例（gtest）：缺字段 / 错类型 / 截断 / 大端 / 行跨度不足都必须被拒绝。
#define RM_INPUT_ADAPTER_UNIT_TEST
#include "rm_input_adapter_node.cpp"

#include <gtest/gtest.h>

using sentry_scan_adapter_cpp::validate_cloud_layout;

constexpr uint8_t F32 = sensor_msgs::msg::PointField::FLOAT32;
constexpr uint8_t F64 = sensor_msgs::msg::PointField::FLOAT64;

namespace {

sensor_msgs::msg::PointCloud2 make_cloud(std::vector<sensor_msgs::msg::PointField> fields,
                                        std::size_t point_step, std::size_t points) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points);
  msg.fields = std::move(fields);
  msg.is_bigendian = false;
  msg.point_step = static_cast<uint32_t>(point_step);
  msg.row_step = msg.point_step * msg.width;
  msg.data.assign(msg.row_step * msg.height, 0);
  msg.is_dense = true;
  return msg;
}

sensor_msgs::msg::PointField field(const std::string &name, uint32_t offset, uint8_t type) {
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  return f;
}

TEST(CloudLayout, RealLivox48ByteLayoutAccepted) {
  auto msg = make_cloud({field("x", 0, F32), field("y", 4, F32), field("z", 8, F32),
                         field("normal_x", 16, F32), field("normal_y", 20, F32),
                         field("normal_z", 24, F32), field("intensity", 32, F32),
                         field("curvature", 36, F32)},
                        48, 10);
  EXPECT_EQ(validate_cloud_layout(msg), "");
}

TEST(CloudLayout, RejectsMissingXyzFields) {
  // 反例：只有 intensity（没有 xyz）。PCL 会给出 10 个 (0,0,0) 有限点，必须在这里挡住。
  auto msg = make_cloud({field("intensity", 0, F32)}, 12, 10);
  EXPECT_NE(validate_cloud_layout(msg).find("missing x/y/z"), std::string::npos);
}

TEST(CloudLayout, RejectsWrongFieldType) {
  auto msg = make_cloud({field("x", 0, F64), field("y", 8, F64), field("z", 16, F64)},
                        24, 4);
  EXPECT_NE(validate_cloud_layout(msg).find("not FLOAT32"), std::string::npos);
}

TEST(CloudLayout, RejectsBigEndian) {
  auto msg = make_cloud({field("x", 0, F32), field("y", 4, F32), field("z", 8, F32)}, 12, 4);
  msg.is_bigendian = true;
  EXPECT_NE(validate_cloud_layout(msg).find("is_bigendian"), std::string::npos);
}

TEST(CloudLayout, RejectsTruncatedData) {
  auto msg = make_cloud({field("x", 0, F32), field("y", 4, F32), field("z", 8, F32)}, 12, 10);
  msg.data.resize(48);
  EXPECT_NE(validate_cloud_layout(msg).find("truncated"), std::string::npos);
}

TEST(CloudLayout, RejectsBadRowStepAndOffset) {
  auto row = make_cloud({field("x", 0, F32), field("y", 4, F32), field("z", 8, F32)}, 12, 4);
  row.row_step = 8;
  EXPECT_NE(validate_cloud_layout(row).find("row_step"), std::string::npos);

  auto offset = make_cloud({field("x", 0, F32), field("y", 4, F32), field("z", 10, F32)}, 12, 4);
  EXPECT_NE(validate_cloud_layout(offset).find("offset"), std::string::npos);
}

TEST(CloudLayout, RejectsFieldCountNotOne) {
  auto msg = make_cloud({field("x", 0, F32), field("y", 4, F32), field("z", 8, F32)}, 12, 4);
  msg.fields[2].count = 2;
  EXPECT_NE(validate_cloud_layout(msg).find("count"), std::string::npos);
}

}  // namespace
