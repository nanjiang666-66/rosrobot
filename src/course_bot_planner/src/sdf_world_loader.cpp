#include "course_bot_planner/sdf_world_loader.hpp"

#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Plane.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/World.hh>
#include <sdf/parser.hh>

namespace course_bot_planner {
namespace {

namespace fs = std::filesystem;

void append_colon_separated_paths(const char *text,
                                  std::vector<fs::path> &paths) {
  if (text == nullptr) return;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ':')) {
    if (!item.empty()) paths.emplace_back(item);
  }
}

std::vector<fs::path> gazebo_model_paths() {
  std::vector<fs::path> paths;
  append_colon_separated_paths(std::getenv("GAZEBO_MODEL_PATH"), paths);
  if (const char *home = std::getenv("HOME")) {
    paths.emplace_back(fs::path(home) / ".gazebo" / "models");
  }
  // Gazebo Classic 11 在 Ubuntu 22.04 的系统模型默认安装位置。
  paths.emplace_back("/usr/share/gazebo-11/models");
  return paths;
}

std::string join_paths(const std::vector<fs::path> &paths) {
  std::ostringstream joined;
  for (std::size_t i = 0; i < paths.size(); ++i) {
    if (i != 0) joined << ':';
    joined << paths[i].string();
  }
  return joined.str();
}

void configure_sdf_model_paths() {
  const std::vector<fs::path> paths = gazebo_model_paths();
  sdf::addURIPath("model://", join_paths(paths));

  // Root::Load 解析 <include> 时会要求回调。这里实现与 Gazebo Classic
  // 一致的 model:// 搜索，不依赖 Gazebo 进程是否已经初始化。
  sdf::setFindCallback([paths](const std::string &uri) -> std::string {
    constexpr const char *prefix = "model://";
    fs::path relative;
    if (uri.rfind(prefix, 0) == 0) {
      relative = uri.substr(std::char_traits<char>::length(prefix));
    } else {
      const fs::path ordinary_path(uri);
      if (fs::exists(ordinary_path)) return fs::absolute(ordinary_path).string();
      // 某些 libsdformat9 调用点会把 model:// 前缀先去掉再调用回调。
      relative = ordinary_path;
    }

    if (relative.empty() || relative.is_absolute()) return "";
    for (const auto &component : relative) {
      if (component == "..") return "";
    }
    for (const auto &base : paths) {
      const fs::path candidate = base / relative;
      if (fs::exists(candidate)) return fs::canonical(candidate).string();
    }
    return "";
  });
}

std::string error_messages(const sdf::Errors &errors) {
  std::ostringstream message;
  for (std::size_t i = 0; i < errors.size(); ++i) {
    if (i != 0) message << "; ";
    message << errors[i].Message();
  }
  return message.str();
}

std::string collision_path(const sdf::Model &model, const sdf::Link &link,
                           const sdf::Collision &collision) {
  return model.Name() + "::" + link.Name() + "::" + collision.Name();
}

std::optional<StaticObstacle2D> read_collision(const sdf::Model &model,
                                               const sdf::Link &link,
                                               const sdf::Collision &collision) {
  const std::string name = collision_path(model, link, collision);
  const sdf::Geometry *geometry = collision.Geom();
  if (geometry == nullptr) {
    throw std::runtime_error("碰撞几何不存在：" + name);
  }

  // Collision 的位姿图只在当前 model 内有效，不能直接查询 world。
  // 因此先解析 collision -> __model__（其中已包含 link pose），再解析
  // model -> world，最后按变换顺序相乘。
  auto collision_in_model = collision.RawPose();
  const auto collision_pose_errors =
      collision.SemanticPose().Resolve(collision_in_model, "__model__");
  if (!collision_pose_errors.empty()) {
    throw std::runtime_error("无法解析碰撞体到模型坐标系的位姿 " + name + "：" +
                             error_messages(collision_pose_errors));
  }

  auto model_in_world = model.RawPose();
  const auto model_pose_errors =
      model.SemanticPose().Resolve(model_in_world, "world");
  if (!model_pose_errors.empty()) {
    throw std::runtime_error("无法解析模型到世界坐标系的位姿 " + model.Name() + "：" +
                             error_messages(model_pose_errors));
  }
  const auto world_pose = model_in_world * collision_in_model;

  if (geometry->Type() == sdf::GeometryType::PLANE) {
    const auto *plane = geometry->PlaneShape();
    if (plane == nullptr) throw std::runtime_error("Plane 参数无效：" + name);
    const auto world_normal = world_pose.Rot().RotateVector(plane->Normal());
    // 水平无限平面是地面，不是需要在二维栅格中膨胀的障碍物。
    // 使用法向量判断而不是模型名称，因此 ground_plane_0 等也能安全处理。
    if (std::abs(world_normal.X()) < 1e-6 &&
        std::abs(world_normal.Y()) < 1e-6 &&
        std::abs(std::abs(world_normal.Z()) - 1.0) < 1e-6) {
      return std::nullopt;
    }
    throw std::runtime_error("暂不支持竖直或倾斜 Plane 的二维投影：" + name);
  }

  StaticObstacle2D obstacle;
  obstacle.name = name;
  obstacle.center = {world_pose.Pos().X(), world_pose.Pos().Y()};
  obstacle.yaw = world_pose.Rot().Yaw();

  switch (geometry->Type()) {
    case sdf::GeometryType::BOX: {
      const auto *box = geometry->BoxShape();
      if (box == nullptr || box->Size().X() <= 0.0 || box->Size().Y() <= 0.0) {
        throw std::runtime_error("Box 尺寸无效：" + name);
      }
      if (std::abs(world_pose.Rot().Roll()) > 1e-6 ||
          std::abs(world_pose.Rot().Pitch()) > 1e-6) {
        throw std::runtime_error("暂不支持倾斜 Box 的二维投影：" + name);
      }
      obstacle.shape = ObstacleShape::Box;
      obstacle.size_x = box->Size().X();
      obstacle.size_y = box->Size().Y();
      break;
    }
    case sdf::GeometryType::CYLINDER: {
      const auto *cylinder = geometry->CylinderShape();
      if (cylinder == nullptr || cylinder->Radius() <= 0.0) {
        throw std::runtime_error("Cylinder 半径无效：" + name);
      }
      if (std::abs(world_pose.Rot().Roll()) > 1e-6 ||
          std::abs(world_pose.Rot().Pitch()) > 1e-6) {
        throw std::runtime_error("暂不支持倾斜 Cylinder 的二维投影：" + name);
      }
      obstacle.shape = ObstacleShape::Cylinder;
      obstacle.radius = cylinder->Radius();
      break;
    }
    case sdf::GeometryType::SPHERE: {
      const auto *sphere = geometry->SphereShape();
      if (sphere == nullptr || sphere->Radius() <= 0.0) {
        throw std::runtime_error("Sphere 半径无效：" + name);
      }
      obstacle.shape = ObstacleShape::Sphere;
      obstacle.radius = sphere->Radius();
      break;
    }
    default:
      throw std::runtime_error(
          "静态碰撞体使用了当前不支持的几何形状（仅支持 box/cylinder/sphere）：" + name);
  }
  return obstacle;
}

}  // namespace

SdfWorldLoadResult load_static_obstacles_from_sdf(const std::string &world_file) {
  if (world_file.empty()) throw std::invalid_argument("world_file 参数不能为空");

  configure_sdf_model_paths();
  sdf::Root root;
  const sdf::Errors errors = root.Load(world_file);
  if (!errors.empty()) {
    throw std::runtime_error(
        "读取 SDF 世界失败 '" + world_file + "'：" + error_messages(errors) +
        "；Gazebo 模型搜索路径：" + join_paths(gazebo_model_paths()));
  }
  if (root.WorldCount() != 1) {
    throw std::runtime_error("SDF 世界文件必须且只能包含一个 <world>：" + world_file);
  }

  const sdf::World *world = root.WorldByIndex(0);
  if (world == nullptr) throw std::runtime_error("SDF 中未找到 world：" + world_file);

  SdfWorldLoadResult result;
  for (std::uint64_t model_index = 0; model_index < world->ModelCount(); ++model_index) {
    const sdf::Model *model = world->ModelByIndex(model_index);
    if (model == nullptr) throw std::runtime_error("读取 SDF model 时出现空指针");

    if (!model->Static()) {
      ++result.skipped_dynamic_models;
      continue;
    }
    for (std::uint64_t link_index = 0; link_index < model->LinkCount(); ++link_index) {
      const sdf::Link *link = model->LinkByIndex(link_index);
      if (link == nullptr) throw std::runtime_error("读取 SDF link 时出现空指针");
      for (std::uint64_t collision_index = 0;
           collision_index < link->CollisionCount(); ++collision_index) {
        const sdf::Collision *collision = link->CollisionByIndex(collision_index);
        if (collision == nullptr) {
          throw std::runtime_error("读取 SDF collision 时出现空指针");
        }
        const auto obstacle = read_collision(*model, *link, *collision);
        if (obstacle) {
          result.obstacles.push_back(*obstacle);
        } else {
          ++result.skipped_horizontal_planes;
        }
      }
    }
  }
  if (result.obstacles.empty()) {
    throw std::runtime_error("SDF 中没有可用于建图的静态碰撞体：" + world_file);
  }
  return result;
}

}  // namespace course_bot_planner
