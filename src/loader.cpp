#include <log2plot/loader.h>
#include <yaml-cpp/yaml.h>

using namespace log2plot;

Log::Log(const std::string &path)
{
  const auto root = YAML::LoadFile(path);
  type = getType(root["dataType"].as<std::string>());

  if(type == LogType::METAFILE)
  {
    Loader::load(root["files"], files);
    return;
  }

  if(!Loader::load(root["legend"], legend))
  {
    legend.resize(1);
    if(!Loader::load(root["legend"], legend))
      legend.clear();
  }
  Loader::load(root["data"], data);

  if(type == LogType::POSE)
    Loader::load(root["invertPose"], invert_pose);

  // look for fixed objects
  uint i = 1;
  while(++i)
  {
    const auto key{"fixedObject" + std::to_string(i)};
    const auto &node{root[key]};
    if(!node)
      break;
    auto &last{fixed_objects.emplace_back()};

    Loader::load(node["legend"], last.legend);
    Loader::load(node["nodes"], last.points);
    Loader::load(node["graph"], last.graph);
  }
}
