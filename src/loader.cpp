#include <log2plot/loader.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>

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
  uint i{};
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


std::vector<std::string> Log::getObjectLegends() const
{
  std::vector<std::string> legends;
  for(const auto &obj: fixed_objects)
  {
    if(!obj.legend.empty())
      legends.push_back(obj.legend);
  }
  return legends;
}

Log::FixedObject Log::getObject(const std::string &legend) const
{
  const auto obj{std::find_if(fixed_objects.begin(),fixed_objects.end(),
                        [legend](const auto &obj){return obj.legend == legend;})};

  if(obj != fixed_objects.end())
    return *obj;
  return {};
}
