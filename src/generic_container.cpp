#include <log2plot/generic_container.h>
#include <log2plot/yaml.h>
#include <filesystem>

namespace log2plot
{

void GenericContainer::setFile(const std::string &file)
{
  filename = file;
  std::filesystem::create_directories(std::filesystem::path(file).parent_path());

  yaml_stream.open(file, std::ios_base::trunc);
  // set log type in file
  switch(log_type)
  {
    case(LogType::XY):
      writeInfo("dataType", "XY");
      break;
    case(LogType::ITERATION):
      writeInfo("dataType", "iteration-based");
      break;
    case(LogType::TIME):
      writeInfo("dataType", "time-based");
      break;
    case(LogType::TIMED_XY):
      writeInfo("dataType", "timed-XY");
      break;
    case(LogType::POSE):
      writeInfo("dataType", "3D pose");
    default:
      break;
  }
}

void GenericContainer::writeInfo(const std::string &key, const std::string &value)
{
  if(key.empty())
    return;
  yaml_stream << key << ": " <<  value << '\n';
}

std::string GenericContainer::close(const std::vector<double> &steps,
                                    const std::vector<double> &steps_timed)
{
  if(yaml_stream.is_open())
  {
    // add steps if needed
    if(!has_steps)
    {
      if(log_type == LogType::ITERATION)
        setSteps(steps);
      else if(log_type == LogType::TIME)
        setSteps(steps_timed);
    }
    yaml_stream.close();
  }
  return filename;
}


void GenericContainer::setSteps(const std::vector<double> &steps)
{
  if(steps.size() == 0 || (log_type != LogType::ITERATION && log_type != LogType::TIME))
    return;

  has_steps = true;
  yaml_stream << "steps" << ": [";
  for(size_t i = 0; i < steps.size()-1; ++i)
    yaml_stream << steps[i] << ", ";
  yaml_stream << steps.back() << "]\n";
}

}
