#include <log2plot/generic_container.h>

namespace log2plot
{

void GenericContainer::setFile(std::string s)
{
  filename = s;
  yaml_stream.open(s, std::ios_base::trunc);
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
  default:
    writeInfo("dataType", "3D pose");
  }
}

void GenericContainer::writeInfo(const std::string &_label, const std::string &_info)
{
  yaml_stream << _label << ": " << _info << '\n';
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
