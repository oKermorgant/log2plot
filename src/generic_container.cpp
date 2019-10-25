#include <log2plot/generic_container.h>

namespace log2plot
{


void GenericContainer::writeInfo(const std::string &_label, const std::string &_info)
{
  ofs_ << _label << ": " << _info << '\n';
}


std::string GenericContainer::close(const std::vector<double> &steps,
                         const std::vector<double> &steps_timed)
{
  if(ofs_.is_open())
  {
    // add steps if needed
    if(!has_steps)
    {
      if(log_type == ITERATION)
        setSteps(steps);
      else if(log_type == TIME)
        setSteps(steps_timed);
    }
    ofs_.close();
  }
  return filename;
}


void GenericContainer::setSteps(const std::vector<double> &steps)
{
  if(steps.size() == 0 || (log_type != ITERATION && log_type != TIME))
    return;

  has_steps = true;
  ofs_ << "steps" << ": [";
  for(size_t i = 0; i < steps.size()-1; ++i)
    ofs_ << steps[i] << ", ";
  ofs_ << steps.back() << "]\n";
}

}
