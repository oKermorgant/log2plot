#include "log2plot_yaml.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QSettings>
#include <QProgressDialog>
#include <QDateTime>
#include <QInputDialog>
#include <yaml-cpp/yaml.h>

#ifdef WITH_EIGEN
#include <eigen3/Eigen/Geometry>
#endif

bool DataLoadYAML::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_data)
{
  RelevantFields fields(plot_data);
  bool invert{false};
  {
    YAML::Node content;
    try
    {
      content = YAML::LoadFile(info->filename.toStdString());

      fields.datatype = content["dataType"].as<std::string>();

      if(fields.datatype == "timed-xy")
      {
        auto err_msg = QString("The data file contains a varying XY curve,\n"
                               "PlotJuggler is not design for those");
        QMessageBox::warning(nullptr, "Error reading file", err_msg );
        return false;
      }

      fields.data = content["data"];
      fields.legend = content["legend"];
#ifdef WITH_EIGEN
      if(auto inv_field = content["invertPose"]; inv_field.IsDefined())
      {
        invert = inv_field.as<bool>();
        if(invert)
           QMessageBox::information(nullptr, "When loading file", "Loading inverted pose");
      }
#endif
    }
    catch (const std::exception *e)
    {
      return false;
    }
  }

  if(fields.datatype == "3D pose")
  {
    return readPose3D(fields, invert);
  }

  if(fields.datatype == "XY")
    return readXY(fields);

  return readBasic(fields);
}

bool DataLoadYAML::readBasic(const RelevantFields &fields)
{   
  const auto timed{fields.datatype == "time-based"};
  const auto size{fields.legend.size() + timed};

  // create a vector of timeseries
  std::vector<PlotData*> plots_vector;

  for (unsigned i = 0; i < size; i++)
  {
    auto it = fields.plot_data.addNumeric(fields.title(i));
    plots_vector.push_back(&(it->second));
  }

  auto linecount{0};
  for(const auto &row: fields.data)
  {
    if(row.size() != size)
    {
      auto err_msg = QString("The number of values at line %1 is %2,\n"
                             "but the expected number of columns is %3.\n"
                             "Aborting...")
          .arg(linecount)
          .arg(row.size())
          .arg(size);
      QMessageBox::warning(nullptr, "Error reading file", err_msg );
      return false;
    }

    const double x = timed ? row[0].as<double>() : linecount;

    for(size_t col = timed; col < row.size(); ++col)
      plots_vector[col]->pushBack(PlotData::Point(x, row[col].as<double>()));

    linecount++;
  }

  return true;
}


bool DataLoadYAML::readXY(const RelevantFields &fields)
{
  const auto size{fields.legend.size()};

  // create a vector of timeseries
  std::vector<PlotData*> plots_vector;

  for (unsigned i = 0; i < size; i++)
  {
    auto group = fields.plot_data.getOrCreateGroup(fields.title(i));
    auto x = fields.plot_data.addNumeric(fields.title(i)+"_x", group);
    plots_vector.push_back(&(x->second));
    auto y = fields.plot_data.addNumeric(fields.title(i)+"_y", group);
    plots_vector.push_back(&(y->second));
  }

  auto linecount{0};
  for(const auto &row: fields.data)
  {
    if(row.size() != 2*size)
    {
      auto err_msg = QString("The number of values at line %1 is %2,\n"
                             "but the expected number of columns is %3.\n"
                             "Aborting...")
          .arg(linecount)
          .arg(row.size())
          .arg(size);
      QMessageBox::warning(nullptr, "Error reading file", err_msg );
      return false;
    }

    for(size_t col = 0; col < size; ++col)
    {
      plots_vector[2*col]->pushBack(PlotData::Point(linecount,
                                  row[2*col].as<double>()));
      plots_vector[2*col+1]->pushBack(PlotData::Point(linecount,
                                  row[2*col+1].as<double>()));
    }
    linecount++;
  }
  return true;
}

bool DataLoadYAML::readPose3D(const RelevantFields &fields, bool invert_pose)
{
  std::vector<PlotData*> plots_vector;
  const auto traj{fields.title(0)};

  auto pose = fields.plot_data.getOrCreateGroup(traj);

  for(auto axis: {"x","y","z","R","P","Y"})
  {
    auto x = fields.plot_data.addNumeric(axis, pose);
    plots_vector.push_back(&(x->second));
  }

  auto linecount{0};
  for(const auto &row: fields.data)
  {
    if(row.size() != 6)
    {
      auto err_msg = QString("The number of values at line %1 is %2,\n"
                             "but the expected number of columns is 6.\n"
                             "Aborting...")
          .arg(linecount)
          .arg(row.size());
      QMessageBox::warning(nullptr, "Error reading file", err_msg );
      return false;
    }
#ifdef WITH_EIGEN
    if(invert_pose)
    {
      const Eigen::Vector3d aa{row[3].as<double>(),row[4].as<double>(),row[5].as<double>()};
      const auto tu{Eigen::AngleAxisd(-aa.norm(), aa/aa.norm())};
      const Eigen::Vector3d t{row[0].as<double>(),row[1].as<double>(),row[2].as<double>()};
      const Eigen::Vector3d ti{-tu.toRotationMatrix() * t};

      for(size_t col = 0; col < 3; ++col)
      {
        plots_vector[col]->pushBack(PlotData::Point(linecount,ti(col)));
        plots_vector[col+3]->pushBack(PlotData::Point(linecount,tu.angle() * tu.axis()(col)));
      }
    }
    else
#endif
    {
      for(size_t col = 0; col < 6; ++col)
      {
        plots_vector[col]->pushBack(PlotData::Point(linecount,
                                    row[col].as<double>()));
      }
    }
    linecount++;
  }
  return true;
}


