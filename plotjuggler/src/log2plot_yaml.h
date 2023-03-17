#pragma once

#include <QObject>
#include <QtPlugin>
#include <PlotJuggler/dataloader_base.h>
#include <yaml-cpp/node/node.h>

using namespace PJ;

struct RelevantFields
{
  YAML::Node legend;
  YAML::Node data;
  std::string datatype;
  PlotDataMapRef& plot_data;
  explicit RelevantFields(PlotDataMapRef& plot_data) : plot_data{plot_data} {}

  auto title(size_t col) const
  {
    return legend[col].as<std::string>();
  }
};

class DataLoadYAML : public DataLoader
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.DataLoader")
  Q_INTERFACES(PJ::DataLoader)

public:
  DataLoadYAML() = default;
  virtual const std::vector<const char*>& compatibleFileExtensions() const override
  {
    return _extensions;
  }

  bool readDataFromFile(PJ::FileLoadInfo* fileload_info,
                        PlotDataMapRef& destination) override;

  ~DataLoadYAML() override = default;

  virtual const char* name() const override
  {
    return "log2plot YAML";
  }


protected:

  bool readBasic(const RelevantFields &fields);
  bool readXY(const RelevantFields &fields);
  bool readPose3D(const RelevantFields &fields);

private:
  std::vector<const char*> _extensions = {"yml", "yaml"};

  std::string _default_time_axis;
};


