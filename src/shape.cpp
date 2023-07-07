#include <log2plot/shape.h>

using namespace log2plot;

// 3D plot: build 3D box nodes
std::vector< std::array<double,3> > buildBoxNodes(const double &xm, const double &ym, const double &zm, const double &xM, const double &yM, const double &zM)
{
  std::vector< std::array<double,3> > M(8, std::array<double,3>{});
  M[0][0] = xm;    M[0][1] = ym;   M[0][2] = zm;
  M[1][0] = xM;     M[1][1] = ym;   M[1][2] = zm;
  M[2][0] = xM;     M[2][1] = yM;    M[2][2] = zm;
  M[3][0] = xm;    M[3][1] = yM;    M[3][2] = zm;
  M[4][0] = xm;    M[4][1] = ym;   M[4][2] = zM;
  M[5][0] = xM;     M[5][1] = ym;   M[5][2] = zM;
  M[6][0] = xM;     M[6][1] = yM;    M[6][2] = zM;
  M[7][0] = xm;    M[7][1] = yM;    M[7][2] = zM;
  return M;
}

Infos Surface::yaml(const std::string &type, const std::vector<double> &params) const
{
  Infos ret{Yaml("type", type, 2), Yaml("alpha", std::to_string(color_alpha), 2)};
  if(!params.empty())
    ret.emplace_back("params", params, 2);
  return ret;
}

Infos Shape::infos() const
{
  if(nodes.empty())
    return {};

  Infos infos{Yaml("nodes", nodes, 1),
        Yaml("graph", graph, 1),
        Yaml("color", color, 1),
        Yaml("legend", legend, 1)};

  if(scale_pc > 0 && scale_pc < 1)
    infos.push_back(Yaml("scale", std::to_string(scale_pc), 1));
  if(pose.size() == 6)
    infos.push_back(Yaml("pose", pose, 1));

  return infos;
}

constexpr inline auto xCam{1.5};
constexpr inline auto yCam{1.};
constexpr inline auto zCam{4.};

Camera::Camera(const std::string &color, const std::string &legend, double scale)
  : Shape({{0,0,0},{xCam,-yCam,zCam},{-xCam,-yCam,zCam},{-xCam,yCam,zCam},{xCam,yCam,zCam}},
{{3,0,4,3,2,0,1,4,1,2}}, color, legend)
{
  scale_pc = scale;
}

Box::Box(double xm, double ym, double zm,
         double xM, double yM, double zM,
         const std::string &color, const std::string &legend)
  : Shape(buildBoxNodes(xm,ym,zm,xM,yM,zM),
{{0,1},{1,2},{2,3},{3,0},{0,4},{1,5},{2,6},{3,7},{4,5},{5,6},{6,7},{7,4}},
          color, legend)
{}

Frame::Frame(double scale, const std::vector<double> &pose)
  : Shape({{0,0,0},{1,0,0},{0,1,0},{0,0,1}},{{0,1},{0,2},{0,3}}, "[r,g,b]", "")
{
  scale_pc = scale;
  if(pose.size() == 6)
    this->pose = pose;
}

