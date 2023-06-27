#include <log2plot/logger.h>
#include <random>
#include <algorithm>

using Point = std::array<double, 3>;

inline double randCoord()
{
  static std::default_random_engine engine;
  static auto gen{std::uniform_real_distribution()};
  return -5 + 10 * gen(engine);
}

inline Point randomPoint()
{
  return Point{randCoord(), randCoord(), randCoord()};
}

inline std::vector<Point> generateSphere(double rad, double x0, double y0, double z0)
{
  const auto n{100};
  std::vector<Point> sphere;
  sphere.reserve(n);

  // use golden spiral
  static const double angle_incr(4*M_PI/(1 + sqrt(5)));
  const auto n_inv(1./n);

  for(uint i = 0; i < n; ++i)
  {
    const double z(i*2*n_inv + n_inv - 1);
    const double rho(sqrt(1-z*z));
    const double angle(i*angle_incr);
    sphere.push_back({x0 + rad*rho*cos(angle),
                      y0 + rad*rho*sin(angle),
                      z0 + rad*z});
  }
  return sphere;
}

int main()
{
  log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);

  std::vector<double> dummy(3);
  logger.save3Dpose(dummy, "static_3d");
  logger.setPlotArgs("--legendCol 2");

  std::vector<Point> cloud(40);

  // some points
  std::generate(cloud.begin(), cloud.end(), randomPoint);
  logger.showFixedObject(cloud, "[]", "C0d", "a discrete object");

  // fully connected
  cloud.resize(10);
  std::generate(cloud.begin(), cloud.end(), randomPoint);
  logger.showFixedObject(cloud, log2plot::legendFullyConnected(cloud.size()), "C1");

  // builtin
  logger.showFixedBox(1, 1, 1, 3, 4, 5, "C2", "a box");

  // surface
  cloud = generateSphere(1, 4, -4, -4);
  logger.showFixedObject(cloud, "", "C4", "a sphere with alpha shape");
  logger.displayObjectAs(log2plot::AlphaShape(1., .3));

  cloud = generateSphere(1, -4, -4, -4);
  logger.showFixedObject(cloud, "", "C5", "a sphere with reconstructed surface");
  logger.displayObjectAs(log2plot::Reconstructed(1., .3));

  logger.plot();
}
