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

int main()
{
  log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);

  std::vector<double> dummy(6, log2plot::nan);
  logger.save3Dpose(dummy, "static_3d", "Points");
  logger.setLineType("[C0d]");

  std::vector<Point> cloud(40);

  std::generate(cloud.begin(), cloud.end(), randomPoint);
  logger.showFixedObject(cloud, "[]", "C0d", "a discrete object");


  cloud.resize(10);
  std::generate(cloud.begin(), cloud.end(), randomPoint);
  logger.showFixedObject(cloud, log2plot::legendFullyConnected(cloud.size()), "C1");

  logger.showFixedBox(1, 1, 1, 3, 4, 5, "C2", "a box");

  logger.plot();
}
