#include <log2plot/logger.h>

struct VibratingString
{
  static const int points;
  static const double scale;
  double n, w, amp;
  inline auto x(double y, double t) const
  {
    return amp*(sin(scale*n*y-w*t)+sin(scale*n*y+w*t))/2.;
  }
};

const int VibratingString::points = 100;
const double VibratingString::scale = M_PI/(0.1*(points-1));

int main()
{
  std::vector<double> xy1(2*VibratingString::points);
  auto xy2{xy1};
  double t{};

  log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);
  logger.setTime(t);

  logger.regroupNext(2);
  logger.saveTimedXY(xy1, "timed_xy1", "[String1]");
  logger.setLineType("[C0]");
  logger.saveTimedXY(xy2, "timed_xy2", "[String2]");
  logger.setLineType("[C1]");
  logger.setPlotArgs("-v 1 --legendLoc 1 --equal");

  const auto dy{.1};

  const VibratingString s1{2, .2, 2};
  const VibratingString s2{3, .3, 1};

  const auto dt{.2};

  while(t < 20)
  {
    for(int point = 0; point < VibratingString::points; ++point)
    {
      const auto y{-dy*point};
      xy1[2*point] = s1.x(y, t);
      xy2[2*point] = s2.x(y, t);
      xy1[2*point+1] = xy2[2*point+1] = y;
    }
    logger.update();
    t += dt;
  }
  logger.plot();

}
