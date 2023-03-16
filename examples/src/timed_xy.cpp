#include <log2plot/logger.h>

struct VibratingString
{
  static const int points;
  static const double scale;
  static constexpr auto dy{.1};
  double n, w, amp;

  VibratingString(double n, double w ,double amp) : n{n}, w{w}, amp{amp}
  {
    xy.resize(2*points);
    for(int point = 0; point < VibratingString::points; ++point)
      xy[2*point+1] = -dy*point;
  }

  inline void update(double t)
  {
    for(int point = 0; point < VibratingString::points; ++point)
    {
      const auto y{xy[2*point+1]};
      xy[2*point] = amp*(sin(scale*n*y-w*t)+sin(scale*n*y+w*t))/2;
    }
  }
  std::vector<double> xy;
};

const int VibratingString::points = 100;
const double VibratingString::scale = M_PI/(0.1*(points-1));

int main()
{
  VibratingString s1{2, .2, 2};
  VibratingString s2{3, .3, 1};

  double t{};

  log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);
  logger.setTime(t);

  logger.regroupNext(2);
  logger.saveTimedXY(s1.xy, "timed_xy1", "[w=2]");
  logger.setLineType("[C0]");
  logger.saveTimedXY(s2.xy, "timed_xy2", "[w=3]");
  logger.setLineType("[C1]");
  logger.setPlotArgs("-v 1 --legendLoc out --equal");


  const auto dt{.2};

  while(t < 20)
  {
    s1.update(t);
    s2.update(t);

    logger.update();
    t += dt;
  }
  logger.plot();

}
