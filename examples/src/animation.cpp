#include <log2plot/log_plotter.h>
#include <math.h>
#include <chrono>

int main()
{
  // target loop time may be possible or not depending on how many dynamic plots
  float dt(0.1);
  int dt_ms(1000*dt);

  log2plot::LogPlotter logger(LOG2PLOT_EXAMPLE_PATH, dt);

  // time variable
  double t = 0;
  // register time variable with time unit (default to seconds)
  logger.setTime(t, "s");

  // save some integers
  std::vector<int> v_int(4,0);
  // saved with iteration X-axis with implicit legend
  //logger.save(v_int, "std_i", "i_", "Value of I");

  // save some doubles
  std::vector<double> v_double(6,0);
  // saved with timed X-axis with explicit legend in Latex math
  logger.saveTimed(v_double, "std_v", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Value of V");
  // set some units
  logger.setUnits("[m/s,m/s,m/s,rad/s,rad/s,rad/s]");
  logger.setLineType("[C0, r., b--, gD, c, y]");

  // save a 3D pose - will not be plotted during runtime
  std::vector<double> pose(6,0);
  // 3D pose is saved as translation + theta-u parametrization
  logger.save3Dpose(pose, "std_pose", "Trajectory");
  const auto cam{log2plot::Camera("g")};
  logger.showMovingObject(cam.nodes, cam.graph);
  logger.setLineType("[r,g]");

  double time(0);

  for(unsigned int c = 0;c<100;++c)
  {
    const auto start = std::chrono::system_clock::now();
    t += 0.05;

    // some saw teeth
    for(uint i=0;i<v_int.size();++i)
      v_int[i] = c % (20*i+2);

    // some cosines
    for(uint i=0;i<v_double.size();++i)
      v_double[i] = cos(t + i);


    // the pose is a spiral
    pose[0] = (100-c)*cos(5*t)/2;
    pose[1] = (100-c)*sin(5*t)/2;
    pose[2] = 120-c;
    pose[4] = cos(t);
    pose[5] = t;

    logger.update();
    std::this_thread::sleep_until(start + std::chrono::milliseconds(dt_ms));

    if(c > 3)
    {
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = end-start;
      time += diff.count();
      std::cout << "Mean period: " << 1000*time/(c-3) << " ms" << std::endl;
    }
  }

  // default script path + verbose
  logger.plot(false, false);
}
