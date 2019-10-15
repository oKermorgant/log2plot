#include <log2plot/logger.h>
#include <math.h>

int main()
{

  log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);

  // time variable
  double t = 0;
  // register time variable with time unit (default to seconds)
  logger.setTime(t, "s");

  // save some integers
  std::vector<int> v_int(4,0);
  // saved with iteration X-axis with implicit legend
  logger.save(v_int, "std_i", "i_", "Value of I");
  // also set custome line types in matplotlib style
  logger.setLineType("[k-, r., b--, gD]");

  // save some doubles
  std::vector<double> v_double(6,0);
  // saved with timed X-axis with explicit legend in Latex math
  logger.saveTimed(v_double, "std_v", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Value of V");
  // set some units
  logger.setUnits("[m/s,m/s,m/s,rad/s,rad/s,rad/s]");
  // also add explicit steps to be plotted
  logger.setSteps({0.5, 1.2, 4});

  // plot some doubles as X-Y
  std::vector<double> xy(4, 0);
  logger.saveXY(xy, "std_xy", "[traj_1, traj_2]", "x-position", "y-position");

  // save a 3D pose
  std::vector<double> pose(6,0);
  // 3D pose is saved as translation + theta-u parametrization
  logger.save3Dpose(pose, "std_pose", "Trajectory");

  // add a fixed box from (-1,-2,-3) to (1,2,3) in cyan
  logger.showFixedBox(-1,-2,-3,1,2,3, "c");

  // add a moving camera with its desired pose
  logger.showMovingCamera({0,0,20,0,M_PI/2,0});

  /* for moving objects, lineType define:
        - trajectory color
        - intermediary poses
        - initial and final poses
        - desired pose

        the command below will plot:
        - red trajectory
        - green intermediary cameras
        - blue initial and final poses
        - black dashed desired pose
      */
  logger.setLineType("[r,g,b,k--]");

  for(unsigned int c = 0;c<100;++c)
  {
    t += 0.05;
    // some cosines
    for(uint i=0;i<v_double.size();++i)
      v_double[i] = cos(t + i);

    xy[0] = cos(t);
    xy[1] = sin(2*t);
    xy[2] = 2*sin(3*t);
    xy[3] = 3*cos(3*t);

    // some saw teeth
    for(uint i=0;i<v_int.size();++i)
      v_int[i] = c % (20*i+2);

    // the pose is a spiral
    pose[0] = (100-c)*cos(5*t)/2;
    pose[1] = (100-c)*sin(5*t)/2;
    pose[2] = 120-c;
    pose[5] = t;

    // say some component are non-defined for a period of time
    if(t > 0.5 && t < 1.2)
    {
      log2plot::setNaN(v_double, 0, 3);
      pose[0] = log2plot::nan;
    }

    logger.update();
  }

  // default script path + verbose
  logger.plot(true);

}
