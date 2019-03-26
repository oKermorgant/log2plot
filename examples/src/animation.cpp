#include <log2plot/log_plotter.h>
#include <math.h>

int main()
{

    log2plot::LogPlotter logger(LOG2PLOT_EXAMPLE_PATH);

    // time variable
    double t = 0;
    // register time variable with time unit (default to seconds)
    logger.setTime(t, "s");

    // save some doubles
    std::vector<double> v_double(6,0);
    // saved with timed X-axis with explicit legend in Latex math
    logger.saveTimed(v_double, "std_v", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Value of V");
    // set some units
    logger.setUnits("[m/s,m/s,m/s,rad/s,rad/s,rad/s]");
    logger.setLineType("[k-, r., b--, gD, c, y]");

    // save some integers
    std::vector<int> v_int(4,0);
    // saved with iteration X-axis with implicit legend
    logger.save(v_int, "std_i", "i_", "Value of I");
    // also set custome line types in matplotlib style
    logger.setLineType("[k-, r., b--, gD]");

    // save a 3D pose - will not be plotted during runtime
    std::vector<double> pose(6,0);
    // 3D pose is saved as translation + theta-u parametrization
    logger.save3Dpose(pose, "std_pose", "Trajectory");
    // add a fixed box from (-1,-2,-3) to (1,2,3) in cyan
    logger.showFixedBox(-1,-2,-3,1,2,3, "c");


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
   // logger.setLineType("[r,g,b,k--]");

    for(unsigned int c = 0;c<100;++c)
    {
        t += 0.05;

        // some saw teeth
        for(int i=0;i<v_int.size();++i)
            v_int[i] = c % (20*i+2);

        // some cosines
        for(int i=0;i<v_double.size();++i)
            v_double[i] = cos(t + i);


        // the pose is a spiral
        pose[0] = (100-c)*cos(5*t)/2;
        pose[1] = (100-c)*sin(5*t)/2;
        pose[2] = 120-c;
        pose[5] = t;

        logger.update();
        matplotlibcpp::pause(0.01);
    }

    // default script path + verbose
   logger.plot("", true);

}
