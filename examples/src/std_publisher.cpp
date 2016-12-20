#include <log2plot/log_publisher.h>
#include <math.h>

int main(int argc, char**argv)
{
    ros::init(argc, argv, "iteration_publisher");
    std::vector<double> v_double(6,0);
    std::vector<int> v_int(10,0);

    log2plot::LogPublisher logger(LOG2PLOT_EXAMPLE_PATH);

    // i has implicit legend and is saved with iteration X-axis
    logger.save(v_int, "std_i", "i_", "Value of I");

    // v has explicit legend in Latex math and is saved with timed X-axis
    logger.saveTimed(v_double, "std_v", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Value of V");

    ros::Rate loop(10);
    double t = 0;
    int c = 0;
    while(c < 100)
    {
        // some cosines
        for(auto &e: v_double)
            e = cos(++t);

        // some proportional
        for(int i=0;i<v_int.size();++i)
            v_int[i] = c * (i-4);
        c++;

        logger.update();
        ros::spinOnce();
        loop.sleep();
    }

    logger.plot();
}
