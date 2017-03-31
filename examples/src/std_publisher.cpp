#include <log2plot/log_publisher.h>
#include <math.h>

int main(int argc, char**argv)
{
    ros::init(argc, argv, "iteration_publisher");



    // to be published, logged variables have to be able to be cast to double
    log2plot::LogPublisher logger(LOG2PLOT_EXAMPLE_PATH);
    double t = 0;
    logger.setTime(t, "s");


    // i has implicit legend and is saved with iteration X-axis
    std::vector<int> v_int(4,0);
    logger.save(v_int, "std_i", "i_", "Value of I");

    // v has explicit legend in Latex math and is saved with timed X-axis
    std::vector<double> v_double(6,0);
    logger.saveTimed(v_double, "std_v", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Value of V");


    double T = 0.1;
    ros::Rate loop(1/T);
    double t0 = ros::Time::now().toSec();
    while(ros::ok())
    {
        // current time
        t = ros::Time::now().toSec() - t0;

        // some cosines
        for(int i=0;i<v_double.size();++i)
            v_double[i] = cos(t + i);

        // some saw teeth
        for(int i=0;i<v_int.size();++i)
            v_int[i] = int(t/T) % (20*i+2);

        logger.update();
        ros::spinOnce();
        loop.sleep();
    }

    // default script path + verbose
   logger.plot("", true);
}
