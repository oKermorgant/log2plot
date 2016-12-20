#include <log2plot/logger.h>
#include <math.h>

int main()
{

    log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);

    std::vector<double> v_double(6,0);
    std::vector<int> v_int(10,0);

    // time variable
    double t = 0;
    logger.setTime(t, "s");

    // i has implicit legend and is saved with iteration X-axis
    logger.save(v_int, "std_i", "i_", "Value of I");

    // v has explicit legend in Latex math and is saved with timed X-axis
    logger.saveTimed(v_double, "std_v", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Value of V");

    for(unsigned int c = 0;c<30;++c)
    {
        t += 0.05;
        // some cosines
        for(int i=0;i<v_double.size();++i)
            v_double[i] = cos(t + i);

        // some proportional
        for(int i=0;i<v_int.size();++i)
            v_int[i] = c * (i-4);

        logger.update();
    }

   logger.plot();

}
