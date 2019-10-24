#include <Eigen/Core>
#include <log2plot/logger.h>

int main()
{
    Eigen::Vector3d t;


    log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);

    // save translation with Latex legend
    logger.save(t, "eigen_t", "[t_x,t_y,t_z]", "Position");

    for(int i=0;i<100;++i)
    {
       t[0] = cos(i*0.01);
       t[1] = sin(i*0.01);
       t[2] = 0.01*i;

        // log
        logger.update();

        if(i % 20 == 0)
          logger.writeStep();
    }

    // default script path + verbose
    logger.plot(true);

}
