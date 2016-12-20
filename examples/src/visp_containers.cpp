#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpExponentialMap.h>
#include <log2plot/logger.h>

int main()
{

    vpHomogeneousMatrix M;
    vpPoseVector pose;

    vpColVector v(6);

    log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);

    // save velocity with Latex legend
    logger.save(v, "visp_velocity", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Camera velocity");

    // save pose as iteration-based with Latex legend
    logger.save(pose, "visp_pose", "[x,y,z,\\theta_x,\\theta_y,\\theta_z]", "Camera pose");

    // save pose as 3D plot
    logger.save3Dpose(pose, "visp_pose3D", "Camera pose");
    // add a camera along the trajectory
    logger.showMovingCamera();

    for(int i=0;i<1000;++i)
    {
        // update velocity
        v[0] = cos(i*0.01);
        v[1] = sin(i*0.01);
        v[2] = 1;
        v[3] = 0;
        v[4] = 0;
        v[5] = 0.01;

        // update pose
        M = vpExponentialMap::direct(v) * M;
        // update pose vector
        pose.buildFrom(M.inverse());

        // log
        logger.update();
    }

    logger.plot();

}
