#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpExponentialMap.h>
#include <log2plot/logger.h>
#include <log2plot/dir_tools.h>

int main()
{

  // pose of a camera expressed as world pose in camera frame
  vpHomogeneousMatrix M(0,0,0.1,M_PI/2,0,0);
  vpPoseVector pose;

  // camera velocity in its own frame
  vpColVector v(6);

  log2plot::Logger logger(LOG2PLOT_EXAMPLE_PATH);

  // save velocity with Latex legend
  logger.save(v, "visp_velocity", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Camera velocity");

  // save pose as iteration-based with Latex legend
  logger.save(pose, "visp_pose", "[x,y,z,\\theta_x,\\theta_y,\\theta_z]", "Camera pose");

  // save pose as 3D plot
  // the saved variable is the world pose in camera frame, we want to plot the invert
  logger.save3Dpose(pose, "visp_pose3D", "Camera pose", true);
  // add a camera along the trajectory
  logger.showMovingCamera();
  // add a green rectangle from (-.1,-.2,0) to (.1,.2,0) in world frame
  logger.showFixedRectangle(-0.1,-0.2,0.1,0.2,"g");

  for(int i=0;i<100;++i)
  {
    // update velocity in camera frame
    v[0] = 0.01*(cos(0.1*i)+1);
    v[1] = -0.1*(cos(0.1*i)+1);
    v[2] = 0;
    v[3] = 0;
    v[4] = -0.1*(cos(0.1*i)+1);
    v[5] = 0;

    // update pose
    M = vpExponentialMap::direct(v) * M;
    // update pose vector
    pose.buildFrom(M);

    // log
    logger.update();
  }

  // default script path + verbose
  logger.plot(true);

}
