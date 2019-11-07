#include <log2plot/config_manager.h>
#include <iostream>

int main()
{
  std::string parser_path(LOG2PLOT_EXAMPLE_PATH);
  parser_path += "/parser";
  log2plot::ConfigManager config(parser_path + "/config.yaml");

  // explicit specialization allows 1-line parsing
  std::cout << config.read<std::string>("dataPath") << "\n\n";

  // implicit if variable already exists
  int n;
  config.read({"1", "n"}, n);
  std::cout << "n is " << n << "\n\n";

  auto pi = config.read<double>("pi");
  std::cout << "pi is exactly " << pi << "\n\n";

  // let us read a vector with some PI's
  std::vector<std::string> keys{"1", "p1"};
  auto v = config.read<std::vector<double>>(keys);
  for(auto val: v)
    std::cout << val << " ";
  std::cout << "\n\n";

#ifdef WITH_VISP
  // 4x4 matrix defined by translation + theta-u
  auto cMo = config.read<vpHomogeneousMatrix>({"visp", "cMo"});
  std::cout << cMo << "\n\n";

  // explicit 3x3 rotation matrix (rotationness is not checked)
  vpRotationMatrix R;
  config.read<vpRotationMatrix>({"visp","R"}, R);
  std::cout << R << "\n\n";  

#else
  auto R = config.read<std::vector<std::vector<double>>>({"visp","R"});
#endif
const bool pure_Z_rotation = fabs(R[2][2] - 1) < 1e-5;

  config.setDirName(parser_path);
  config.addNameElement(config.read<std::string>("exp"));
  config.addNameElement("n", n);
  config.addConditionalNameElement("Rz", pure_Z_rotation, "");
  config.saveConfig();

}
