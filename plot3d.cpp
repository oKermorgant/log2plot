#include <log2plot/plot/matplotlibcpp.h>
#include <iostream>
#include <thread>
#include <memory>

using namespace std::chrono_literals;

int main()
{
  std::vector<double> x(100, 0);
  std::vector<double> y(100, 0);
  std::vector<double> z(100, 0), z2(100);

  for(int i = 0; i < 100; ++i)
  {
    z[i] = 0.005*i;
    z2[i] = 0.8*z[i];
  }

  std::vector<double> x2(100, std::nan("")), y2(100, std::nan(""));
  //td::vector<double> y2;

  matplotlibcpp::ion();

  auto p3 = std::make_unique<matplotlibcpp::Plot3D>("spiral", x, y, z, "C0");
  matplotlibcpp::legend({{"loc", "upper right"}});
  //matplotlibcpp::pause(0);
  auto p2 = matplotlibcpp::Plot3D("spiral", x2, y2, z2, "C1d",1);
  matplotlibcpp::legend({{"loc", "upper right"}});
  //matplotlibcpp::pause(0.001);
  double start(0);
  int iter(0);
  while(true)
  {
    start += 0.15;

   /* if(iter < 100)
    {
      const double angle(start + 0.2*iter);
      x2[iter] = 0.001*(iter+1)*cos(angle - 0.3);
      y2[iter] = 0.001*(iter+1)*sin(angle - 0.3);
      //z2.push_back(0.9*z[iter]);
    }*/
    iter++;

    for(int i = 0; i < 100; ++i)
    {
      const double angle(start + 0.2*i);
      x[i] = 0.001*(i+1)*cos(angle);
      y[i] = 0.001*(i+1)*sin(angle);
      if(i <= iter)
      {
        x2[i] = 0.001*(i+1)*cos(angle-0.3);
        y2[i] = 0.001*(i+1)*sin(angle-0.3);
      }
    }
 //   if(iter > 10)
 //     p2.update(x2, y2, z2);

    /*
    if(iter == 50)
    {
    //  p2.remove();p2.update(x2, y2, z2);
      p2.clear();
    }*/
    //p3.update(x, y, z);

    //

    matplotlibcpp::pause(0.001);
  }

  matplotlibcpp::pause(0);
}
