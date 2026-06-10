//
// Created by Administrator on 2026/2/28.
//

#include <iostream>
#include "Su27/Su27Missile.h"


int main()
{
    Eigen::Vector3d missileLLA            = {120,40,20000};
    ModelDevelop::Su27::Missile missile;
    missile.init(0.005,missileLLA);

    missile.take_off(0,5);

    for(int i = 0;i<50/0.005;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
