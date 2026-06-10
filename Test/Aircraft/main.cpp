//
// Created by charles on 2026/3/30.
//

#include <iostream>
#include "Aircraft/AircraftModel.h"


int main(){
    constexpr double step = 0.005;
    // Eigen::Vector3d m_aircraftLLA = {120, 40, 10 * 1000};
    const Eigen::Vector3d m_aircraftLLA = {120, 40, 18000};
    ModelDevelop::Aircraft::AircraftModel m_aircraft;
    m_aircraft.init(step, m_aircraftLLA);
    m_aircraft.take_off(0, 0);
    for (int i = 0; i < 570 / 0.005; i++){
        const double ret = m_aircraft.update();
        //TODO 下降

        // if (i == 500){
        //     ModelDevelop::Aircraft::UpAndDown_Para l_para{3000};
        //     auto l_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::UpAndDown_Para>(l_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::UpAndDown, l_para_str);
        // }
        //TODO 上升

        // if (i == 500){
        //     ModelDevelop::Aircraft::UpAndDown_Para l_para{10050};
        //     auto l_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::UpAndDown_Para>(l_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::UpAndDown, l_para_str);
        // }
        //TODO 右L

        // if (i == 500){
        //     ModelDevelop::Aircraft::L_Para l_para = {ModelDevelop::Aircraft::TurnDirection::Right};
        //     auto l_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::L_Para>(l_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::L, l_para_str);
        // }
        //TODO 圆周

        // if (i == 500){
        //     ModelDevelop::Aircraft::Circle_Para circle_para = {ModelDevelop::Aircraft::TurnDirection::Left,1.5};
        //     auto circle_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::Circle_Para>(circle_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::Circle, circle_para_str);
        //TODO 右L

        // if (i == 500){
        //     ModelDevelop::Aircraft::L_Para l_para = {ModelDevelop::Aircraft::TurnDirection::Right};
        //     auto l_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::L_Para>(l_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::L, l_para_str);
        // }
        //TODO 水平S

        // if (i == 500){
        //     ModelDevelop::Aircraft::SLevel_Para sLevel_para = {ModelDevelop::Aircraft::TurnDirection::Left};
        //     auto sLevel_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::SLevel_Para>(sLevel_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::SLevel, sLevel_para_str);
        //TODO 圆周

        // if (i == 500){
        //     ModelDevelop::Aircraft::Circle_Para circle_para = {ModelDevelop::Aircraft::TurnDirection::Left,1.5};
        //     auto circle_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::Circle_Para>(circle_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::Circle, circle_para_str);
        // }
        //TODO 垂直S

        // if (i == 500){
        //     ModelDevelop::Aircraft::SVertical_Para sVertial_para={50.0};
        //     auto sVertial_para_str =ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::SVertical_Para>(sVertial_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::SVertical, sVertial_para_str);
        // }
        //TODO 筋斗

        // if (i == 500){
        //     // ModelDevelop::Aircraft::SVertical_Para sVertial_para = {50.0};
        //     // auto sVertial_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<
        //     //     ModelDevelop::Aircraft::SVertical_Para>(sVertial_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::Somersault, "");
        // }
        //TODO 水平S

        // if (i == 500){
        //     ModelDevelop::Aircraft::SLevel_Para sLevel_para = {ModelDevelop::Aircraft::TurnDirection::Left};
        //     auto sLevel_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::SLevel_Para>(sLevel_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::SLevel, sLevel_para_str);
        // }
        //TODO 垂直S

        // if (i == 500){
        //     ModelDevelop::Aircraft::SVertical_Para sVertial_para={50.0};
        //     auto sVertial_para_str =ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::SVertical_Para>(sVertial_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::SVertical, sVertial_para_str);
        // }
        //TODO 筋斗

        // if (i == 500){
        //     ModelDevelop::Aircraft::Somersault_Para somersaultPara{10.0};
        //     auto somersaultPara_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<
        //         ModelDevelop::Aircraft::Somersault_Para>(somersaultPara);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::Somersault, somersaultPara_str);
        // }
        //TODO 斜半滚倒转

        if (i == 10000){
            // ModelDevelop::Aircraft::Somersault_Para somersaultPara{10.0};
            // auto somersaultPara_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<
            // ModelDevelop::Aircraft::Somersault_Para>(somersaultPara);
            m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::SplitS, "");
        }
        if (ret > 0){
            std::cout << "end Time:" << m_aircraft.flyTime() << std::endl;
            break;
        }
    }
    std::cout << "end Time:" << m_aircraft.flyTime() << std::endl;
    return 0;
}
