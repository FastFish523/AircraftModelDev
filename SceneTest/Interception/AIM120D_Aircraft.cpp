//
// Created by charles on 2026/3/30.
//

#include <iostream>
#include "Aircraft/AircraftModel.h"
#include "include/AIM120D/AIM120DMissile.h"


int main(){
    /*!
     *纬度：每差 1° ≈ 111 公里（基本不变）
     *经度：每差 1° ≈ 111 × cos (纬度) 公里（越靠近两极越短）
     *经度：赤道：经纬度都 ≈ 111 km/°
     *经度：北纬 30°/ 南纬 30°：经度 ≈ 96 km/°
     *经度：北纬 45°/ 南纬 45°：经度 ≈ 79 km/°
     *经度：北纬 60°/ 南纬 60°：经度 ≈ 55 km/°
     */
    double step = 0.005;
    Eigen::Vector3d m_aircraftLLA = {118.5,40,18000};
    Eigen::Vector3d missileLLA = {120,40,18500};
    //飞机目标 初始化
    ModelDevelop::Aircraft::AircraftModel m_aircraft;
    m_aircraft.init(step, m_aircraftLLA);
    m_aircraft.take_off(0, 5);
    //missile 初始化
    ModelDevelop::AIM120D::Missile missile;
    missile.init(step, missileLLA);
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(m_aircraftLLA);
    missile.setTargetEcf(targetPosEcf, {0, 0, 0},true);
    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(
        ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, missileLLA.x(), missileLLA.y())) * 57.3;
    const auto targetTheta = ModelDevelop::Utils::CoordinateHelper::getTheta(
        ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, missileLLA.x(), missileLLA.y())) * 57.3;
    missile.launch(targetTheta, targetPsi);
    bool is_target_one_L = false;
    std::cout << "_dis:" << missile.targetDis() << std::endl;
    for (int i = 0; i < 1000 / 0.005; i++){
        m_aircraft.update();
        auto m_aircraftVelEcf = ModelDevelop::Utils::CoordinateHelper::nueToEcefVelocity(
            m_aircraft.velocityNUE(), m_aircraft.lla().x(), m_aircraft.lla().y());
        missile.setTargetLLA(m_aircraft.lla(), m_aircraftVelEcf,false);
        double ret = missile.update();
        if (missile.targetDis() < 40000 && !is_target_one_L){
            // ModelDevelop::Aircraft::L_Para l_para = {ModelDevelop::Aircraft::TurnDirection::Left};
            // auto l_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::L_Para>(l_para);
            // m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::L, l_para_str);
            ModelDevelop::Aircraft::SVertical_Para sVertial_para={100.0};
            auto sVertial_para_str =ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::SVertical_Para>(sVertial_para);
            m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::SVertical, sVertial_para_str);
            is_target_one_L = true;
        }
        if (ret > 0){
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    std::cout << "end Time:" << m_aircraft.flyTime() << std::endl;
    return 0;
}
