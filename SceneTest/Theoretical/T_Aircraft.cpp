//
// Created by charles on 2026/3/30.
//

#include <iostream>
#include "Aircraft/AircraftModel.h"
#include "include/TheoreticalModel/TheoreticalModelMissile.h"



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
    Eigen::Vector3d m_aircraftLLA = {120, 39, 3000};
    // Eigen::Vector3d pac2LLA = {120, 41, 2};//拦不到
    Eigen::Vector3d missileLLA = {120, 40.75, 3000};//拦截到了
    // Eigen::Vector3d pac2LLA = {120, 40.5, 2};//拦截到了
    //飞机目标 初始化
    ModelDevelop::Aircraft::AircraftModel m_aircraft;
    m_aircraft.init(step, m_aircraftLLA);
    m_aircraft.take_off(0, 0);
    //PAC2 初始化
    ModelDevelop::TheoreticalModel::Missile missile;
    missile.init(step, missileLLA);
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(m_aircraftLLA);
    missile.setTargetEcf(targetPosEcf, {0, 0, 0},true);
    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(
        ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, missileLLA.x(), missileLLA.y())) * 57.3;
    const auto targetTheta = ModelDevelop::Utils::CoordinateHelper::getTheta(
        ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, missileLLA.x(), missileLLA.y())) * 57.3;
    missile.launch(0, targetPsi);
    bool is_target_one_L = false;
    for (int i = 0; i < 200/ 0.005; i++){
        m_aircraft.update();
        auto m_aircraftVelEcf = ModelDevelop::Utils::CoordinateHelper::nueToEcefVelocity(
            m_aircraft.velocityNUE(), m_aircraft.lla().x(), m_aircraft.lla().y());
        missile.setTargetLLA(m_aircraft.lla(), m_aircraftVelEcf,false);
        double ret = missile.update();
        // if (missile.targetDis() < 15000 && !is_target_one_L){
        //     ModelDevelop::Aircraft::L_Para l_para = {ModelDevelop::Aircraft::TurnDirection::Left};
        //     auto l_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::L_Para>(l_para);
        //     m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::L, l_para_str);
        //     is_target_one_L = true;
        // }
        if (ret > 0){
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    std::cout << "end Time:" << m_aircraft.flyTime()  << "terminal_dis:" << missile.targetDis()<< std::endl;
    return 0;
}
