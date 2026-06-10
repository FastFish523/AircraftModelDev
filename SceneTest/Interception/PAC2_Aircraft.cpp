//
// Created by charles on 2026/3/30.
//

#include <iostream>
#include "Aircraft/AircraftModel.h"
#include "include/PAC500/Missile.h"


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
    Eigen::Vector3d m_aircraftLLA = {120, 41, 10 * 1000};
    Eigen::Vector3d PAC500LLA = {120, 40, 2};
    //飞机目标 初始化
    ModelDevelop::Aircraft::AircraftModel m_aircraft;
    m_aircraft.init(step, m_aircraftLLA);
    m_aircraft.take_off(0, 0);
    //PAC500 初始化
    ModelDevelop::PAC500::Missile PAC500;
    PAC500.init(step, PAC500LLA);
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(m_aircraftLLA);
    PAC500.setTargetEcf(targetPosEcf, {0, 0, 0},true);
    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(
        ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, PAC500LLA.x(), PAC500LLA.y())) * 57.3;
    const auto targetTheta = ModelDevelop::Utils::CoordinateHelper::getTheta(
        ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, PAC500LLA.x(), PAC500LLA.y())) * 57.3;
    PAC500.launch(targetTheta, targetPsi);
    bool is_target_one_L = false;
    std::cout << "_dis:" <<PAC500.targetDis() << std::endl;
    for (int i = 0; i < 1000 / 0.005; i++){
        m_aircraft.update();
        auto m_aircraftVelEcf = ModelDevelop::Utils::CoordinateHelper::nueToEcefVelocity(
            m_aircraft.velocityNUE(), m_aircraft.lla().x(), m_aircraft.lla().y());
        PAC500.setTargetLLA(m_aircraft.lla(), m_aircraftVelEcf,false);
        double ret = PAC500.update();
        if (PAC500.targetDis() < 15000 && !is_target_one_L){
            ModelDevelop::Aircraft::L_Para l_para = {ModelDevelop::Aircraft::TurnDirection::Left};
            auto l_para_str = ModelDevelop::Aircraft::BaseManeuvering::setManeuveringPara<ModelDevelop::Aircraft::L_Para>(l_para);
            m_aircraft.setManeuvering(ModelDevelop::Aircraft::ManeuveringModeType::L, l_para_str);
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
