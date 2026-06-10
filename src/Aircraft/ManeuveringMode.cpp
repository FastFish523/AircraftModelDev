//
// Created by MikuSoft on 2026/3/30.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//
#include "Aircraft/ManeuveringMode.h"
#include "CoordinateHelper.h"
#include  "Util/Constants.h"

namespace ModelDevelop::Aircraft{
    TurnChecker::TurnChecker(double startYawDeg, TurnDirection dir, double turnDeg) : m_direction(dir){
        if (turnDeg <= 0) throw std::invalid_argument("所转角度必须大于0");
        m_turn = turnDeg;
        m_prev_yaw = normalize(startYawDeg);
        m_total = 0.0;
    }

    bool TurnChecker::updateAndCheck(double currentYawDeg){
        double curr = normalize(currentYawDeg);
        double delta = calcStep(curr, m_prev_yaw);
        m_total += delta;
        // std::cout << "m_prev_yaw=" << m_prev_yaw << "\tcurr=" << curr << "\tdelta=" << delta << "\tm_total=" << m_total
        //     << std::endl;
        m_prev_yaw = curr;
        return isFinished();
    }

    double TurnChecker::getAccumulated() const{ return m_total; }

    void TurnChecker::reset(double startYawDeg, TurnDirection dir, double turnDeg){
        if (turnDeg <= 0) throw std::invalid_argument("所转角度必须大于0");
        m_turn = turnDeg;
        m_direction = dir;
        m_prev_yaw = normalize(startYawDeg);
        m_total = 0.0;
    }

    double TurnChecker::normalize(double deg){
        deg = fmod(deg, 360.0); //控[-360，360]
        if (deg > 180) deg -= 360;
        if (deg < -180) deg += 360;
        //控[-180，180]
        return deg;
    }

    double TurnChecker::calcStep(double curr, double prev) const{
        double re = (m_direction == TurnDirection::Left)
                        ? ((curr >= prev) || (fabs(curr - prev) < 0.05) ? (curr - prev) : ((curr + 360) - prev)) //右
                        : ((curr <= prev) || (fabs(curr - prev) < 0.05) ? (curr - prev) : (-(360 - (curr - prev)))); //左
        return re;
    }

    bool TurnChecker::isFinished() const{
        // std::cout << "\tm_turn=" << m_turn << "\tm_total=" << m_total<<std::endl;
        return fabs(m_total) >= m_turn - 0.05;
    }

    void LevelFight::init(const State& state, std::string para){
        std::cout << "设置平飞" << std::endl;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        m_targetHeight = lla.z();
        std::cout << "target height: " << m_targetHeight << std::endl;
    }

    RESULT LevelFight::update(const State& state){
        // std::cout << "target height: " << m_targetHeight << std::endl;
        RESULT result;
        result.isCompleted = false;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        auto selfVel_nue = Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double Vy = selfVel_nue.y();
        double acc_cmd_v_y = 9.8 + 0.01 * (m_targetHeight - lla.z()) - 2 * 4 * 0.05 * Vy;
        result.acc_cmd_v = {0, acc_cmd_v_y, 0};
        return result;
    }

    void UpAndDown::init(const State& state, std::string para){
        UpAndDown_Para m_para = getManeuveringPara<UpAndDown_Para>(para);
        std::cout << "设置升降，目标高度=" << m_para.targetHeight << "m" << std::endl;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        m_targetHeight = m_para.targetHeight;
        std::cout << "current height: " << lla.z() << std::endl;
        std::cout << "target heigt: " << m_targetHeight << std::endl;
    }

    RESULT UpAndDown::update(const State& state){
        RESULT result;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        auto selfVel_nue = Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double Vy = selfVel_nue.y();
        double acc_cmd_v_y = 0.0;
        double dirt_height = m_targetHeight - lla.z();
        acc_cmd_v_y = 9.8 + 0.01 * (dirt_height) - 2 * 4 * 0.05 * Vy;
        // std::cout << "acc_cmd_v_y: " << acc_cmd_v_y << std::endl;
        acc_cmd_v_y = std::clamp(acc_cmd_v_y, -10.0 * 9.81, 10.0 * 9.81);
        if (fabs(dirt_height) < 0.5){
            result.isCompleted = true;
            std::cout << "升降阶段结束，结束信息：" << "current height= " << lla.z() << "，target height=" << m_targetHeight <<
                std::endl;
        }
        result.acc_cmd_v = {0, acc_cmd_v_y, 0};
        return result;
    }


    void L::init(const State& state, std::string para){
        L_Para m_para = getManeuveringPara<L_Para>(para);
        std::cout << "设置L型机动:" << static_cast<int>(m_para.turnDir) << std::endl;
        auto currentYaw_deg = Utils::CoordinateHelper::quaternionToEuler231(state.qbn).x();
        m_direction = m_para.turnDir;
        m_turnChecker = std::make_unique<TurnChecker>(currentYaw_deg, m_para.turnDir, 90);
        std::cout << "current yaw_deg: " << currentYaw_deg << std::endl;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        m_targetHeight = lla.z();
    }

    RESULT L::update(const State& state){
        RESULT result;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        auto selfVel_nue = Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double Vy = selfVel_nue.y();
        double rollC = 0.0; //弧度
        auto att = Utils::CoordinateHelper::quaternionToEuler231(state.qbn);
        //补偿滚转产生的升力丢失
        double roll_deg = std::clamp(att.z(), -80.0, 80.0);
        double acc_cmd_v_y = (9.8 + 0.01 * (m_targetHeight - lla.z()) - 2 * 4 * 0.05 * Vy) / std::cos(roll_deg / 57.3);
        //计算机的数学按弧度
        result.isCompleted = m_turnChecker->updateAndCheck(att.x());
        if (!result.isCompleted){
            const double e = (m_direction == TurnDirection::Right)
                                 ? -1.0 * (90.0 - fabs(m_turnChecker->getAccumulated()))
                                 : 1.0 * (90.0 - fabs(m_turnChecker->getAccumulated()));
            rollC = 2 * (-60 / (1 + exp(-1 * e)) + 30) / 57.3; //Sigmoid函数 控制在-60到60
            result.acc_cmd_v = {rollC, acc_cmd_v_y, 0};
        }
        else{
            std::cout << "L型机动结束，结束信息：" << "current yaw_deg= " << att.x() << std::endl;
            result.acc_cmd_v = {0.0, acc_cmd_v_y, 0.0};
        }
        return result;
    }

    void Circle::init(const State& state, std::string para){
        Circle_Para m_para = getManeuveringPara<Circle_Para>(para);
        m_number = m_para.numberOfTurns;
        std::cout << "设置圆周运动:" << static_cast<int>(m_para.turnDir) << "、圈数=" << m_number << std::endl;
        auto currentYaw_deg = Utils::CoordinateHelper::quaternionToEuler231(state.qbn).x();
        m_direction = m_para.turnDir;
        m_turnChecker = std::make_unique<TurnChecker>(currentYaw_deg, m_para.turnDir, 360 * m_number);
        std::cout << "current yaw_deg: " << currentYaw_deg << std::endl;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        m_targetHeight = lla.z();
    }

    RESULT Circle::update(const State& state){
        RESULT result;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        auto selfVel_nue = Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double Vy = selfVel_nue.y();
        double rollC = 0.0; //弧度
        auto att = Utils::CoordinateHelper::quaternionToEuler231(state.qbn);
        //补偿滚转产生的升力丢失
        double roll_deg = std::clamp(att.z(), -80.0, 80.0);
        double acc_cmd_v_y = (9.8 + 0.01 * (m_targetHeight - lla.z()) - 2 * 4 * 0.05 * Vy) / std::cos(roll_deg / 57.3);
        //计算机的数学按弧度
        result.isCompleted = m_turnChecker->updateAndCheck(att.x());
        if (!result.isCompleted){
            const double e = (m_direction == TurnDirection::Right)
                                 ? -1.0 * (360.0 * m_number - fabs(m_turnChecker->getAccumulated()))
                                 : 1.0 * (360.0 * m_number - fabs(m_turnChecker->getAccumulated()));
            // std::cout<<"存在滚转角，e="<<e<<std::endl;
            rollC = 2 * (-60 / (1 + exp(-1 * e)) + 30) / 57.3; //Sigmoid函数 控制在-60到60
            result.acc_cmd_v = {rollC, acc_cmd_v_y, 0};
        }
        else{
            std::cout << "圆周运动结束，结束信息：" << "current yaw_deg= " << att.x() << std::endl;
            result.acc_cmd_v = {0.0, acc_cmd_v_y, 0.0};
        }
        return result;
    }

    void SLevel::init(const State& state, std::string para){
        SLevel_Para m_para = getManeuveringPara<SLevel_Para>(para);
        std::cout << "设置水平S曲线:" << static_cast<int>(m_para.turnDir) << std::endl;
        auto currentYaw_deg = Utils::CoordinateHelper::quaternionToEuler231(state.qbn).x();
        m_direction = m_para.turnDir;
        m_turnChecker = std::make_unique<TurnChecker>(currentYaw_deg, m_direction, 90);
        std::cout << "current yaw_deg: " << currentYaw_deg << std::endl;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        m_targetHeight = lla.z();
    }

    RESULT SLevel::update(const State& state){
        RESULT result;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        auto selfVel_nue = Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double Vy = selfVel_nue.y();
        double rollC = 0.0; //弧度
        auto att = Utils::CoordinateHelper::quaternionToEuler231(state.qbn);
        //补偿滚转产生的升力丢失
        double roll_deg = std::clamp(att.z(), -80.0, 80.0);
        double acc_cmd_v_y = (9.8 + 0.01 * (m_targetHeight - lla.z()) - 2 * 4 * 0.05 * Vy) / std::cos(roll_deg / 57.3);
        //计算机的数学按弧度
        result.isCompleted = false;
        bool completeOneBeat = m_turnChecker->updateAndCheck(att.x());
        if (completeOneBeat){
            m_direction = (m_direction == TurnDirection::Right) ? TurnDirection::Left : TurnDirection::Right;
            m_turnChecker->reset(att.x(), m_direction, turnDeg);
            isFirst = false;
        }
        if (isFirst){
            const double e = (m_direction == TurnDirection::Right)
                                 ? -1.0 * (90 - fabs(m_turnChecker->getAccumulated()))
                                 : 1.0 * (90 - fabs(m_turnChecker->getAccumulated()));
            // std::cout<<"存在滚转角，e="<<e<<std::endl;
            rollC = 2 * (-60 / (1 + exp(-1 * e)) + 30) / 57.3; //Sigmoid函数 控制在-60到60
        }
        else{
            const double e = (m_direction == TurnDirection::Right)
                                 ? -1.0 * (turnDeg - fabs(m_turnChecker->getAccumulated()))
                                 : 1.0 * (turnDeg - fabs(m_turnChecker->getAccumulated()));
            // std::cout<<"存在滚转角，e="<<e<<std::endl;
            rollC = 2 * (-60 / (1 + exp(-1 * e)) + 30) / 57.3; //Sigmoid函数 控制在-60到60
        }
        result.acc_cmd_v = {rollC, acc_cmd_v_y, 0};
        return result;
    }

    void SVertical::init(const State& state, std::string para){
        std::cout << "设置垂直S" << std::endl;
        SVertical_Para m_para = getManeuveringPara<SVertical_Para>(para);
        m_minDurationTime = m_para.minDurationTime;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        m_initialHeight = lla.z();
        std::cout << "minDurationTime:" << m_minDurationTime << " initial height: " << m_initialHeight << std::endl;
    }

    RESULT SVertical::update(const State& state){
        m_durationTime += 0.005;
        RESULT result;
        result.isCompleted = false;
        Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        auto selfVel_nue = Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double Vy = selfVel_nue.y();
        double acc_cmd_v_y = 0;
        if (m_durationTime < m_minDurationTime){
            //正弦波y(t) = A * sin(2πf t + φ)
            const double dirt_height =
                500 * sin(m_durationTime * 0.04 * Utils::Constants::PI + Utils::Constants::PI / 2) / 3;
            acc_cmd_v_y = 9.8 + 0.9 * (dirt_height) - 2 * 4 * 0.28 * Vy; // 类似1/（2.5s+1）一阶阶跃信号
        }
        else{
            acc_cmd_v_y = 9.8 + 0.9 * (m_initialHeight - lla.z()) - 2 * 4 * 0.28 * Vy;
            result.isCompleted = (fabs(lla.z() - m_initialHeight) < 1e-6) ? true : false;
        }
        result.acc_cmd_v = {0, acc_cmd_v_y, 0};
        if (result.isCompleted) std::cout << "垂直S运动结束，结束信息：" << "current height= " << lla.y() << std::endl;
        return result;
    }

    void Somersault::init(const State& state, std::string para){
        std::cout << "设置筋斗" << std::endl;
        lastPitch = Utils::CoordinateHelper::quaternionToEuler231(state.qbn).y();
    }

    RESULT Somersault::update(const State& state){
        RESULT result;
        result.isCompleted = false;
        result.acc_cmd_v = {0.0, 0.0, 0.0};
        // std::cout<<lastPitch<<std::endl;
        auto currentAtt = Utils::CoordinateHelper::quaternionToEuler231(state.qbn);
        pitchChangeCum += fabs(currentAtt.y() - lastPitch);
        lastPitch = currentAtt.y();
        if ((pitchChangeCum > 270) && (fabs(currentAtt.y()) < 0.05)){
            result.isCompleted = true;
            std::cout << "筋斗结束" << std::endl;
        }
        return result;
    }

    void SplitS::init(const State& state, std::string para){
        std::cout << "设置斜半滚倒转机动" << std::endl;
        // lastPitch = Utils::CoordinateHelper::quaternionToEuler231(state.qbn).y();
    }

    RESULT SplitS::update(const State& state){
        RESULT result;
        result.isCompleted = false;
        result.acc_cmd_v = {0.0, 0.0, 0.0};
        auto currentAtt = Utils::CoordinateHelper::quaternionToEuler231(state.qbn);
        switch (getStage()){
            case ManeuveringStage::STAGE1: {
                if (fabs(currentAtt.z()) > 179.5){
                    setStage(ManeuveringStage::STAGE2);
                }
                break;
            }
            case ManeuveringStage::STAGE2: {
                pitchChangeCum += fabs(currentAtt.y() - lastPitch);
                lastPitch = currentAtt.y();
                if ((pitchChangeCum > 135) && (fabs(currentAtt.y()) < 0.05)){
                    result.isCompleted = true;
                    std::cout << "斜半滚倒转机动结束" << std::endl;
                }
                break;
            }
            default: break;
        }
        return result;
    }

    auto ManeuveringMode::setManeuvering(ManeuveringModeType _type, const std::string& _paraStr,
                                         const State& state) -> void{
        _control.isCompleted = false;
        m_currentType = _type;
        switch (m_currentType){
            case ManeuveringModeType::LevelFlight: {
                _maneuveringPtr = std::make_unique<LevelFight>();
                _maneuveringPtr->init(state, "");
                break;
            }
            case ManeuveringModeType::UpAndDown: {
                _maneuveringPtr = std::make_unique<UpAndDown>();
                _maneuveringPtr->init(state, _paraStr);
                break;
            }
            case ManeuveringModeType::L: {
                _maneuveringPtr = std::make_unique<L>();
                _maneuveringPtr->init(state, _paraStr);
                break;
            }
            case ManeuveringModeType::Circle: {
                _maneuveringPtr = std::make_unique<Circle>();
                _maneuveringPtr->init(state, _paraStr);
                break;
            }
            case ManeuveringModeType::SLevel: {
                _maneuveringPtr = std::make_unique<SLevel>();
                _maneuveringPtr->init(state, _paraStr);
                break;
            }
            case ManeuveringModeType::SVertical: {
                _maneuveringPtr = std::make_unique<SVertical>();
                _maneuveringPtr->init(state, _paraStr);
                break;
            }
            case ManeuveringModeType::Somersault: {
                _maneuveringPtr = std::make_unique<Somersault>();
                _maneuveringPtr->init(state, _paraStr);
                Somersault_Para somersault_para = BaseManeuvering::getManeuveringPara<Somersault_Para>(_paraStr);
                _control.setAngularRateControl({0, 0, somersault_para.yawRundder});
                break;
            }
            case ManeuveringModeType::SplitS: {
                _maneuveringPtr = std::make_unique<SplitS>();
                _maneuveringPtr->init(state, _paraStr);
                _control.setManeuveringStageFun([&]()-> ManeuveringStage{
                    return _maneuveringPtr->getStage();
                });
                break;
            }
            default: break;
        }
    }

    auto ManeuveringMode::update(double step, const State& state, const double totalMass,
                                 const Eigen::Vector3d& p_body, const ImuInfo& imu_info,
                                 const double& s) -> std::pair<Eigen::Vector3d, Eigen::Vector3d>{
        if (_maneuveringPtr == nullptr) return {{0, 0, 0}, {0, 0, 0}};;
        auto result = _maneuveringPtr->update(state);
        auto acc_cmd_v = result.acc_cmd_v;
        auto re = _control.P6dof_Control(step, acc_cmd_v, state, totalMass, p_body, imu_info, s,
                                         m_currentType);
        if (result.isCompleted){
            setManeuvering(ManeuveringModeType::LevelFlight, "", state);
        }
        return re;
    }

    auto ManeuveringMode::getManeuveringModeType() const -> ManeuveringModeType{
        return m_currentType;
    };
}
