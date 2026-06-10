#ifndef AIRCRAFT_AIRCRAFT_MAIN_HPP
#define AIRCRAFT_AIRCRAFT_MAIN_HPP
#include "ylt/struct_pack.hpp"
#include "State.h"
#include <iostream>

namespace ModelDevelop::Aircraft{
    /*!
     * @brief
     */
    /*!
     * @brief 机动步进结果
     */
    struct RESULT{
        /*!
         * @brief 机动指令
         */
        Eigen::Vector3d acc_cmd_v = {0.0, 0.0, 0.0};
        /*!
         * @brief 机动是否完成
         */
        bool isCompleted = false;
    };

    /*!
     * @brief 机动类型
     */
    enum class ManeuveringModeType{
        LevelFlight = 0,
        UpAndDown = 1,
        L = 2, //左右L型
        Circle = 3, //圆周
        SLevel = 4, //水平S曲线
        SVertical = 5, //数值S
        Somersault = 6, //筋斗
        SplitS = 7, //斜半滚倒转机动,破S
        Immelmann = 8 //殷麦曼机动
    };

    enum class ManeuveringStage{
        STAGE1 = 1,
        STAGE2 = 2,
        STAGE3 = 3
    };

    /*!
     * @brief 机动基类
     */
    class BaseManeuvering{
    public:
        virtual ~BaseManeuvering() = default;
        /*!
         * @brief 初始化
         * @param state 实体当前状态信息
         * @param para 参数值序列化的字符串
         */
        virtual void init(const State& state, std::string para) =0;
        /*!
         * @brief
         * @param state 实体当前状态信息
         * @return RESULT
         */
        virtual RESULT update(const State& state) =0;
        /*!
         * @brief 参数序列化
         * @tparam T 参数类型
         * @param value 参数值
         * @return str类型的参数序列化结果
         */
        template <typename T>
        static auto setManeuveringPara(T value) -> std::string{
            return struct_pack::serialize<std::string>(value);
        }

        /*!
         * @brief 参数反序列化
         * @tparam T_PARA 参数类型
         * @param paraStr 参数值序列化的字符串
         * @return 参数值
         */
        template <typename T_PARA>
        static auto getManeuveringPara(std::string paraStr) -> T_PARA{
            auto result = struct_pack::deserialize<T_PARA>(paraStr);
            if (result){
                return std::move(result.value());
            }
            std::cerr << "反序列化失败：" << result.error().message() << std::endl;
            return T_PARA{};
        }
    public:
        void setStage(const ManeuveringStage &_Stage){
             m_Stage=_Stage;
        }
        ManeuveringStage getStage() const{
            return m_Stage;
        }
    private:
        ManeuveringStage m_Stage=ManeuveringStage::STAGE1;
    };

    /*!
     * @brief 方向
     */
    enum class TurnDirection{
        Left = 0,
        Right = 1,
    };

    /*!
     * @brief 升降参数
     */
    struct UpAndDown_Para{
        /*!
         * 目标高度
         */
        double targetHeight;
    };

    /*!
     * @brief 左右L型机动参数
     */
    struct L_Para{
        /*!
         * 方向
         */
        TurnDirection turnDir = TurnDirection::Left;
    };

    /*!
     * @brief 圆周运动参数
     */
    struct Circle_Para{
        /*!
         * 方向
         */
        TurnDirection turnDir = TurnDirection::Left;
        /*!
         * 圈数
         */
        double numberOfTurns = 1.0;
    };

    struct SLevel_Para{
        /*!
         * 方向
         */
        TurnDirection turnDir = TurnDirection::Left;
    };

    struct SVertical_Para{
        double minDurationTime = 0.0;
    };

    struct Somersault_Para{
        double yawRundder = 0.0;
    };
}
#endif
