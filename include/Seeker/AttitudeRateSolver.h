#ifndef SEEKER_ATTITUDER_RATE_SOLVER_ATTITUDER_RATE_SOLVER
#define SEEKER_ATTITUDER_RATE_SOLVER_ATTITUDER_RATE_SOLVER
#include <cmath>
#include <iostream>
namespace ModelDevelop::SEEKER{
    class AttitudeRateSolver{
    public:
         AttitudeRateSolver ()=default ;
        /*!
         * @brief 即使视线角 q_d 没变，这个函数也应该以固定的 h 频率调用，因为 TD 需要保持内部状态的积分连续性
         * @param q_d_p 目标视线角-俯仰
         * @param q_d_y 目标视线角-偏航
         */
        void update(double q_d_p, double q_d_y);
        /*!
         * @return 获取计算出的俯仰角速率指令
         */
        [[nodiscard]] double getPitchRate() const{ return rate_pitch_cmd; }
        /*!
         * @return 获取计算出的偏航角速率指令
         */
        [[nodiscard]] double getYawRate() const{ return rate_yaw_cmd; }
    private:
        /*!
         * @brief 单通道跟踪微分器,对输入信号进行平滑处理并提取微分
         */
        class SingleChannelTD{
        public:
             SingleChannelTD() = default;
            /*!
             * @brief 第一帧初始化，或发生剧烈跳变后重置
             * @param initial_v 初始的输入信号值
             */
            void reset(double initial_v);
            /*!
             * @brief 步进
             * @param v 输入信号
             */
            void update(double v);

            /*!
             * @return获取平滑后的跟踪信号
             */
            [[nodiscard]] double getTracking() const{ return x1; }
            /*!
             * @return 获取解算的微分信号
             */
            [[nodiscard]] double getRate() const{ return x2; }

        private:
            inline static constexpr  double fs = 200;//采样率
            inline static constexpr  double r=1000; // 速度因子 r
            inline static constexpr  double h=0.5;//采样时间
            double x1=0.0, x2=0.0;
            static double sign(const double& x);
        } td_pitch, td_yaw;
        double rate_pitch_cmd{};// 存储计算出的俯仰速率指令
        double rate_yaw_cmd{}; // 存储计算出的偏航速率指令
        bool isFirstUpdate{true};
    };
}
#endif
