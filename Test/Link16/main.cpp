#include <iostream>
#include <string>
#include <thread>
#include <cstdint>

/*
==============================================================================
Link‑16 最简模型（仿真时间驱动 / 无系统时间）
==============================================================================
*/

// ============================================================================
// 全局仿真时间（单位：微秒）
// ============================================================================

class SimTime {
public:
    static uint64_t now() {
        return currentTimeUs_;
    }

    static void step(uint64_t us) {
        currentTimeUs_ += us;
    }

private:
    static uint64_t currentTimeUs_;
};

// 静态成员定义
uint64_t SimTime::currentTimeUs_ = 0;


// ============================================================================
// 全局仿真参数
// ============================================================================

constexpr uint64_t SLOT_DURATION_US = 1'000;   // 伪时隙长度（1 毫秒）
constexpr uint32_t  FRAME_SIZE      = 64;


// =========================================================================
// Link‑16 终端（收发一体）
// =========================================================================

class LIN16Terminal {
public:
    LIN16Terminal(
        std::string id,
        uint32_t    slotId,//时隙编号
        double      freqMHz,//辐射信号中心频率 哪个波段通信
        double      bwKHz,//带宽
        std::string modType,//调制方式
        double      pwUs,//脉冲宽度
        double      priUs,//重复周期
        std::string termClass//通信终端类别
    )
        : id_(std::move(id)),
          slotId_(slotId),
          freqMHz_(freqMHz),
          bwKHz_(bwKHz),
          modType_(std::move(modType)),
          pwUs_(pwUs),
          priUs_(priUs),
          termClass_(std::move(termClass)) {}

    bool isMySlot(uint64_t simTimeUs) const {
        uint32_t slot = static_cast<uint32_t>(
            simTimeUs / SLOT_DURATION_US
        );
        bool ret = (slot % FRAME_SIZE) == slotId_;
        return ret;
    }

    std::string send(const std::string& msg, uint64_t simTimeUs) {
        if (!isMySlot(simTimeUs)) {
            return msg;
        }

        std::cout << "[TX] " << id_
                  << " | 仿真时间 " << simTimeUs/1000.
                  << " ms | 发送: " << msg
                  << std::endl;
        return msg;
    }

    void receive(const std::string& msg, uint64_t simTimeUs) {
        if (!isMySlot(simTimeUs)) {
            return;
        }

        if (!msg.empty()) {
            std::cout << "[RX] " << id_
                      << " | 仿真时间 " << simTimeUs/1000.
                      << " ms | 接收成功"
                      << std::endl;
        }
    }

private:
    std::string id_;
    uint32_t    slotId_;

    double      freqMHz_;
    double      bwKHz_;
    std::string modType_;
    double      pwUs_;
    double      priUs_;
    std::string termClass_;
};


// =========================================================================
// 主仿真循环
// =========================================================================

int main() {
    using namespace std::chrono;

    // 两个终端（同一伪时隙）
    LIN16Terminal t1(
        "T1", 2, 960.0, 25.0, "MSK", 20.0, 1000.0, "Link16_Terminal_A"
    );

    LIN16Terminal t2(
        "T2", 4, 960.0, 25.0, "MSK", 20.0, 1000.0, "Link16_Terminal_B"
    );

    // 仿真步长（1 ms）
    constexpr uint64_t STEP_US = 1000;

    while (true) {
        uint64_t now = SimTime::now();

        std::string msg = t1.send("J-message-001", now);
        t2.receive(msg, now);

        SimTime::step(STEP_US);
        std::this_thread::sleep_for(milliseconds(1));
    }

    return 0;
}