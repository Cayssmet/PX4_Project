#ifndef WAYPOINT_MANAGER__ARRIVAL_TRACKER_HPP_
#define WAYPOINT_MANAGER__ARRIVAL_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace waypoint_manager
{

/**
 * @brief 跟踪“连续到达确认 -> 航点停留 -> 完成”的小型状态机。
 *
 * 输入的 reached 必须已经是“平面位置、yaw、高度和通信状态全部正常”的合并结果。
 * 该类与 ROS 2 无关，因此可以独立做单元测试。
 *
 * 设计要点：
 * 1. 刚出现一次 reached=true 并不立即算到达，必须连续满足 confirmation_time；
 * 2. 进入停留后，短于 lost_grace_time 的 false 被视为反馈噪声，不清空计时；
 * 3. false 持续过久则回到寻找状态，确认和停留都重新开始；
 * 4. 时间倒退或出现非有限时间时自动重置，避免错误地跳过航点。
 */
class ArrivalTracker
{
public:
  enum class Stage
  {
    SEEKING,
    CONFIRMING,
    DWELLING,
    COMPLETE
  };

  ArrivalTracker(double confirmation_time, double lost_grace_time)
  : confirmation_time_(confirmation_time), lost_grace_time_(lost_grace_time)
  {
    if (!std::isfinite(confirmation_time_) || confirmation_time_ < 0.0) {
      throw std::invalid_argument("confirmation_time 必须是大于等于 0 的有限数");
    }
    if (!std::isfinite(lost_grace_time_) || lost_grace_time_ < 0.0) {
      throw std::invalid_argument("lost_grace_time 必须是大于等于 0 的有限数");
    }
  }

  /**
   * @brief 输入当前完整到达条件，更新状态。
   * @param reached 当前是否同时满足所有到达条件。
   * @param now_sec 单调递增时间，单位秒。
   * @param hold_time 当前航点实际停留时间，单位秒。
   * @return 当前航点是否已经完整完成。
   */
  bool update(bool reached, double now_sec, double hold_time)
  {
    if (!std::isfinite(now_sec) || !std::isfinite(hold_time) || hold_time < 0.0) {
      reset();
      return false;
    }

    if (has_last_time_ && now_sec < last_time_sec_) {
      reset();
    }
    has_last_time_ = true;
    last_time_sec_ = now_sec;

    if (stage_ == Stage::COMPLETE) {
      return true;
    }

    if (stage_ == Stage::SEEKING || stage_ == Stage::CONFIRMING) {
      if (!reached) {
        stage_ = Stage::SEEKING;
        has_confirmation_start_ = false;
        return false;
      }

      if (!has_confirmation_start_) {
        confirmation_start_sec_ = now_sec;
        has_confirmation_start_ = true;
        stage_ = Stage::CONFIRMING;
      }

      if ((now_sec - confirmation_start_sec_) >= confirmation_time_) {
        stage_ = Stage::DWELLING;
        dwell_start_sec_ = now_sec;
        has_loss_start_ = false;
      }
    }

    if (stage_ == Stage::DWELLING) {
      if (reached) {
        has_loss_start_ = false;
      } else if (!has_loss_start_) {
        // 第一次失去到达条件时只启动宽限计时，不立刻破坏整个停留过程。
        loss_start_sec_ = now_sec;
        has_loss_start_ = true;
      } else if ((now_sec - loss_start_sec_) > lost_grace_time_) {
        reset();
        return false;
      }

      if ((now_sec - dwell_start_sec_) >= hold_time) {
        stage_ = Stage::COMPLETE;
        return true;
      }
    }

    return false;
  }

  void reset()
  {
    stage_ = Stage::SEEKING;
    has_confirmation_start_ = false;
    has_loss_start_ = false;
    has_last_time_ = false;
    confirmation_start_sec_ = 0.0;
    dwell_start_sec_ = 0.0;
    loss_start_sec_ = 0.0;
    last_time_sec_ = 0.0;
  }

  Stage stage() const {return stage_;}
  bool dwelling() const {return stage_ == Stage::DWELLING;}
  bool complete() const {return stage_ == Stage::COMPLETE;}

private:
  double confirmation_time_{0.7};
  double lost_grace_time_{0.3};

  Stage stage_{Stage::SEEKING};
  bool has_confirmation_start_{false};
  bool has_loss_start_{false};
  bool has_last_time_{false};
  double confirmation_start_sec_{0.0};
  double dwell_start_sec_{0.0};
  double loss_start_sec_{0.0};
  double last_time_sec_{0.0};
};

}  // namespace waypoint_manager

#endif  // WAYPOINT_MANAGER__ARRIVAL_TRACKER_HPP_
