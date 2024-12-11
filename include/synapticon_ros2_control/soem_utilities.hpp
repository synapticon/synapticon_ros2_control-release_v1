#pragma once

#include "ethercat.h"

#include <atomic>

namespace synapticon_ros2_control {
namespace soem_globals {
inline std::atomic<bool> kInOp;
inline volatile std::atomic<int> kWkc;
inline std::atomic<int> kExpectedWkc;
inline std::atomic<bool> kNeedlf;
} // namespace soem_globals

/**
 * @brief Error checking. Typically runs in a separate thread.
 */
OSAL_THREAD_FUNC ecatcheck(void *ptr);
} // namespace synapticon_ros2_control
