/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#pragma once

namespace twist {

/**
 * @ingroup common
 * Namespace for common utilities
 */
namespace common {

#define LOG_INFO(message) ROS_INFO_STREAM("[TWIST] " << message)
#define LOG_ERROR(message) ROS_ERROR_STREAM("[TWIST] " << message)
#define LOG_WARN(message) ROS_WARN_STREAM("[TWIST] " << message)
#define LOG_FATAL(message) ROS_FATAL_STREAM("[TWIST] " << message)

}  // namespace common
}  // namespace twist
