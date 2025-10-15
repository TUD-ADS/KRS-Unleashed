/*
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Copyright 2021 Víctor Mayoral-Vilches
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** \mainpage tracetools_kernels: tracing tools and instrumentation for
 *  for kernels in ROS 2.
 *
 * `tracetools_kernels` provides utilities to instrument ROS packages.
 * It provides two main headers:
 *
 * - tracetools/tracetools.h
 *   - instrumentation functions
 * - tracetools/utils.hpp
 *   - utility functions
 */

#ifndef TRACETOOLS_KERNELS__TRACETOOLS_H_
#define TRACETOOLS_KERNELS__TRACETOOLS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "tracetools_kernels/config.h"
#include "tracetools_kernels/visibility_control.hpp"

#ifndef TRACETOOLS_DISABLED
/// Call a tracepoint.
/**
 * This is the preferred method over calling the actual function directly.
 */
#define TRACEPOINT(event_name, ...) \
  (ros_trace_##event_name)(__VA_ARGS__)
#define DECLARE_TRACEPOINT(event_name, ...) \
  TRACETOOLS_PUBLIC void ros_trace_##event_name(__VA_ARGS__);
#else
#define TRACEPOINT(event_name, ...) ((void)(0))
#define DECLARE_TRACEPOINT(event_name, ...)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  /// Get tracing compilation status.
  /**
   * \return `true` if tracing is enabled, `false` otherwise
   */
  TRACETOOLS_PUBLIC bool ros_trace_compile_status();

  /// `kernel_register`
  /**
   * Kernel initialisation
   *
   *
   * \param[in] kernel pointer to the kernel object this call belongs to
   * \param[in] node rclcpp::node::Node subject to the kernel function (node's `rcl_node_t` handle)
   * \param[in] kernel_name custom name given by the developer to enable easy recoginition
   */
  DECLARE_TRACEPOINT(
      kernel_register,
      const void *node,
      const void *kernel,
      const char *kernel_name)

  /// `kernel_start`
  /**
   * Tracepoint while initiating the call of a kernel component
   *
   *
   * \param[in] kernel pointer to the kernel object this call belongs to
   * \param[in] message ROS message stored as message pointer to the message being taken
   * \param[in] header_nsec nanosec field of the header (std_msgs/Header) of ROS message
   * \param[in] header_sec sec field of the header (std_msgs/Header) of ROS message
   * \param[in] message_size size of ROS message stored as bytes
   */
  DECLARE_TRACEPOINT(
      kernel_start,
      const void *kernel,
      const void *message,
      uint32_t header_nsec,
      uint32_t header_sec,
      size_t message_size)

  /// `kernel_end`
  /**
   * Tracepoint after ending the call of a kernel component
   *
   *
   * \param[in] kernel pointer to the kernel object this call belongs to
   */
  DECLARE_TRACEPOINT(
      kernel_end,
      const void *kernel)

#ifdef __cplusplus
}
#endif

#endif // TRACETOOLS_KERNELS__TRACETOOLS_H_
