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

#include "tracetools_kernels/tracetools.h"

#ifndef TRACETOOLS_DISABLED

#ifdef TRACETOOLS_LTTNG_ENABLED
#include "tracetools_kernels/tp_call.h"
#define CONDITIONAL_TP(...) \
  tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
#else
#define CONDITIONAL_TP(...)
#endif

bool ros_trace_compile_status()
{
#ifdef TRACETOOLS_LTTNG_ENABLED
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#else
#pragma warning(push)
#pragma warning(disable : 4100)
#endif

void TRACEPOINT(
    kernel_register,
    const void *node,
    const void *kernel,
    const char *kernel_name)
{
  CONDITIONAL_TP(
      kernel_register,
      node,
      kernel,
      kernel_name);
}

void TRACEPOINT(
    kernel_start,
    const void *kernel,
    const void *message,
    uint32_t header_nsec,
    uint32_t header_sec,
    size_t message_size)
{
  CONDITIONAL_TP(
      kernel_start,
      kernel,
      message,
      header_nsec,
      header_sec,
      message_size);
}

void TRACEPOINT(
    kernel_end,
    const void *kernel)
{
  CONDITIONAL_TP(
      kernel_end,
      kernel);
}

#ifndef _WIN32
#pragma GCC diagnostic pop
#else
#pragma warning(pop)
#endif

#endif // TRACETOOLS_DISABLED
