/*
   @@@@@@@@@@@@@@@@@@@@
   @@@@@@@@@&@@@&&@@@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
   @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
   @@@@@ @@  @@    @@@@
   @@@@@@@@@&@@@@@@@@@@
   @@@@@@@@@@@@@@@@@@@@

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

// Provide fake header guard for cpplint
#undef TRACETOOLS_KERNELS__TP_CALL_H_
#ifndef TRACETOOLS_KERNELS__TP_CALL_H_
#define TRACETOOLS_KERNELS__TP_CALL_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER ros2_kernels

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "tracetools_kernels/tp_call.h"

#if !defined(_TRACETOOLS_KERNELS__TP_CALL_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRACETOOLS_KERNELS__TP_CALL_H_

#include <lttng/tracepoint.h>

#include <stdint.h>
#include <stdbool.h>

// kernel init(link to some further metadata)
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER, // tracepoint provider name
    kernel_register,     // tracepoint name
    TP_ARGS(
        // input arguments, see https://lttng.org/docs/v2.12/#doc-tpp-def-input-args
        const void *, node_arg, // link to connect kernels to nodes
        const void *, kernel_arg,
        const char *, kernel_name_arg),
    TP_FIELDS(
        // output event fields, see https://lttng.org/man/3/lttng-ust/v2.12/#doc-ctf-macros
        ctf_integer_hex(const void *, node, node_arg)
            ctf_integer_hex(const void *, kernel, kernel_arg)
                ctf_string(kernel_name, kernel_name_arg)))

// kernel start point of call
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER, // tracepoint provider name
    kernel_start,        // tracepoint name
    TP_ARGS(
        // input arguments, see https://lttng.org/docs/v2.12/#doc-tpp-def-input-args
        const void *, kernel_arg,
        const void *, msg_arg,
        uint32_t, header_nsec_arg,
        uint32_t, header_sec_arg,
        size_t, msg_size_arg),
    TP_FIELDS(
        // output event fields, see https://lttng.org/man/3/lttng-ust/v2.12/#doc-ctf-macros
        ctf_integer_hex(const void *, kernel, kernel_arg)
            ctf_integer_hex(const void *, message, msg_arg)
                ctf_integer(uint32_t, header_nsec, header_nsec_arg)
                    ctf_integer(uint32_t, header_sec, header_sec_arg)
                        ctf_integer(size_t, message_size, msg_size_arg)))

// // kernel end point of call
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER,
    kernel_end,
    TP_ARGS(
        const void *, kernel_arg),
    TP_FIELDS(
        ctf_integer_hex(const void *, kernel, kernel_arg)))

#endif // _TRACETOOLS_KERNELS__TP_CALL_H_

#include <lttng/tracepoint-event.h>

#endif // TRACETOOLS_KERNELS__TP_CALL_H_
