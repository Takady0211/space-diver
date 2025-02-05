// Copyright (c) 2024 Tohoku Univ. Space Robotics Lab.
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

#ifndef SPACEDIVER_COMMAND_INTERFACE__VISIBILITY_CONTROL_H_
#define SPACEDIVER_COMMAND_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SPACEDIVER_COMMAND_INTERFACE_EXPORT __attribute__((dllexport))
#define SPACEDIVER_COMMAND_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define SPACEDIVER_COMMAND_INTERFACE_EXPORT __declspec(dllexport)
#define SPACEDIVER_COMMAND_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef SPACEDIVER_COMMAND_INTERFACE_BUILDING_LIBRARY
#define SPACEDIVER_COMMAND_INTERFACE_PUBLIC SPACEDIVER_COMMAND_INTERFACE_EXPORT
#else
#define SPACEDIVER_COMMAND_INTERFACE_PUBLIC SPACEDIVER_COMMAND_INTERFACE_IMPORT
#endif
#define SPACEDIVER_COMMAND_INTERFACE_PUBLIC_TYPE SPACEDIVER_COMMAND_INTERFACE_PUBLIC
#define SPACEDIVER_COMMAND_INTERFACE_LOCAL
#else
#define SPACEDIVER_COMMAND_INTERFACE_EXPORT __attribute__((visibility("default")))
#define SPACEDIVER_COMMAND_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define SPACEDIVER_COMMAND_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define SPACEDIVER_COMMAND_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define SPACEDIVER_COMMAND_INTERFACE_PUBLIC
#define SPACEDIVER_COMMAND_INTERFACE_LOCAL
#endif
#define SPACEDIVER_COMMAND_INTERFACE_PUBLIC_TYPE
#endif

#endif  // SPACEDIVER_COMMAND_INTERFACE__VISIBILITY_CONTROL_H_
