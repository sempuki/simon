// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#define DECLARE_DEFAULTABLE(ClassName) ClassName() = default;
#define DECLARE_NON_DEFAULTABLE(ClassName) ClassName() = delete;

#define DECLARE_COPYABLE(ClassName)      \
  ClassName(const ClassName&) = default; \
  ClassName& operator=(const ClassName&) = default;

#define DECLARE_NON_COPYABLE(ClassName) \
  ClassName(const ClassName&) = delete; \
  ClassName& operator=(const ClassName&) = delete;

#define DECLARE_MOVABLE(ClassName)  \
  ClassName(ClassName&&) = default; \
  ClassName& operator=(ClassName&&) = default;

#define DECLARE_NON_MOVABLE(ClassName) \
  ClassName(ClassName&&) = delete;     \
  ClassName& operator=(ClassName&&) = delete;

#define DECLARE_COPY_ONLY(ClassName) \
  DECLARE_COPYABLE(ClassName)        \
  DECLARE_NON_MOVABLE(ClassName)

#define DECLARE_MOVE_ONLY(ClassName) \
  DECLARE_NON_COPYABLE(ClassName)    \
  DECLARE_MOVABLE(ClassName)

#define DECLARE_MOVE_COPY_ABLE(ClassName) \
  DECLARE_COPYABLE(ClassName)             \
  DECLARE_MOVABLE(ClassName)

#define DECLARE_UNMOVABLE(ClassName) \
  DECLARE_NON_COPYABLE(ClassName)    \
  DECLARE_NON_MOVABLE(ClassName)

#define DECLARE_SEMIREGULAR(ClassName) \
  DECLARE_DEFAULTABLE(ClassName)       \
  DECLARE_COPYABLE(ClassName)          \
  DECLARE_MOVABLE(ClassName)

