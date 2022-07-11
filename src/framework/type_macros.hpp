// Copyright 2022 -- CONTRIBUTORS.

#pragma once

#define MARK_NON_DEFAULTABLE(ClassName) ClassName() = delete;

#define MARK_NON_COPYABLE(ClassName)    \
  ClassName(const ClassName&) = delete; \
  ClassName& operator=(const ClassName&) = delete;

#define MARK_NON_MOVECOPYABLE(ClassName)           \
  ClassName(const ClassName&) = delete;            \
  ClassName& operator=(const ClassName&) = delete; \
  ClassName(ClassName&&) = delete;                 \
  ClassName& operator=(ClassName&&) = delete;

