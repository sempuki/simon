// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "base/core.hpp"

// lib's parameter vocabulary, declared in simon's namespace so that simon
// names it plainly: arguments a function writes (Out), reads and writes
// (InOut), or keeps a reference to (Depend).
namespace simon {

using lib::Depend;
using lib::InOut;
using lib::Out;

}  // namespace simon
