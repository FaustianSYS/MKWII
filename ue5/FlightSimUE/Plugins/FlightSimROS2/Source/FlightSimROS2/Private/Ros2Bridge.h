#pragma once
// Private UE5-side glue — no rclcpp, no STL templates, no RTTI.
// Includes only the plain C API header that the wrapper library exports.

#include "CoreMinimal.h"
#include "fsros2_bridge.h"   // plain C — safe under UE5's libc++ / -fno-rtti
