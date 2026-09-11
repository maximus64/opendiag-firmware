// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "../platform/platform.h"

namespace diagnostics {
class ApiTrace {
  public:
    ApiTrace(const void *anchor, const char *api, const char *function, const char *format, ...);
    ~ApiTrace();
    void inputValue(const char *name, const J2534_ULONG *value);
    void outputValue(const char *name, const J2534_ULONG *value, bool includeErrors = false);
    J2534_LONG result(J2534_LONG status);

  private:
    struct Output {
        const char *name;
        const J2534_ULONG *value;
        bool includeErrors;
    };
    bool enabled_;
    bool returned_;
    const char *api_;
    const char *function_;
    uint32_t started_;
    uint32_t call_;
    uint32_t maximumBytes_;
    char path_[4096];
    Output outputs_[4];
    unsigned outputCount_;
    void record(const char *format, ...);
    ApiTrace(const ApiTrace &) = delete;
    ApiTrace &operator=(const ApiTrace &) = delete;
};
} // namespace diagnostics
