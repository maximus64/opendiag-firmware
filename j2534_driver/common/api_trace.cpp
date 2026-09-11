// SPDX-License-Identifier: GPL-3.0-only
#include "api_trace.h"
#include <algorithm>

namespace diagnostics {
static std::atomic<uint32_t> nextCall(0);
static std::atomic<bool> writing(false);

ApiTrace::ApiTrace(const void *anchor,
                   const char *api,
                   const char *function,
                   const char *format,
                   ...)
    : enabled_(false),
      returned_(false),
      api_(api),
      function_(function),
      started_(platform::milliseconds()),
      call_(0),
      maximumBytes_(0),
      outputCount_(0) {
    platform::ErrorPreserver preserve;
    try {
        std::string ini = platform::configPath(anchor);
        if (platform::readNumber(ini, "logging", "Enabled", 0)) {
            unsigned maximumKB = platform::readNumber(ini, "logging", "MaxFileKB", 4096);
            maximumBytes_ = std::max(64U, std::min(65536U, maximumKB)) * 1024;
            int count = platform::format(path_,
                                         sizeof(path_),
                                         "%sopendiag-%s-%u.log",
                                         platform::moduleDirectory(anchor).c_str(),
                                         api,
                                         static_cast<unsigned>(platform::processId()));
            if (count >= 0 && static_cast<size_t>(count) + 2 < sizeof(path_)) {
                enabled_ = true;
                call_ = nextCall.fetch_add(1) + 1;
            }
        }
    } catch (...) {
        // Logging failures must not change the API's result.
    }
    if (enabled_) {
        char arguments[768];
        va_list values;
        va_start(values, format);
        platform::formatV(arguments, sizeof(arguments), format, values);
        va_end(values);
        record("BEGIN %s", arguments);
    }
}

ApiTrace::~ApiTrace() {
    if (enabled_ && !returned_) {
        record("ABORT no return recorded");
    }
}

void ApiTrace::record(const char *format, ...) {
    platform::ErrorPreserver preserve;
    if (!enabled_ || writing.exchange(true)) {
        return;
    }
    char detail[768];
    va_list values;
    va_start(values, format);
    platform::formatV(detail, sizeof(detail), format, values);
    va_end(values);
    char timestamp[32];
    platform::utcTimestamp(timestamp, sizeof(timestamp));
    char line[1024];
    platform::format(line,
                     sizeof(line),
                     "%s pid=%u tid=%u call=%u api=%s %s %s\r\n",
                     timestamp,
                     static_cast<unsigned>(platform::processId()),
                     static_cast<unsigned>(platform::threadId()),
                     static_cast<unsigned>(call_),
                     api_,
                     function_,
                     detail);
    platform::appendLog(path_, line, maximumBytes_);
    writing.store(false);
}

void ApiTrace::inputValue(const char *name, const J2534_ULONG *source) {
    if (!enabled_) {
        return;
    }
    platform::ErrorPreserver preserve;
    J2534_ULONG value = 0;
    if (platform::readValue(source, value)) {
        record("INPUT %s=%u (0x%08X)",
               name,
               static_cast<unsigned>(value),
               static_cast<unsigned>(value));
    } else {
        record("INPUT %s=<unreadable> pointer=%p", name, static_cast<const void *>(source));
    }
}

void ApiTrace::outputValue(const char *name, const J2534_ULONG *value, bool includeErrors) {
    if (enabled_ && outputCount_ < sizeof(outputs_) / sizeof(outputs_[0])) {
        Output &output = outputs_[outputCount_++];
        output.name = name;
        output.value = value;
        output.includeErrors = includeErrors;
    }
}

J2534_LONG ApiTrace::result(J2534_LONG status) {
    returned_ = true;
    if (!enabled_) {
        return status;
    }
    platform::ErrorPreserver preserve;
    record("END status=0x%08X elapsed_ms=%u",
           static_cast<unsigned>(status),
           static_cast<unsigned>(platform::milliseconds() - started_));
    for (unsigned index = 0; index < outputCount_; ++index) {
        const Output &output = outputs_[index];
        if (!status || output.includeErrors) {
            J2534_ULONG value = 0;
            if (platform::readValue(output.value, value)) {
                record("OUTPUT %s=%u (0x%08X)",
                       output.name,
                       static_cast<unsigned>(value),
                       static_cast<unsigned>(value));
            } else {
                record("OUTPUT %s=<unreadable> pointer=%p",
                       output.name,
                       static_cast<const void *>(output.value));
            }
        }
    }
    return status;
}
} // namespace diagnostics
