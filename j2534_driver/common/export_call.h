// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "api_trace.h"
#include <exception>

// Runs one API body and reports its status through finish(), which both the 05.00 and the
// 04.04 surface provide. A failure that escapes as an exception becomes ERR_FAILED with the
// exception's message, so no C++ exception crosses the C ABI. The including file must have
// the J2534 status codes in scope.
template <typename Body, typename Finish>
J2534_LONG runExport(diagnostics::ApiTrace &trace, Body body, Finish finish) {
    try {
        return trace.result(finish(body(), NULL));
    } catch (const std::exception &error) {
        return trace.result(finish(ERR_FAILED, error.what()));
    } catch (...) {
        return trace.result(finish(ERR_FAILED, "Unexpected host exception"));
    }
}
