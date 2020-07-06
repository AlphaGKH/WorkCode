#include "spider/init.h"

#include <mutex>
#include <csignal>

#include "spider/spinner/spinner.h"

namespace spider {

namespace {
bool g_atexit_registered = false;
std::mutex g_mutex;
}

void OnShutdown(int sig) {
    (void)sig;
    if (GetState() != STATE_SHUTDOWN) {
        SetState(STATE_SHUTTING_DOWN);
    }
}

void ExitHandle() { Clear(); }

bool Init(const char* binary_name) {
    std::lock_guard<std::mutex> lg(g_mutex);
    if (GetState() != STATE_UNINITIALIZED) {
        return false;
    }

    std::signal(SIGINT, OnShutdown);
    // Register exit handlers
    if (!g_atexit_registered) {
        if (std::atexit(ExitHandle) != 0) {
            AERROR << "Register exit handle failed";
            return false;
        }
        AINFO << "Register exit handle succ.";
        g_atexit_registered = true;
    }
    SetState(STATE_INITIALIZED);
    return true;
}

void Clear() {
    std::lock_guard<std::mutex> lg(g_mutex);
    if (GetState() == STATE_SHUTDOWN || GetState() == STATE_UNINITIALIZED) {
        return;
    }

    Spinner::CleanUp();

    SetState(STATE_SHUTDOWN);
}

}
