// SPDX-License-Identifier: GPL-3.0-only
#include "resource.h"
#include "settings.h"
#include <process.h>
#include <shellapi.h>
#include <stdexcept>

static const UINT WM_CHECK_FINISHED = WM_APP + 1;
static const char UPDATE_URL[] = "https://dalalogic.com/opendiag/update";

struct Application {
    HWND window;
    std::string directory;
    std::string ini;
    AdapterSettings settings;
    bool loading;
    bool dirty;
    bool busy;
};

static std::string controlText(HWND window, int id) {
    char text[256];
    GetDlgItemTextA(window, id, text, sizeof(text));
    return text;
}

static void setText(HWND window, int id, const std::string &value) {
    SetDlgItemTextA(window, id, value.c_str());
}

static bool checked(HWND window, int id) {
    return IsDlgButtonChecked(window, id) == BST_CHECKED;
}

static void updateControls(Application &app) {
    bool com = SendDlgItemMessage(app.window, IDC_TRANSPORT, CB_GETCURSEL, 0, 0) == 0;
    for (HWND child = GetWindow(app.window, GW_CHILD); child;
         child = GetWindow(child, GW_HWNDNEXT)) {
        EnableWindow(child, !app.busy);
    }
    EnableWindow(GetDlgItem(app.window, IDC_COM), !app.busy && com);
    EnableWindow(GetDlgItem(app.window, IDC_REFRESH), !app.busy && com);
    EnableWindow(GetDlgItem(app.window, IDC_BAUD), !app.busy && com);
    EnableWindow(GetDlgItem(app.window, IDC_HOST), !app.busy && !com);
    EnableWindow(GetDlgItem(app.window, IDC_PORT), !app.busy && !com);
    EnableWindow(GetDlgItem(app.window, IDC_LOG_SIZE),
                 !app.busy && checked(app.window, IDC_LOGGING));
    EnableWindow(GetDlgItem(app.window, IDC_STATUS), TRUE);
}

static void refreshPorts(Application &app) {
    std::string data = controlText(app.window, IDC_COM);
    std::vector<std::string> ports = availableComPorts();
    SendDlgItemMessage(app.window, IDC_COM, CB_RESETCONTENT, 0, 0);
    for (size_t i = 0; i < ports.size(); ++i) {
        SendDlgItemMessageA(app.window, IDC_COM, CB_ADDSTRING, 0, (LPARAM)ports[i].c_str());
    }
    setText(app.window, IDC_COM, data);
}

static void showSettings(Application &app) {
    const AdapterSettings &s = app.settings;
    app.loading = true;
    setText(app.window, IDC_NAME, s.name);
    SendDlgItemMessageA(app.window, IDC_TRANSPORT, CB_ADDSTRING, 0, (LPARAM) "USB serial (COM)");
    SendDlgItemMessageA(app.window, IDC_TRANSPORT, CB_ADDSTRING, 0, (LPARAM) "TCP/IP");
    SendDlgItemMessage(app.window, IDC_TRANSPORT, CB_SETCURSEL, s.transport == "com" ? 0 : 1, 0);
    setText(app.window, IDC_COM, s.com);
    setText(app.window, IDC_HOST, s.host);
    SetDlgItemInt(app.window, IDC_PORT, s.port, FALSE);
    SetDlgItemInt(app.window, IDC_BAUD, s.baud, FALSE);
    SetDlgItemInt(app.window, IDC_TIMEOUT, s.timeout, FALSE);
    SetDlgItemInt(app.window, IDC_LOG_SIZE, s.logSizeKB, FALSE);
    CheckDlgButton(app.window, IDC_LOGGING, s.logging ? BST_CHECKED : BST_UNCHECKED);
    refreshPorts(app);
    setText(app.window, IDC_VERSIONS, driverVersions(app.directory));
    setText(app.window,
            IDC_STATUS,
            "Settings: " + app.ini + "\r\nClose diagnostic software before testing the adapter.");
    app.loading = false;
    app.dirty = false;
    updateControls(app);
}

static AdapterSettings editedSettings(Application &app) {
    AdapterSettings s = app.settings;
    s.name = controlText(app.window, IDC_NAME);
    s.transport =
        SendDlgItemMessage(app.window, IDC_TRANSPORT, CB_GETCURSEL, 0, 0) == 0 ? "com" : "tcp";
    s.com = controlText(app.window, IDC_COM);
    s.host = controlText(app.window, IDC_HOST);
    s.port = parseNumber(controlText(app.window, IDC_PORT), "Data TCP port", 1, 65535);
    s.baud = parseNumber(controlText(app.window, IDC_BAUD), "Serial baud rate", 300, 4000000);
    s.timeout = parseNumber(controlText(app.window, IDC_TIMEOUT), "Connection timeout", 100, 30000);
    s.logSizeKB = parseNumber(controlText(app.window, IDC_LOG_SIZE), "Log file limit", 64, 65536);
    s.logging = checked(app.window, IDC_LOGGING);
    validateSettings(s);
    return s;
}

static void save(Application &app) {
    AdapterSettings next = editedSettings(app);
    saveSettings(app.ini, next);
    app.settings = next;
    app.dirty = false;
    setText(app.window,
            IDC_STATUS,
            "Settings saved. Connection changes apply when the diagnostic application reopens the "
            "adapter.");
}

static unsigned __stdcall connectionWorker(void *context) {
    Application &app = *(Application *)context;
    std::string *result = new std::string;
    try {
        *result = checkConnection(app.directory, app.settings);
    } catch (const std::exception &error) {
        *result = std::string(error.what()) +
                  "\r\nCheck power, cables and port settings. Close other diagnostic applications.";
    }
    if (!PostMessage(app.window, WM_CHECK_FINISHED, 0, (LPARAM)result)) {
        delete result;
    }
    return 0;
}

static void startCheck(Application &app) {
    save(app);
    app.busy = true;
    updateControls(app);
    setText(app.window, IDC_STATUS, "Checking the adapter connection...");
    uintptr_t thread = _beginthreadex(NULL, 0, connectionWorker, &app, 0, NULL);
    if (!thread) {
        app.busy = false;
        updateControls(app);
        throw std::runtime_error("Cannot start the connection check.");
    }
    CloseHandle((HANDLE)thread);
}

static void openLink(HWND window, const char *target) {
    HINSTANCE result = ShellExecuteA(window, "open", target, NULL, NULL, SW_SHOWNORMAL);
    if ((INT_PTR)result <= 32) {
        throw std::runtime_error(std::string("Cannot open ") + target +
                                 ". Open this address or folder manually.");
    }
}

static void closeDialog(Application &app) {
    if (app.busy) {
        return;
    }
    if (app.dirty) {
        int answer = MessageBoxA(app.window,
                                 "Save your configuration changes?",
                                 "OpenDIAG",
                                 MB_YESNOCANCEL | MB_ICONQUESTION);
        if (answer == IDCANCEL) {
            return;
        }
        if (answer == IDYES) {
            save(app);
        }
    }
    EndDialog(app.window, IDOK);
}

static INT_PTR CALLBACK dialogProcedure(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
    Application *app = (Application *)GetWindowLongPtr(window, DWLP_USER);
    try {
        if (message == WM_INITDIALOG) {
            app = (Application *)lparam;
            app->window = window;
            SetWindowLongPtr(window, DWLP_USER, (LONG_PTR)app);
            const int edits[] = {IDC_NAME, IDC_HOST};
            for (unsigned i = 0; i < sizeof(edits) / sizeof(edits[0]); ++i) {
                SendDlgItemMessage(window, edits[i], EM_SETLIMITTEXT, 79, 0);
            }
            SendDlgItemMessage(window, IDC_COM, CB_LIMITTEXT, 16, 0);
            showSettings(*app);
            return TRUE;
        }
        if (!app) {
            return FALSE;
        }
        if (message == WM_CHECK_FINISHED) {
            std::string *result = (std::string *)lparam;
            app->busy = false;
            updateControls(*app);
            setText(window, IDC_STATUS, *result);
            delete result;
            return TRUE;
        }
        if (message == WM_CLOSE) {
            closeDialog(*app);
            return TRUE;
        }
        if (message != WM_COMMAND || app->loading || app->busy) {
            return FALSE;
        }
        int id = LOWORD(wparam);
        int notification = HIWORD(wparam);
        if (notification == EN_CHANGE || notification == CBN_SELCHANGE ||
            notification == CBN_EDITCHANGE || id == IDC_LOGGING) {
            if (id != IDC_STATUS) {
                app->dirty = true;
            }
        }
        switch (id) {
        case IDC_TRANSPORT:
        case IDC_LOGGING:
            updateControls(*app);
            break;
        case IDC_REFRESH:
            refreshPorts(*app);
            break;
        case IDC_SAVE:
            save(*app);
            break;
        case IDC_TEST:
            startCheck(*app);
            break;
        case IDC_LOG_FOLDER:
            openLink(window, app->directory.c_str());
            break;
        case IDC_UPDATE:
            openLink(window, UPDATE_URL);
            break;
        case IDCANCEL:
            closeDialog(*app);
            break;
        }
    } catch (const std::exception &error) {
        MessageBoxA(window, error.what(), "OpenDIAG", MB_OK | MB_ICONERROR);
    }
    return FALSE;
}

int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int) {
    try {
        Application app = {};
        app.directory = applicationDirectory();
        app.ini = app.directory + "opendiag.ini";
        app.settings = readSettings(app.ini);
        INT_PTR result = DialogBoxParamA(instance,
                                         MAKEINTRESOURCEA(IDD_CONFIG),
                                         NULL,
                                         dialogProcedure,
                                         (LPARAM)&app);
        if (result == -1) {
            throw std::runtime_error("Cannot create the configuration window.");
        }
        return 0;
    } catch (const std::exception &error) {
        MessageBoxA(NULL, error.what(), "OpenDIAG", MB_OK | MB_ICONERROR);
        return 1;
    }
}
