/********************************************************************
* Description: macros.cc
*   The [MDI_MACROS] table, see macros.hh.
*
*   [MDI_MACROS]
*   NAME = tool_change_position
*   COMMAND = G53 G0 Z0
*   MODES = MANUAL MDI
*
*   [HALUI]MDI_COMMAND entries are macros too, named mdi-command-NN like
*   their halui pins and allowed in every mode, so that every MDI command
*   goes through the same mode check.
*
* License: GPL Version 2
********************************************************************/

#include "macros.hh"

#include <inifile.hh>

#include <linuxcnc.h>              /* LINELEN */
#include <rtapi.h>

#include <cctype>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Macro {
    std::string name;
    std::string command;
    std::string modes;      /* for messages */
    bool manual = false, mdi = false, autom = false;
    bool pin = true;        /* halui.macro.<name>; MDI_COMMAND has its own */
};

std::vector<Macro> macros;

/* A name becomes part of a HAL pin name, halui.macro.<name> */
bool validName(const std::string &name)
{
    if (name.empty()) {
        return false;
    }
    for (char c : name) {
        if (!isalnum((unsigned char)c) && c != '_' && c != '-') {
            return false;
        }
    }
    return true;
}

bool parseModes(const std::string &text, Macro &m)
{
    std::istringstream in(text);
    std::string word;
    while (in >> word) {
        for (char &c : word) {
            c = toupper((unsigned char)c);
        }
        if (word == "MANUAL") {
            m.manual = true;
        } else if (word == "MDI") {
            m.mdi = true;
        } else if (word == "AUTO") {
            m.autom = true;
        } else {
            return false;
        }
    }
    if (m.manual) m.modes += "Manual";
    if (m.mdi) m.modes += m.modes.empty() ? "MDI" : ", MDI";
    if (m.autom) m.modes += m.modes.empty() ? "Auto" : ", Auto";
    return !m.modes.empty();
}

} // namespace

int taskMacrosLoad(linuxcnc::IniFile &inifile)
{
    macros.clear();
    for (int n = 1; n <= TASK_MACRO_MDI_COMMAND_MAX; n++) {
        auto command = inifile.findString(n, "MDI_COMMAND", "HALUI");
        if (!command) {
            break;
        }
        Macro m;
        char name[32];
        snprintf(name, sizeof(name), "mdi-command-%02d", n - 1);
        m.name = name;
        m.command = *command;
        m.manual = m.mdi = m.autom = true;
        m.modes = "Manual, MDI, Auto";
        m.pin = false;
        macros.push_back(m);
    }
    for (int n = 1; ; n++) {
        auto name = inifile.findString(n, "NAME", "MDI_MACROS");
        auto command = inifile.findString(n, "COMMAND", "MDI_MACROS");
        auto modes = inifile.findString(n, "MODES", "MDI_MACROS");
        if (!name && !command && !modes) {
            break;
        }
        if (!name || !command || !modes) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "[MDI_MACROS]: macro %d needs a NAME, a COMMAND and MODES, no macros loaded\n", n);
            macros.clear();
            return -1;
        }
        Macro m;
        m.name = *name;
        m.command = *command;
        if (!validName(m.name)) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "[MDI_MACROS]: NAME '%s': use letters, digits, '_' and '-' only, no macros loaded\n",
                m.name.c_str());
            macros.clear();
            return -1;
        }
        if (taskMacroFind(m.name.c_str()) >= 0) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "[MDI_MACROS]: NAME '%s' is used twice, no macros loaded\n", m.name.c_str());
            macros.clear();
            return -1;
        }
        if (m.command.empty() || m.command.size() >= LINELEN) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "[MDI_MACROS]: macro '%s': COMMAND is empty or too long, no macros loaded\n",
                m.name.c_str());
            macros.clear();
            return -1;
        }
        if (!parseModes(*modes, m)) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "[MDI_MACROS]: macro '%s': MODES '%s' must list MANUAL, MDI and/or AUTO, no macros loaded\n",
                m.name.c_str(), modes->c_str());
            macros.clear();
            return -1;
        }
        macros.push_back(m);
    }
    return 0;
}

int taskMacroCount(void)
{
    return (int)macros.size();
}

bool taskMacroHasPin(int n)
{
    return macros[n].pin;
}

const char *taskMacroName(int n)
{
    return macros[n].name.c_str();
}

int taskMacroFind(const char *name)
{
    for (size_t n = 0; n < macros.size(); n++) {
        if (macros[n].name == name) {
            return (int)n;
        }
    }
    return -1;
}

const char *taskMacroCommand(int n)
{
    return macros[n].command.c_str();
}

bool taskMacroAllowed(int n, EMC_TASK_MODE mode)
{
    const Macro &m = macros[n];
    switch (mode) {
    case EMC_TASK_MODE::MANUAL:
        return m.manual;
    case EMC_TASK_MODE::MDI:
        return m.mdi;
    case EMC_TASK_MODE::AUTO:
        return m.autom;
    }
    return false;
}

const char *taskMacroModes(int n)
{
    return macros[n].modes.c_str();
}
