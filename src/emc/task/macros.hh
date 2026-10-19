/********************************************************************
* Description: macros.hh
*   MDI commands the integrator lists in [MDI_MACROS], each with the task
*   modes it may run in. A user interface, a panel or a halui pin starts
*   one by name (EMC_TASK_PLAN_MACRO); task checks the selected mode
*   against the list and runs the command as MDI. Free MDI text from a
*   user interface runs only in MDI mode, see emcTaskPlan().
*
* License: GPL Version 2
********************************************************************/
#ifndef TASK_MACROS_HH
#define TASK_MACROS_HH

#include "nml_intf/emc.hh"

namespace linuxcnc { class IniFile; }

/* Read [MDI_MACROS]. Each macro is a NAME, a COMMAND and its MODES, in
   that order; the n-th NAME goes with the n-th COMMAND and MODES. Returns
   0, or -1 with the macros dropped if the section is inconsistent. */
int taskMacrosLoad(linuxcnc::IniFile &inifile);

/* [HALUI]MDI_COMMAND entries load first, as macros mdi-command-NN
   allowed in every mode; halui gives them their own pins */
#define TASK_MACRO_MDI_COMMAND_MAX 64

int taskMacroCount(void);
const char *taskMacroName(int n);
/* false for the MDI_COMMAND macros: no halui.macro.<name> pin */
bool taskMacroHasPin(int n);

/* The macro called name, or -1 */
int taskMacroFind(const char *name);
const char *taskMacroCommand(int n);
bool taskMacroAllowed(int n, EMC_TASK_MODE mode);

/* The modes macro n may run in, for messages: "Manual, MDI" */
const char *taskMacroModes(int n);

#endif /* TASK_MACROS_HH */
