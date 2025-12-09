#ifndef ARM_CONTROL_COMMAND_H
#define ARM_CONTROL_COMMAND_H

#include <vector>
#include <functional>
#include "CommonDefines.h"

/* === Command Table API === */
struct CommandEntry {
  const char* name;
  std::function<void()> func;
};

class ArmControlCommand {
public:
  ArmControlCommand();
  void addCommand(
    const char* name,
    std::function<void()> func
  );
  bool executeCommand(
    const String& rawInput
  );

private:
  std::vector<CommandEntry> commands;
};

#endif
