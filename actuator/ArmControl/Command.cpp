#include "Command.h"

ArmControlCommand::ArmControlCommand() {}

void ArmControlCommand::addCommand(
  const char* name,
  std::function<void()> func
) {
  commands.push_back({name, func});
}
/// --- End of addCommand() ---

bool ArmControlCommand::executeCommand(
  const String& rawInput
) {
  for (auto& cmd : commands) {
    if (rawInput == cmd.name) {
      cmd.func();
      return true;
    }
  }
  return false;
}
/// --- End of executeCommand() ---
