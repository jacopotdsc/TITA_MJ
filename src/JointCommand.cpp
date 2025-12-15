#include <JointCommand.hpp>
#include <stdexcept>

namespace labrob {

double
JointCommand::operator[](const std::string& key) const {
  auto it = joint_command_.find(key);
  if (it == joint_command_.end()) {
    return 0.0;
  }
  return it->second;
}

double&
JointCommand::operator[](const std::string& key) {
  return joint_command_[key];
}

JointCommand::JointCommandMap::iterator
JointCommand::begin() {
  return joint_command_.begin();
}

JointCommand::JointCommandMap::iterator
JointCommand::end() {
  return joint_command_.end();
}

} // end namespace labrob