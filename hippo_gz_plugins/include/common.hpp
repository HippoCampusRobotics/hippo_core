#include <sdf/Element.hh>
#include <ignition/common.hh>

template<typename T>
bool AssignSdfParam(const std::shared_ptr<const sdf::Element> &_sdf, std::string name, T &var) {
  bool success;
  std::tie(var, success) = _sdf->Get<T>(name, var);
  if (!success) {
    ignwarn << "No value set for [" << name << "]. Using default value." << std::endl;
  }
  return success;
}
