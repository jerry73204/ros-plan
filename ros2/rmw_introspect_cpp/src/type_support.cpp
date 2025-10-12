#include "rmw_introspect/type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include <sstream>

namespace rmw_introspect {

std::string extract_message_type(const rosidl_message_type_support_t * type_support)
{
  if (!type_support) {
    return "unknown/msg/Unknown";
  }

  // Get the members directly - the type_support->data points to MessageMembers
  const auto * members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    type_support->data);

  if (!members) {
    return "unknown/msg/Unknown";
  }

  // Format: package_name/msg/MessageName
  // Note: message_namespace_ contains "package_name::msg" in C++ format
  // We need to convert it to "package_name/msg/MessageName"
  std::string ns = members->message_namespace_;

  // Replace "::" with "/"
  size_t pos = ns.find("::");
  if (pos != std::string::npos) {
    ns.replace(pos, 2, "/");
  }

  std::ostringstream oss;
  oss << ns << "/" << members->message_name_;
  return oss.str();
}

std::string extract_service_type(const rosidl_service_type_support_t * type_support)
{
  if (!type_support) {
    return "unknown/srv/Unknown";
  }

  // Get the members directly - the type_support->data points to ServiceMembers
  const auto * members =
    static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    type_support->data);

  if (!members) {
    return "unknown/srv/Unknown";
  }

  // Format: package_name/srv/ServiceName
  // Note: service_namespace_ contains "package_name::srv" in C++ format
  // We need to convert it to "package_name/srv/ServiceName"
  std::string ns = members->service_namespace_;

  // Replace "::" with "/"
  size_t pos = ns.find("::");
  if (pos != std::string::npos) {
    ns.replace(pos, 2, "/");
  }

  std::ostringstream oss;
  oss << ns << "/" << members->service_name_;
  return oss.str();
}

}  // namespace rmw_introspect
