#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include <cstring>

extern "C" {

// QoS event stubs - minimal implementation
// Events are not needed for introspection

rmw_ret_t rmw_publisher_event_init(
  rmw_event_t * rmw_event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  if (!rmw_event) {
    RMW_SET_ERROR_MSG("rmw_event is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (!publisher) {
    RMW_SET_ERROR_MSG("publisher is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Create minimal event structure
  std::memset(rmw_event, 0, sizeof(*rmw_event));
  rmw_event->event_type = event_type;
  rmw_event->data = nullptr;

  return RMW_RET_OK;
}

rmw_ret_t rmw_subscription_event_init(
  rmw_event_t * rmw_event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  if (!rmw_event) {
    RMW_SET_ERROR_MSG("rmw_event is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (!subscription) {
    RMW_SET_ERROR_MSG("subscription is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Create minimal event structure
  std::memset(rmw_event, 0, sizeof(*rmw_event));
  rmw_event->event_type = event_type;
  rmw_event->data = nullptr;

  return RMW_RET_OK;
}

rmw_ret_t rmw_take_event(
  const rmw_event_t * event_handle,
  void * event_info,
  bool * taken)
{
  (void)event_handle;
  (void)event_info;

  if (!taken) {
    RMW_SET_ERROR_MSG("taken is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // No events available in introspection mode
  *taken = false;
  return RMW_RET_OK;
}

rmw_ret_t rmw_event_set_callback(
  rmw_event_t * event,
  rmw_event_callback_t callback,
  const void * user_data)
{
  (void)event;
  (void)callback;
  (void)user_data;

  // No-op: callbacks not supported in introspection mode
  return RMW_RET_OK;
}

}  // extern "C"
