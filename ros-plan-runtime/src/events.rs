use ros_plan_format::key::KeyOwned;
use serde::{Deserialize, Serialize};
use std::time::SystemTime;

/// Runtime event for logging
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuntimeEvent {
    /// When the event occurred
    pub timestamp: SystemTime,
    /// Type of event
    pub event_type: EventType,
    /// Associated node (if applicable)
    pub node_id: Option<KeyOwned>,
    /// Event message
    pub message: String,
}

/// Types of runtime events
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum EventType {
    /// Runtime started
    RuntimeStart,
    /// Runtime stopped
    RuntimeStop,
    /// Node started
    NodeStart,
    /// Node stopped
    NodeStop,
    /// Node crashed
    NodeCrash,
    /// Node restarted
    NodeRestart,
    /// Parameter updated
    ParameterUpdate,
    /// Launch include reloaded
    LaunchReload,
    /// Error occurred
    Error,
}

/// Event logger for runtime events
#[derive(Debug, Clone)]
pub struct EventLog {
    events: Vec<RuntimeEvent>,
    max_events: usize,
}

impl EventLog {
    /// Create new event log with maximum size
    pub fn new(max_events: usize) -> Self {
        Self {
            events: Vec::new(),
            max_events,
        }
    }

    /// Log an event
    pub fn log(&mut self, event_type: EventType, node_id: Option<KeyOwned>, message: String) {
        let event = RuntimeEvent {
            timestamp: SystemTime::now(),
            event_type,
            node_id,
            message,
        };

        self.events.push(event);

        // Trim to max size
        if self.events.len() > self.max_events {
            self.events.remove(0);
        }
    }

    /// Get all events
    pub fn events(&self) -> &[RuntimeEvent] {
        &self.events
    }

    /// Get events by type
    pub fn events_by_type(&self, event_type: &EventType) -> Vec<&RuntimeEvent> {
        self.events
            .iter()
            .filter(|e| &e.event_type == event_type)
            .collect()
    }

    /// Get recent events (last N)
    pub fn recent(&self, count: usize) -> &[RuntimeEvent] {
        let start = self.events.len().saturating_sub(count);
        &self.events[start..]
    }

    /// Clear all events
    pub fn clear(&mut self) {
        self.events.clear();
    }
}

impl Default for EventLog {
    fn default() -> Self {
        Self::new(1000) // Default to 1000 events
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_event_log_creation() {
        let log = EventLog::new(10);
        assert_eq!(log.events().len(), 0);
    }

    #[test]
    fn test_log_event() {
        let mut log = EventLog::new(10);
        log.log(
            EventType::NodeStart,
            Some("test_node".parse().unwrap()),
            "Node started".to_string(),
        );

        assert_eq!(log.events().len(), 1);
        assert_eq!(log.events()[0].event_type, EventType::NodeStart);
    }

    #[test]
    fn test_max_events() {
        let mut log = EventLog::new(3);

        for i in 0..5 {
            log.log(EventType::NodeStart, None, format!("Event {}", i));
        }

        assert_eq!(log.events().len(), 3);
        assert!(log.events()[0].message.contains("Event 2"));
    }

    #[test]
    fn test_filter_by_type() {
        let mut log = EventLog::new(10);
        log.log(EventType::NodeStart, None, "Start".to_string());
        log.log(EventType::NodeStop, None, "Stop".to_string());
        log.log(EventType::NodeStart, None, "Start 2".to_string());

        let starts = log.events_by_type(&EventType::NodeStart);
        assert_eq!(starts.len(), 2);
    }

    #[test]
    fn test_recent_events() {
        let mut log = EventLog::new(10);
        for i in 0..5 {
            log.log(EventType::NodeStart, None, format!("Event {}", i));
        }

        let recent = log.recent(2);
        assert_eq!(recent.len(), 2);
        assert!(recent[0].message.contains("Event 3"));
        assert!(recent[1].message.contains("Event 4"));
    }
}
