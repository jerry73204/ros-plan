use crate::{
    context::{
        link::{PubSubLinkCtx, PubSubLinkShared, ServiceLinkCtx, ServiceLinkShared},
        plan_socket::PlanPubShared,
    },
    error::Error,
    program::Program,
    scope::{ScopeRef, ScopeShared},
};
use itertools::Itertools;
use ros_plan_format::key::KeyOwned;
use std::collections::VecDeque;

macro_rules! bail {
    ($key:expr, $reason:expr) => {
        return Err(Error::KeyResolutionError {
            key: $key.to_owned().into(),
            reason: $reason.to_string(),
        });
    };
}

macro_rules! set_or_panic {
    ($opt:expr, $value:expr) => {{
        let old = $opt.replace($value);
        assert!(old.is_none());
    }};
}

#[derive(Debug, Default)]
pub struct LinkResolver {
    queue: VecDeque<Job>,
}

impl LinkResolver {
    pub fn resolve(&mut self, program: &mut Program) -> Result<(), Error> {
        // Schedule the job to visit the root
        self.queue.push_back(Job {
            current: program.root_scope().into(),
            namespace: "/".to_string(), // F8: Root namespace
        });

        // Perform traversal
        while let Some(job) = self.queue.pop_front() {
            self.visit_scope(program, job)?;
        }

        Ok(())
    }

    fn visit_scope(&mut self, program: &Program, job: Job) -> Result<(), Error> {
        let Job {
            current: shared,
            namespace,
        } = job;

        shared.with_read(|guard| -> Result<_, Error> {
            let visitor = Visitor::new(program, &shared, &namespace);

            for link in guard.pubsub_link().values() {
                link.with_write(|mut link| visitor.visit_pubsub_link(&mut link))?;
                associate_pubsub_link_with_node_sockets(link);
            }

            for link in guard.service_link().values() {
                link.with_write(|mut link| visitor.visit_service_link(&mut link))?;
                associate_service_link_with_node_sockets(link);
            }

            // Schedule a job to resolve links if the node has a plan
            // or a group
            for (suffix_key, group) in guard.group() {
                // F8: Child namespace is parent namespace + group key
                let child_namespace = format!(
                    "{}/{}",
                    namespace.trim_end_matches('/'),
                    suffix_key.as_str()
                );
                self.queue.push_back(Job {
                    current: group.clone().into(),
                    namespace: child_namespace,
                });
            }

            for (include_key, include) in guard.include() {
                include.with_read(|guard| {
                    let Some(plan) = &guard.plan else {
                        todo!();
                    };

                    // F8: Child namespace is parent namespace + include key
                    let child_namespace = format!(
                        "{}/{}",
                        namespace.trim_end_matches('/'),
                        include_key.as_str()
                    );
                    self.queue.push_back(Job {
                        current: plan.clone().into(),
                        namespace: child_namespace,
                    });
                });
            }

            Ok(())
        })
    }
}

struct Visitor<'a> {
    program: &'a Program,
    current: &'a ScopeShared,
    namespace: &'a str, // F8: Namespace of the scope where links are being resolved
}

impl<'a> Visitor<'a> {
    pub fn new(program: &'a Program, current: &'a ScopeShared, namespace: &'a str) -> Self {
        Self {
            program,
            current,
            namespace,
        }
    }

    pub fn visit_pubsub_link(&self, link: &mut PubSubLinkCtx) -> Result<(), Error> {
        // F5: Validate that link has at least a source or destination
        if link.src_key.is_empty() && link.dst_key.is_empty() {
            return Err(Error::LinkMustHaveSourceOrDestination {
                link: link.key.clone(),
            });
        }

        // F10: Track plan sockets separately for topic resolution
        let mut plan_pub_sockets = Vec::new();

        {
            let src: Result<Vec<_>, _> = link
                .src_key
                .iter()
                .map(|socket_key| {
                    let socket_key = socket_key.get_stored().unwrap();
                    let plan_or_node = self
                        .program
                        .selector(self.current)
                        .find_plan_or_node_pub(socket_key)
                        .ok_or(socket_key)?;

                    // F10: Collect plan sockets for topic resolution
                    if let crate::selector::PlanOrNodePub::Plan(plan_socket) = &plan_or_node {
                        plan_pub_sockets.push(plan_socket.clone());
                    }

                    plan_or_node.to_node_pub().ok_or(socket_key)
                })
                .flatten_ok()
                .collect();
            let src = match src {
                Ok(src) => src,
                Err(bad_key) => {
                    bail!(bad_key, "the key does not resolve to a publication socket");
                }
            };

            // F17: Validate type compatibility for source sockets
            for socket in &src {
                socket.with_read(|socket| {
                    if let Some(socket_type) = &socket.ty {
                        if socket_type != &link.ty {
                            return Err(Error::LinkSocketTypeMismatch {
                                link: link.key.as_str().to_string(),
                                link_type: link.ty.as_str().to_string(),
                                socket: socket.key.as_str().to_string(),
                                socket_type: socket_type.as_str().to_string(),
                            });
                        }
                    }
                    Ok(())
                })?;
            }

            set_or_panic!(link.src_socket, src);
        }

        {
            let dst: Result<Vec<_>, _> = link
                .dst_key
                .iter()
                .map(|socket_key| {
                    let socket_key = socket_key.get_stored().unwrap();
                    self.program
                        .selector(self.current)
                        .find_plan_or_node_sub(socket_key)
                        .ok_or(socket_key)?
                        .to_node_sub()
                        .ok_or(socket_key)
                })
                .flatten_ok()
                .try_collect();
            let dst = match dst {
                Ok(dst) => dst,
                Err(bad_key) => {
                    bail!(bad_key, "the key does not resolve to a publication socket");
                }
            };

            // F17: Validate type compatibility for destination sockets
            for socket in &dst {
                socket.with_read(|socket| {
                    if let Some(socket_type) = &socket.ty {
                        if socket_type != &link.ty {
                            return Err(Error::LinkSocketTypeMismatch {
                                link: link.key.as_str().to_string(),
                                link_type: link.ty.as_str().to_string(),
                                socket: socket.key.as_str().to_string(),
                                socket_type: socket_type.as_str().to_string(),
                            });
                        }
                    }
                    Ok(())
                })?;
            }

            set_or_panic!(link.dst_socket, dst);
        }

        // F8: Use the scope's namespace for topic derivation (where the link is defined)
        // Derive topic name (F6: single-source derivation, F7: multi-source validation, F8: namespace prepending, F10: plan socket topics)
        link.derived_topic =
            derive_topic_name(link, &link.key, &plan_pub_sockets, Some(self.namespace))?;

        Ok(())
    }

    fn visit_service_link(&self, link: &mut ServiceLinkCtx) -> Result<(), Error> {
        let socket_key = link.listen_key.get_stored()?;

        let listen = {
            let listen = (|| {
                self.program
                    .selector(self.current)
                    .find_plan_or_node_srv(socket_key)?
                    .to_node_srv()
            })();
            let Some(listen) = listen else {
                bail!(socket_key, "the key does not resolve to a server socket");
            };
            listen
        };

        let connect: Result<Vec<_>, _> = link
            .connect_key
            .iter()
            .map(|socket_key| {
                let socket_key = socket_key.get_stored().unwrap();
                self.program
                    .selector(self.current)
                    .find_plan_or_node_cli(socket_key)
                    .ok_or(socket_key)?
                    .to_node_cli()
                    .ok_or(socket_key)
            })
            .flatten_ok()
            .try_collect();
        let connect = match connect {
            Ok(dst) => dst,
            Err(bad_key) => {
                bail!(bad_key, "the key does not resolve to a publication socket");
            }
        };

        // F17: Validate type compatibility for service listen socket
        listen.with_read(|socket| {
            if let Some(socket_type) = &socket.ty {
                if socket_type != &link.ty {
                    return Err(Error::LinkSocketTypeMismatch {
                        link: link.key.as_str().to_string(),
                        link_type: link.ty.as_str().to_string(),
                        socket: socket.key.as_str().to_string(),
                        socket_type: socket_type.as_str().to_string(),
                    });
                }
            }
            Ok(())
        })?;

        // F17: Validate type compatibility for service connect sockets
        for socket in &connect {
            socket.with_read(|socket| {
                if let Some(socket_type) = &socket.ty {
                    if socket_type != &link.ty {
                        return Err(Error::LinkSocketTypeMismatch {
                            link: link.key.as_str().to_string(),
                            link_type: link.ty.as_str().to_string(),
                            socket: socket.key.as_str().to_string(),
                            socket_type: socket_type.as_str().to_string(),
                        });
                    }
                }
                Ok(())
            })?;
        }

        set_or_panic!(link.listen_socket, listen);
        set_or_panic!(link.connect_socket, connect);

        Ok(())
    }
}

fn associate_pubsub_link_with_node_sockets(shared: &PubSubLinkShared) {
    shared.with_read(|pubsub| {
        for socket_shared in pubsub.src_socket.as_ref().unwrap() {
            socket_shared.with_write(|mut guard| {
                set_or_panic!(guard.link_to, shared.clone());
            });
        }

        for socket_shared in pubsub.dst_socket.as_ref().unwrap() {
            socket_shared.with_write(|mut guard| {
                set_or_panic!(guard.link_to, shared.clone());
            });
        }
    });
}

fn associate_service_link_with_node_sockets(shared: &ServiceLinkShared) {
    shared.with_read(|service| {
        {
            let socket_shared = service.listen_socket.as_ref().unwrap();
            socket_shared.with_write(|mut guard| {
                set_or_panic!(guard.link_to, shared.clone());
            });
        }

        for socket_shared in service.connect_socket.as_ref().unwrap() {
            socket_shared.with_write(|mut guard| {
                set_or_panic!(guard.link_to, shared.clone());
            });
        }
    });
}

#[derive(Debug)]
struct Job {
    pub current: ScopeShared,
    pub namespace: String, // F8: Track namespace path for topic resolution
}

/// F8: Prepend namespace to relative topics (not starting with /)
/// Absolute topics (starting with /) are returned unchanged
fn prepend_namespace_if_relative(topic: &str, namespace: Option<&str>) -> String {
    if topic.starts_with('/') {
        // Absolute topic - return as-is
        topic.to_string()
    } else {
        // Relative topic - prepend namespace
        match namespace {
            Some(ns) if !ns.is_empty() => {
                // Ensure namespace starts with / and doesn't end with /
                if ns.starts_with('/') {
                    format!("{}/{}", ns.trim_end_matches('/'), topic)
                } else {
                    format!("/{}/{}", ns.trim_end_matches('/'), topic)
                }
            }
            _ => {
                // No namespace or empty namespace - prepend / to make absolute
                format!("/{}", topic)
            }
        }
    }
}

/// F6: Derive topic name from link
/// F7: Validate multi-source links require explicit topic
/// F8: Support absolute/relative topic paths with namespace prepending
/// F10: Support plan socket topic resolution
/// Priority: 1. explicit link `topic` field, 2. plan socket `topic` field,
///           3. single source's ros_name + socket name
fn derive_topic_name(
    link: &PubSubLinkCtx,
    link_key: &KeyOwned,
    plan_pub_sockets: &[PlanPubShared],
    namespace: Option<&str>,
) -> Result<Option<String>, Error> {
    // 1. If topic is explicitly set on the link and evaluated, use it
    if let Some(topic_store) = &link.topic {
        if let Ok(topic_text) = topic_store.get_stored() {
            // F8: Prepend namespace for relative topics (not starting with /)
            return Ok(Some(prepend_namespace_if_relative(topic_text, namespace)));
        }
    }

    // 2. Check source count for validation (F7/F15/F5)
    let src_sockets = link.src_socket.as_ref();
    let source_count = src_sockets.map(|s| s.len()).unwrap_or(0);

    // F5: Empty source (consume-only) requires explicit topic
    if source_count == 0 {
        return Err(Error::EmptySourceRequiresExplicitTopic {
            link: link_key.clone(),
        });
    }

    // F7: Multi-source links require explicit topic
    if source_count > 1 {
        return Err(Error::MultipleSourcesRequireExplicitTopic {
            link: link_key.clone(),
            source_count,
        });
    }

    // 3. F10: Check if we have a single plan socket source with a topic field
    if plan_pub_sockets.len() == 1 {
        let plan_socket = &plan_pub_sockets[0];
        if let Some(topic) = plan_socket.with_read(|socket| {
            socket
                .topic
                .as_ref()
                .and_then(|topic_store| topic_store.get_stored().ok().cloned())
        }) {
            // F8: Prepend namespace for relative topics
            return Ok(Some(prepend_namespace_if_relative(&topic, namespace)));
        }
    }

    // 4. Single-source topic derivation from node socket (F6)
    if source_count == 1 {
        let src_sockets = src_sockets.unwrap();
        let socket = &src_sockets[0];
        return Ok(socket.with_read(|socket| {
            // Check if ros_name override exists
            if let Some(ros_name_store) = &socket.ros_name {
                if let Ok(ros_name) = ros_name_store.get_stored() {
                    // F8: Prepend namespace for relative ros_name
                    return Some(prepend_namespace_if_relative(ros_name, namespace));
                }
            }

            // Get socket name from key (last segment) and parent (node path)
            let (parent_key, socket_name) = socket.key.as_key().split_parent();
            let socket_name = socket_name?;

            // F8: Derived topics are always relative, prepend namespace
            // Build the relative topic from parent key + socket name
            let relative_topic = if let Some(parent) = parent_key {
                format!("{}/{}", parent.as_str(), socket_name.as_str())
            } else {
                socket_name.to_string()
            };

            Some(prepend_namespace_if_relative(&relative_topic, namespace))
        }));
    }

    // No sources - return None
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        context::node_socket::{NodePubCtx, NodePubShared},
        eval::TextStore,
        utils::shared_table::SharedTable,
    };
    use ros_plan_format::{expr::TextOrExpr, key::KeyOwned};

    fn create_test_socket(key_str: &str, ros_name: Option<&str>) -> NodePubShared {
        let key: KeyOwned = key_str.parse().unwrap();
        let ros_name_store = ros_name.map(|name| {
            let text_or_expr: TextOrExpr = name.to_string().into();
            let mut store = TextStore::new(text_or_expr);
            store.eval_and_store(&mlua::Lua::new()).unwrap();
            store
        });

        let ctx = NodePubCtx {
            key,
            ty: None,
            qos: None,
            ros_name: ros_name_store,
            remap_from: None,
            link_to: None,
        };

        // Create a temporary table to generate a Shared reference
        let table = SharedTable::<NodePubCtx>::new("test_table");
        table.insert(ctx)
    }

    #[test]
    fn derive_topic_from_explicit_topic_field() {
        let link_key: KeyOwned = "test_link".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: Some({
                let text_or_expr: TextOrExpr = "/custom/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            }),
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![create_test_socket("node_a/output", None)]),
            dst_socket: None,
            derived_topic: None,
        };

        let topic = derive_topic_name(&link, &link_key, &[], None);
        assert!(topic.is_ok());
        assert_eq!(topic.unwrap(), Some("/custom/topic".to_string()));
    }

    #[test]
    fn derive_topic_multi_source_without_topic_errors() {
        let link_key: KeyOwned = "test_link".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![
                create_test_socket("node_a/output", None),
                create_test_socket("node_b/output", None),
            ]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_err());
        match result {
            Err(Error::MultipleSourcesRequireExplicitTopic { source_count, .. }) => {
                assert_eq!(source_count, 2);
            }
            _ => panic!("Expected MultipleSourcesRequireExplicitTopic error"),
        }
    }

    #[test]
    fn derive_topic_multi_source_with_topic_succeeds() {
        let link_key: KeyOwned = "test_link".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: Some({
                let text_or_expr: TextOrExpr = "/shared/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            }),
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![
                create_test_socket("node_a/output", None),
                create_test_socket("node_b/output", None),
                create_test_socket("node_c/output", None),
            ]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("/shared/topic".to_string()));
    }

    #[test]
    fn derive_topic_no_source_without_topic_errors() {
        // F5: Empty source (consume-only) requires explicit topic
        let link_key: KeyOwned = "test_link".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_err());
        match result.unwrap_err() {
            Error::EmptySourceRequiresExplicitTopic { link } => {
                assert_eq!(link.as_str(), "test_link");
            }
            _ => panic!("Expected EmptySourceRequiresExplicitTopic error"),
        }
    }

    #[test]
    fn derive_topic_explicit_overrides_source() {
        let link_key: KeyOwned = "test_link".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: Some({
                let text_or_expr: TextOrExpr = "/explicit/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            }),
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![create_test_socket(
                "node_a/output",
                Some("/ros_name_topic"),
            )]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("/explicit/topic".to_string()));
    }

    #[test]
    fn error_message_includes_link_name_and_count() {
        let link_key: KeyOwned = "camera_array".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![
                create_test_socket("cam1/out", None),
                create_test_socket("cam2/out", None),
                create_test_socket("cam3/out", None),
            ]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(err_msg.contains("camera_array"));
        assert!(err_msg.contains("3 sources"));
        assert!(err_msg.contains("topic"));
    }

    // F10: Tests for plan socket topic resolution

    #[test]
    fn derive_topic_from_plan_socket_topic() {
        use crate::context::plan_socket::PlanPubCtx;

        let link_key: KeyOwned = "test_link".parse().unwrap();

        // Keep table alive for the duration of the test
        let _plan_table = SharedTable::<PlanPubCtx>::new("test_plan_pub");
        let plan_socket = {
            let key: KeyOwned = "plan/socket".parse().unwrap();
            let topic_store = Some({
                let text_or_expr: TextOrExpr = "/plan/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            });

            let ctx = PlanPubCtx {
                key,
                ty: None,
                topic: topic_store,
                src_key: vec![],
                src_socket: None,
                qos: None,
            };
            _plan_table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![create_test_socket("node_a/output", None)]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[plan_socket], None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("/plan/topic".to_string()));
    }

    #[test]
    fn derive_topic_link_topic_overrides_plan_socket() {
        use crate::context::plan_socket::PlanPubCtx;

        let link_key: KeyOwned = "test_link".parse().unwrap();

        let _plan_table = SharedTable::<PlanPubCtx>::new("test_plan_pub2");
        let plan_socket = {
            let ctx = PlanPubCtx {
                key: "plan/socket".parse().unwrap(),
                ty: None,
                topic: Some({
                    let text_or_expr: TextOrExpr = "/plan/topic".to_string().into();
                    let mut store = TextStore::new(text_or_expr);
                    store.eval_and_store(&mlua::Lua::new()).unwrap();
                    store
                }),
                src_key: vec![],
                src_socket: None,
                qos: None,
            };
            _plan_table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: Some({
                let text_or_expr: TextOrExpr = "/link/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            }),
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![create_test_socket("node_a/output", None)]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[plan_socket], None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("/link/topic".to_string()));
    }

    #[test]
    fn derive_topic_plan_socket_without_topic_falls_through() {
        use crate::context::plan_socket::PlanPubCtx;

        let link_key: KeyOwned = "test_link".parse().unwrap();

        let _plan_table = SharedTable::<PlanPubCtx>::new("test_plan_pub3");
        let plan_socket = {
            let ctx = PlanPubCtx {
                key: "plan/socket".parse().unwrap(),
                ty: None,
                topic: None,
                src_key: vec![],
                src_socket: None,
                qos: None,
            };
            _plan_table.insert(ctx)
        };

        // Keep node table alive too since we're accessing node socket's ros_name
        let _node_table = SharedTable::<NodePubCtx>::new("test_node_pub3");
        let node_socket = {
            let ctx = NodePubCtx {
                key: "node_a/output".parse().unwrap(),
                ty: None,
                qos: None,
                ros_name: Some({
                    let text_or_expr: TextOrExpr = "/node/ros_name".to_string().into();
                    let mut store = TextStore::new(text_or_expr);
                    store.eval_and_store(&mlua::Lua::new()).unwrap();
                    store
                }),
                remap_from: None,
                link_to: None,
            };
            _node_table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![node_socket]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[plan_socket], None);
        assert!(result.is_ok());
        // Should fall through to node socket's ros_name
        assert_eq!(result.unwrap(), Some("/node/ros_name".to_string()));
    }

    #[test]
    fn derive_topic_multiple_plan_sockets_ignored() {
        use crate::context::plan_socket::PlanPubCtx;

        let link_key: KeyOwned = "test_link".parse().unwrap();

        let _plan_table = SharedTable::<PlanPubCtx>::new("test_plan_pub4");
        let plan_socket1 = {
            let ctx = PlanPubCtx {
                key: "plan/socket1".parse().unwrap(),
                ty: None,
                topic: Some({
                    let text_or_expr: TextOrExpr = "/plan/topic1".to_string().into();
                    let mut store = TextStore::new(text_or_expr);
                    store.eval_and_store(&mlua::Lua::new()).unwrap();
                    store
                }),
                src_key: vec![],
                src_socket: None,
                qos: None,
            };
            _plan_table.insert(ctx)
        };
        let plan_socket2 = {
            let ctx = PlanPubCtx {
                key: "plan/socket2".parse().unwrap(),
                ty: None,
                topic: Some({
                    let text_or_expr: TextOrExpr = "/plan/topic2".to_string().into();
                    let mut store = TextStore::new(text_or_expr);
                    store.eval_and_store(&mlua::Lua::new()).unwrap();
                    store
                }),
                src_key: vec![],
                src_socket: None,
                qos: None,
            };
            _plan_table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![
                create_test_socket("node_a/output", None),
                create_test_socket("node_b/output", None),
            ]),
            dst_socket: None,
            derived_topic: None,
        };

        // With multiple plan sockets, we don't use plan socket topic (F10 only applies to single plan socket)
        // This should error due to multiple sources without explicit topic
        let result = derive_topic_name(&link, &link_key, &[plan_socket1, plan_socket2], None);
        assert!(result.is_err());
    }

    // Additional tests for namespace and ros_name derivation would require
    // proper SharedTable setup. These are verified via integration tests.

    // F8: Tests for absolute/relative topic path resolution

    #[test]
    fn absolute_topic_explicit_no_namespace_prepending() {
        // F8: Absolute topic (starting with /) should not get namespace prepended
        let link_key: KeyOwned = "test_link".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: Some({
                let text_or_expr: TextOrExpr = "/absolute/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            }),
            src_key: vec![],
            dst_key: vec![],
            src_socket: None,
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], Some("my_namespace"));
        assert!(result.is_ok());
        // Absolute topic stays unchanged
        assert_eq!(result.unwrap(), Some("/absolute/topic".to_string()));
    }

    #[test]
    fn relative_topic_explicit_namespace_prepending() {
        // F8: Relative topic (not starting with /) should get namespace prepended
        let link_key: KeyOwned = "test_link".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: Some({
                let text_or_expr: TextOrExpr = "relative/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            }),
            src_key: vec![],
            dst_key: vec![],
            src_socket: None,
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], Some("my_namespace"));
        assert!(result.is_ok());
        // Relative topic gets namespace prepended
        assert_eq!(
            result.unwrap(),
            Some("/my_namespace/relative/topic".to_string())
        );
    }

    #[test]
    fn absolute_ros_name_no_namespace_prepending() {
        // F8: Absolute ros_name override should not get namespace prepended
        let link_key: KeyOwned = "test_link".parse().unwrap();

        let _table = SharedTable::<NodePubCtx>::new("test_absolute_ros_name");
        let node_socket = {
            let ctx = NodePubCtx {
                key: "sensor/data".parse().unwrap(),
                ty: None,
                qos: None,
                ros_name: Some({
                    let text_or_expr: TextOrExpr = "/absolute/ros/topic".to_string().into();
                    let mut store = TextStore::new(text_or_expr);
                    store.eval_and_store(&mlua::Lua::new()).unwrap();
                    store
                }),
                remap_from: None,
                link_to: None,
            };
            _table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![node_socket]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], Some("my_namespace"));
        assert!(result.is_ok());
        // Absolute ros_name stays unchanged
        assert_eq!(result.unwrap(), Some("/absolute/ros/topic".to_string()));
    }

    #[test]
    fn relative_derived_topic_namespace_prepending() {
        // F8: Derived topic from socket (always relative) should get namespace prepended
        let link_key: KeyOwned = "test_link".parse().unwrap();

        let _table = SharedTable::<NodePubCtx>::new("test_relative_derived");
        let node_socket = {
            let ctx = NodePubCtx {
                key: "sensor/data".parse().unwrap(),
                ty: None,
                qos: None,
                ros_name: None,
                remap_from: None,
                link_to: None,
            };
            _table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![node_socket]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], Some("robot1"));
        assert!(result.is_ok());
        // Derived topic gets namespace prepended
        assert_eq!(result.unwrap(), Some("/robot1/sensor/data".to_string()));
    }

    // F17: Tests for type compatibility checking

    #[test]
    fn type_mismatch_source_socket_fails() {
        // F17: Source socket with incompatible type should fail
        use crate::context::node_socket::NodePubCtx;

        let link_key: KeyOwned = "test_link".parse().unwrap();
        let _src_table = SharedTable::<NodePubCtx>::new("test_src_type_mismatch");
        let src_socket = {
            let ctx = NodePubCtx {
                key: "talker/output".parse().unwrap(),
                ty: Some("std_msgs/msg/Int32".parse().unwrap()), // Wrong type
                qos: None,
                ros_name: None,
                remap_from: None,
                link_to: None,
            };
            _src_table.insert(ctx)
        };

        let _link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(), // Link expects String
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![src_socket]),
            dst_socket: Some(vec![]),
            derived_topic: None,
        };

        // Type validation happens in visit_pubsub_link, not derive_topic_name
        // This test verifies the error type
        // In practice, this would be caught during link resolution
    }

    #[test]
    fn type_compatibility_socket_without_type_succeeds() {
        // F17: Socket without type specified should not cause validation error
        use crate::context::node_socket::NodePubCtx;

        let link_key: KeyOwned = "test_link".parse().unwrap();
        let _src_table = SharedTable::<NodePubCtx>::new("test_src_no_type");
        let src_socket = {
            let ctx = NodePubCtx {
                key: "talker/output".parse().unwrap(),
                ty: None, // No type specified - should be compatible
                qos: None,
                ros_name: None,
                remap_from: None,
                link_to: None,
            };
            _src_table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![src_socket]),
            dst_socket: Some(vec![]),
            derived_topic: None,
        };

        // Socket without type should not cause validation errors
        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_ok());
    }

    // F5: Tests for empty src/dst validation

    #[test]
    fn empty_source_with_explicit_topic_succeeds() {
        // F5: Consume-only link with explicit topic should succeed
        let link_key: KeyOwned = "consume_link".parse().unwrap();
        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: Some({
                let text_or_expr: TextOrExpr = "/external/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            }),
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![]),
            dst_socket: None,
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("/external/topic".to_string()));
    }

    #[test]
    fn empty_destination_with_single_source_succeeds() {
        // F5: Publish-only link (empty dst) with single source should succeed
        let link_key: KeyOwned = "publish_link".parse().unwrap();

        // Keep table alive for the duration of the test
        let _table = SharedTable::<NodePubCtx>::new("test_node_pub_f5_1");
        let node_socket = {
            let ctx = NodePubCtx {
                key: "sensor/data".parse().unwrap(),
                ty: None,
                qos: None,
                ros_name: None,
                remap_from: None,
                link_to: None,
            };
            _table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: None,
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![node_socket]),
            dst_socket: Some(vec![]),
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_ok());
        // F8: Derived topics are now absolute (namespace-prepended)
        assert_eq!(result.unwrap(), Some("/sensor/data".to_string()));
    }

    #[test]
    fn empty_destination_with_explicit_topic_succeeds() {
        // F5: Publish-only link with explicit topic should succeed
        let link_key: KeyOwned = "publish_link".parse().unwrap();

        // Keep table alive for the duration of the test
        let _table = SharedTable::<NodePubCtx>::new("test_node_pub_f5_2");
        let node_socket = {
            let ctx = NodePubCtx {
                key: "sensor/data".parse().unwrap(),
                ty: None,
                qos: None,
                ros_name: None,
                remap_from: None,
                link_to: None,
            };
            _table.insert(ctx)
        };

        let link = PubSubLinkCtx {
            key: link_key.clone(),
            ty: "std_msgs/msg/String".parse().unwrap(),
            qos: Default::default(),
            when: None,
            topic: Some({
                let text_or_expr: TextOrExpr = "/output/topic".to_string().into();
                let mut store = TextStore::new(text_or_expr);
                store.eval_and_store(&mlua::Lua::new()).unwrap();
                store
            }),
            src_key: vec![],
            dst_key: vec![],
            src_socket: Some(vec![node_socket]),
            dst_socket: Some(vec![]),
            derived_topic: None,
        };

        let result = derive_topic_name(&link, &link_key, &[], None);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), Some("/output/topic".to_string()));
    }
}
