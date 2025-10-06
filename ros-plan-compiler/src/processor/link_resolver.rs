use crate::{
    context::link::{PubSubLinkCtx, PubSubLinkShared, ServiceLinkCtx, ServiceLinkShared},
    error::Error,
    program::Program,
    scope::{ScopeRef, ScopeShared},
};
use itertools::Itertools;
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
        });

        // Perform traversal
        while let Some(job) = self.queue.pop_front() {
            self.visit_scope(program, job)?;
        }

        Ok(())
    }

    fn visit_scope(&mut self, program: &Program, job: Job) -> Result<(), Error> {
        let Job { current: shared } = job;

        shared.with_read(|guard| -> Result<_, Error> {
            let visitor = Visitor::new(program, &shared);

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
            for (_suffix, group) in guard.group() {
                self.queue.push_back(Job {
                    current: group.clone().into(),
                });
            }

            for include in guard.include().values() {
                include.with_read(|guard| {
                    let Some(plan) = &guard.plan else {
                        todo!();
                    };

                    self.queue.push_back(Job {
                        current: plan.clone().into(),
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
}

impl<'a> Visitor<'a> {
    pub fn new(program: &'a Program, current: &'a ScopeShared) -> Self {
        Self { program, current }
    }

    pub fn visit_pubsub_link(&self, link: &mut PubSubLinkCtx) -> Result<(), Error> {
        {
            let src: Result<Vec<_>, _> = link
                .src_key
                .iter()
                .map(|socket_key| {
                    let socket_key = socket_key.get_stored().unwrap();
                    self.program
                        .selector(self.current)
                        .find_plan_or_node_pub(socket_key)
                        .ok_or(socket_key)?
                        .to_node_pub()
                        .ok_or(socket_key)
                })
                .flatten_ok()
                .collect();
            let src = match src {
                Ok(src) => src,
                Err(bad_key) => {
                    bail!(bad_key, "the key does not resolve to a publication socket");
                }
            };
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
            set_or_panic!(link.dst_socket, dst);
        }

        // Derive topic name (F6: single-source derivation)
        link.derived_topic = derive_topic_name(link);

        Ok(())
    }

    fn visit_service_link(&self, link: &mut ServiceLinkCtx) -> Result<(), Error> {
        let listen = {
            let socket_key = link.listen_key.get_stored()?;

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
}

/// F6: Derive topic name from link
/// Priority: 1. explicit `topic` field, 2. single source's ros_name + socket name
/// For Phase 2, we only implement single-source derivation
fn derive_topic_name(link: &PubSubLinkCtx) -> Option<String> {
    // 1. If topic is explicitly set and evaluated, use it
    if let Some(topic_store) = &link.topic {
        if let Ok(topic_text) = topic_store.get_stored() {
            return Some(topic_text.clone());
        }
    }

    // 2. Single-source topic derivation
    let src_sockets = link.src_socket.as_ref()?;
    if src_sockets.len() == 1 {
        let socket = &src_sockets[0];
        return socket.with_read(|socket| {
            // Check if ros_name override exists
            if let Some(ros_name_store) = &socket.ros_name {
                if let Ok(ros_name) = ros_name_store.get_stored() {
                    return Some(ros_name.clone());
                }
            }

            // Get socket name from key (last segment) and parent (node path)
            let (parent_key, socket_name) = socket.key.as_key().split_parent();
            let socket_name = socket_name?;

            // If there's a parent key, use it as namespace, otherwise use socket name only
            if let Some(parent) = parent_key {
                Some(format!("{}/{}", parent.as_str(), socket_name.as_str()))
            } else {
                Some(socket_name.to_string())
            }
        });
    }

    // Multi-source not supported in Phase 2 (deferred to F7 in Phase 3)
    None
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
        let link = PubSubLinkCtx {
            key: "test_link".parse().unwrap(),
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

        let topic = derive_topic_name(&link);
        assert_eq!(topic, Some("/custom/topic".to_string()));
    }

    #[test]
    fn derive_topic_multi_source_returns_none() {
        let link = PubSubLinkCtx {
            key: "test_link".parse().unwrap(),
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

        let topic = derive_topic_name(&link);
        assert!(topic.is_none());
    }

    #[test]
    fn derive_topic_no_source_returns_none() {
        let link = PubSubLinkCtx {
            key: "test_link".parse().unwrap(),
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

        let topic = derive_topic_name(&link);
        assert!(topic.is_none());
    }

    #[test]
    fn derive_topic_explicit_overrides_source() {
        let link = PubSubLinkCtx {
            key: "test_link".parse().unwrap(),
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

        let topic = derive_topic_name(&link);
        assert_eq!(topic, Some("/explicit/topic".to_string()));
    }

    // Additional tests for namespace and ros_name derivation would require
    // proper SharedTable setup. These are verified via integration tests.
}
