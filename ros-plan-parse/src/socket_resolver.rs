use crate::{
    context::{
        node::NodeArc,
        socket::{
            PubSocketContext, QuerySocketContext, ServerSocketContext, SocketArc, SocketContext,
            SubSocketContext,
        },
        uri::NodeTopicUri,
    },
    error::Error,
    resource::{PlanContext, PlanResource, TrieContext, TrieRef},
};
use itertools::Itertools;
use ros_plan_format::{
    key::{Key, KeyOwned},
    link::TopicUri,
};
use std::collections::VecDeque;

macro_rules! bail_resolve_key_error {
    ($key:expr, $reason:expr) => {
        return Err(Error::KeyResolutionError {
            key: $key.clone().into(),
            reason: $reason.to_string(),
        });
    };
}

pub struct SocketResolver {
    queue: VecDeque<Job>,
}

impl SocketResolver {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

    pub fn traverse(&mut self, context: &mut PlanResource) -> Result<(), Error> {
        self.queue.push_back(
            VisitNodeJob {
                current: context.root.clone(),
                current_prefix: KeyOwned::new_root().clone(),
            }
            .into(),
        );

        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::VisitNode(job) => {
                    self.visit_node(job)?;
                }
                Job::ResolveSocket(job) => {
                    self.resolve_sockets_in_plan(job)?;
                }
            }
        }

        Ok(())
    }

    pub fn visit_node(&mut self, job: VisitNodeJob) -> Result<(), Error> {
        let VisitNodeJob {
            current,
            current_prefix,
        } = job;

        let guard = current.read();
        {
            // Visit children
            for (suffix, child) in &guard.children {
                let Ok(child_prefix) = &current_prefix / suffix else {
                    unreachable!()
                };

                self.queue.push_back(
                    VisitNodeJob {
                        current: child.clone(),
                        current_prefix: child_prefix,
                    }
                    .into(),
                );
            }
        }

        if let Some(TrieContext::Plan(_)) = &guard.context {
            self.queue.push_back(
                ResolveSocketJob {
                    current: current.clone(),
                    // current_prefix,
                }
                .into(),
            );
        }

        Ok(())
    }

    fn resolve_sockets_in_plan(&self, job: ResolveSocketJob) -> Result<(), Error> {
        let ResolveSocketJob {
            current,
            // current_prefix: _,
        } = job;

        // Take out the socket_map from the plan context
        let mut socket_map = {
            let mut guard = current.write();
            let Some(TrieContext::Plan(plan_ctx)) = &mut guard.context else {
                unreachable!("a plan context is expected");
            };
            let plan_ctx: &mut PlanContext = plan_ctx;
            std::mem::take(&mut plan_ctx.socket_map)
        };

        // Resolve URIs in the socket_map
        socket_map
            .values_mut()
            .try_for_each(|socket_ctx| -> Result<_, Error> {
                let mut guard = socket_ctx.write();
                resolve_socket_topics(current.clone(), &mut guard)
            })?;

        // Update plan context
        {
            let mut guard = current.write();
            let Some(TrieContext::Plan(plan_ctx)) = guard.context.take() else {
                unreachable!("a plan context is expected");
            };

            let PlanContext {
                path,
                arg,
                node_map,
                link_map,
                ..
            } = plan_ctx;
            guard.context = Some(
                PlanContext {
                    path,
                    arg,
                    socket_map,
                    node_map,
                    link_map,
                }
                .into(),
            );
        }

        Ok(())
    }
}

fn resolve_socket_topics(current: TrieRef, socket_ctx: &mut SocketContext) -> Result<(), Error> {
    match socket_ctx {
        SocketContext::Pub(pub_ctx) => resolve_pub_socket_topics(current, pub_ctx)?,
        SocketContext::Sub(sub_ctx) => resolve_sub_socket_topics(current, sub_ctx)?,
        SocketContext::Srv(srv_ctx) => resolve_srv_socket_topics(current, srv_ctx)?,
        SocketContext::Qry(qry_ctx) => resolve_qry_socket_topics(current, qry_ctx)?,
    }
    Ok(())
}

fn resolve_pub_socket_topics(current: TrieRef, pub_: &mut PubSocketContext) -> Result<(), Error> {
    let src: Vec<_> = pub_
        .config
        .src
        .iter()
        .map(|uri| {
            let TopicUri {
                node: node_key,
                topic,
            } = uri;

            let Some(resolve) = resolve_node_key(current.clone(), node_key) else {
                bail_resolve_key_error!(
                    node_key,
                    "the key does not resolve to a node or a plan socket"
                );
            };

            let topic_uris = match resolve {
                ResolveNode::Node(child_node_arc) => vec![NodeTopicUri {
                    node: child_node_arc.downgrade(),
                    topic: topic.clone(),
                }],
                ResolveNode::Socket(child_socket_arc) => {
                    let guard = child_socket_arc.read();
                    let SocketContext::Pub(child_pub_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "socket type mismatch");
                    };
                    child_pub_ctx.src.clone().unwrap()
                }
            };

            Ok(topic_uris)
        })
        .flatten_ok()
        .try_collect()?;
    pub_.src = Some(src);

    Ok(())
}

fn resolve_sub_socket_topics(current: TrieRef, sub: &mut SubSocketContext) -> Result<(), Error> {
    let dst: Vec<_> = sub
        .config
        .dst
        .iter()
        .map(|uri| {
            let TopicUri {
                node: node_key,
                topic,
            } = uri;

            let Some(resolve) = resolve_node_key(current.clone(), &node_key) else {
                bail_resolve_key_error!(
                    node_key,
                    "the key does not resolve to a node or a plan socket"
                );
            };

            let topic_uris = match resolve {
                ResolveNode::Node(child_node_arc) => vec![NodeTopicUri {
                    node: child_node_arc.downgrade(),
                    topic: topic.clone(),
                }],
                ResolveNode::Socket(child_socket_arc) => {
                    let guard = child_socket_arc.read();
                    let SocketContext::Sub(child_sub_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "socket type mismatch");
                    };
                    child_sub_ctx.dst.clone().unwrap()
                }
            };

            Ok(topic_uris)
        })
        .flatten_ok()
        .try_collect()?;

    sub.dst = Some(dst);
    Ok(())
}

fn resolve_srv_socket_topics(current: TrieRef, srv: &mut ServerSocketContext) -> Result<(), Error> {
    let TopicUri {
        node: node_key,
        topic,
    } = &srv.config.listen;

    let Some(resolve) = resolve_node_key(current.clone(), node_key) else {
        bail_resolve_key_error!(
            node_key,
            "the key does not resolve to a node or a plan socket"
        );
    };

    let listen = match resolve {
        ResolveNode::Node(child_node_arc) => NodeTopicUri {
            node: child_node_arc.downgrade(),
            topic: topic.clone(),
        },
        ResolveNode::Socket(child_socket_arc) => {
            let guard = child_socket_arc.read();
            let SocketContext::Srv(child_srv_ctx) = &*guard else {
                bail_resolve_key_error!(node_key, "socket type mismatch");
            };
            child_srv_ctx.listen.clone().unwrap()
        }
    };

    srv.listen = Some(listen);
    Ok(())
}

fn resolve_qry_socket_topics(current: TrieRef, qry: &mut QuerySocketContext) -> Result<(), Error> {
    let connect: Vec<_> = qry
        .config
        .connect
        .iter()
        .map(|uri| {
            let TopicUri {
                node: node_key,
                topic,
            } = uri;

            let Some(resolve) = resolve_node_key(current.clone(), node_key) else {
                bail_resolve_key_error!(
                    node_key,
                    "the key does not resolve to a node or a plan socket"
                );
            };

            let topic_uris = match resolve {
                ResolveNode::Node(child_node_arc) => vec![NodeTopicUri {
                    node: child_node_arc.downgrade(),
                    topic: topic.clone(),
                }],
                ResolveNode::Socket(child_socket_arc) => {
                    let guard = child_socket_arc.read();
                    let SocketContext::Qry(child_qry_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "socket type mismatch");
                    };
                    child_qry_ctx.connect.clone().unwrap()
                }
            };

            Ok(topic_uris)
        })
        .flatten_ok()
        .try_collect()?;

    qry.connect = Some(connect);
    Ok(())
}

fn resolve_node_key(root: TrieRef, key: &Key) -> Option<ResolveNode> {
    {
        let guard = root.read();
        if !matches!(guard.context, Some(TrieContext::Plan(_))) {
            panic!("the search must starts at a plan node");
        }
    }
    assert!(!key.is_absolute(), "the key must be relative");

    let (prefix, node_name) = key.split_parent();
    let node_name = node_name.expect("the key should not be empty");

    let resolve: ResolveNode = match prefix {
        Some(prefix) => {
            let child = root.get_child(prefix)?;
            let guard = child.read();
            match guard.context.as_ref()? {
                TrieContext::Plan(ctx) => {
                    let socket_arc = ctx.socket_map.get(node_name)?;
                    socket_arc.clone().into()
                }
                TrieContext::HerePlan(ctx) => {
                    let node_arc = ctx.node_map.get(node_name)?;
                    node_arc.clone().into()
                }
            }
        }
        None => {
            let guard = root.read();
            let Some(TrieContext::Plan(ctx)) = &guard.context else {
                unreachable!();
            };
            let node_arc = ctx.node_map.get(node_name)?;
            node_arc.clone().into()
        }
    };

    Some(resolve)
}

pub enum Job {
    VisitNode(VisitNodeJob),
    ResolveSocket(ResolveSocketJob),
}

impl From<VisitNodeJob> for Job {
    fn from(v: VisitNodeJob) -> Self {
        Self::VisitNode(v)
    }
}

impl From<ResolveSocketJob> for Job {
    fn from(v: ResolveSocketJob) -> Self {
        Self::ResolveSocket(v)
    }
}

pub struct VisitNodeJob {
    current: TrieRef,
    current_prefix: KeyOwned,
}

pub struct ResolveSocketJob {
    current: TrieRef,
    // current_prefix: KeyOwned,
}

pub enum ResolveNode {
    Node(NodeArc),
    Socket(SocketArc),
}

impl From<SocketArc> for ResolveNode {
    fn from(v: SocketArc) -> Self {
        Self::Socket(v)
    }
}

impl From<NodeArc> for ResolveNode {
    fn from(v: NodeArc) -> Self {
        Self::Node(v)
    }
}
