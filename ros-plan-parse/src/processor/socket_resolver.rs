use crate::{
    context::{
        expr::ExprContext,
        node::NodeArc,
        socket::{
            PubSocketContext, QuerySocketContext, ServerSocketContext, SocketArc, SocketContext,
            SubSocketContext,
        },
        uri::NodeTopicUri,
    },
    error::Error,
    resource::{Resource, Scope, ScopeTreeRef},
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

#[derive(Debug, Default)]
pub struct SocketResolver {
    queue: VecDeque<Job>,
}

impl SocketResolver {
    pub fn traverse(&mut self, context: &mut Resource) -> Result<(), Error> {
        self.queue.push_back(
            VisitNodeJob {
                current: context.root.clone().unwrap(),
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

        if let Scope::PlanFile { .. } = &guard.value {
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
            let Scope::PlanFile(plan_ctx) = &mut guard.value else {
                unreachable!("a plan context is expected");
            };
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
            let Scope::PlanFile(plan_ctx) = &mut guard.value else {
                unreachable!("a plan context is expected");
            };
            plan_ctx.socket_map = socket_map;
        }

        Ok(())
    }
}

fn resolve_socket_topics(
    current: ScopeTreeRef,
    socket_ctx: &mut SocketContext,
) -> Result<(), Error> {
    match socket_ctx {
        SocketContext::Pub(pub_ctx) => resolve_pub_socket_topics(current, pub_ctx)?,
        SocketContext::Sub(sub_ctx) => resolve_sub_socket_topics(current, sub_ctx)?,
        SocketContext::Srv(srv_ctx) => resolve_srv_socket_topics(current, srv_ctx)?,
        SocketContext::Qry(qry_ctx) => resolve_qry_socket_topics(current, qry_ctx)?,
    }
    Ok(())
}

fn resolve_pub_socket_topics(
    current: ScopeTreeRef,
    pub_: &mut PubSocketContext,
) -> Result<(), Error> {
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
                    topic: ExprContext::new(topic.clone()),
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

fn resolve_sub_socket_topics(
    current: ScopeTreeRef,
    sub: &mut SubSocketContext,
) -> Result<(), Error> {
    let dst: Vec<_> = sub
        .config
        .dst
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
                    topic: ExprContext::new(topic.clone()),
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

fn resolve_srv_socket_topics(
    current: ScopeTreeRef,
    srv: &mut ServerSocketContext,
) -> Result<(), Error> {
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
            topic: ExprContext::new(topic.clone()),
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

fn resolve_qry_socket_topics(
    current: ScopeTreeRef,
    qry: &mut QuerySocketContext,
) -> Result<(), Error> {
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
                    topic: ExprContext::new(topic.clone()),
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

fn resolve_node_key(root: ScopeTreeRef, key: &Key) -> Option<ResolveNode> {
    {
        let guard = root.read();
        if !matches!(guard.value, Scope::PlanFile { .. }) {
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
            match &guard.value {
                Scope::PlanFile(ctx) => {
                    let socket_arc = ctx.socket_map.get(node_name)?;
                    socket_arc.clone().into()
                }
                Scope::Group(ctx) => {
                    let node_arc = ctx.node_map.get(node_name)?;
                    node_arc.clone().into()
                }
            }
        }
        None => {
            let guard = root.read();
            let Scope::PlanFile(ctx) = &guard.value else {
                unreachable!();
            };
            let node_arc = ctx.node_map.get(node_name)?;
            node_arc.clone().into()
        }
    };

    Some(resolve)
}

#[derive(Debug)]
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

#[derive(Debug)]
pub struct VisitNodeJob {
    current: ScopeTreeRef,
    current_prefix: KeyOwned,
}

#[derive(Debug)]
pub struct ResolveSocketJob {
    current: ScopeTreeRef,
    // current_prefix: KeyOwned,
}

#[derive(Debug)]
enum ResolveNode {
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
