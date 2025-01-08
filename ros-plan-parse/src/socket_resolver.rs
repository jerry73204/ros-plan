use crate::{
    context::{
        GlobalContextV1, NodeArc, NodeTopicUri, PlanContextV1, PlanContextV2, PubSocketContext,
        QuerySocketContext, ServerSocketContext, SocketArc, SocketContext, SubSocketContext,
        TrieContext, TrieRef,
    },
    error::Error,
};
use itertools::Itertools;
use ros_plan_format::{
    key::{Key, KeyOwned},
    link::TopicUri,
    socket::{PubSocket, QuerySocket, ServerSocket, Socket, SubSocket},
};
use std::collections::{HashMap, VecDeque};

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

    pub fn traverse(&mut self, context: &mut GlobalContextV1) -> Result<(), Error> {
        self.queue.push_back(
            VisitNodeJob {
                current: context.root.clone(),
                current_prefix: context.namespace.clone(),
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
                let child_prefix = &current_prefix / suffix;

                self.queue.push_back(
                    VisitNodeJob {
                        current: child.clone(),
                        current_prefix: child_prefix,
                    }
                    .into(),
                );
            }
        }

        if let Some(TrieContext::PlanV1(_)) = &guard.context {
            self.queue.push_back(
                ResolveSocketJob {
                    current: current.clone(),
                    current_prefix,
                }
                .into(),
            );
        }

        Ok(())
    }

    fn resolve_sockets_in_plan(&self, job: ResolveSocketJob) -> Result<(), Error> {
        let ResolveSocketJob {
            current,
            current_prefix: _,
        } = job;

        // Take out the socket_map from the plan context
        let socket_map = {
            let mut guard = current.write();
            let Some(TrieContext::PlanV1(plan_ctx)) = &mut guard.context else {
                unreachable!("a plan context is expected");
            };
            let plan_ctx: &mut PlanContextV1 = plan_ctx;
            std::mem::take(&mut plan_ctx.socket_map)
        };

        // Resolve URIs in the socket_map
        let socket_map: HashMap<_, _> = socket_map
            .into_iter()
            .map(|(socket_ident, socket_cfg)| -> Result<_, Error> {
                let socket_ctx = resolve_socket_topics(current.clone(), socket_cfg)?;
                let socket_arc: SocketArc = socket_ctx.into();
                Ok((socket_ident, socket_arc))
            })
            .try_collect()?;

        // Update plan context
        {
            let mut guard = current.write();
            let Some(TrieContext::PlanV1(plan_ctx)) = guard.context.take() else {
                unreachable!("a plan context is expected");
            };

            let PlanContextV1 {
                path,
                arg,
                node_map,
                link_map,
                ..
            } = plan_ctx;
            guard.context = Some(
                PlanContextV2 {
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

fn resolve_socket_topics(current: TrieRef, socket_ctx: Socket) -> Result<SocketContext, Error> {
    let socket_ctx: SocketContext = match socket_ctx {
        Socket::Pub(pub_ctx) => resolve_pub_socket_topics(current, pub_ctx)?.into(),
        Socket::Sub(sub_ctx) => resolve_sub_socket_topics(current, sub_ctx)?.into(),
        Socket::Srv(srv_ctx) => resolve_srv_socket_topics(current, srv_ctx)?.into(),
        Socket::Qry(qry_ctx) => resolve_qry_socket_topics(current, qry_ctx)?.into(),
    };
    Ok(socket_ctx)
}

fn resolve_pub_socket_topics(current: TrieRef, pub_: PubSocket) -> Result<PubSocketContext, Error> {
    let src: Vec<_> = pub_
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
                    child_pub_ctx.src.clone()
                }
            };

            Ok(topic_uris)
        })
        .flatten_ok()
        .try_collect()?;

    Ok(PubSocketContext { config: pub_, src })
}

fn resolve_sub_socket_topics(current: TrieRef, sub: SubSocket) -> Result<SubSocketContext, Error> {
    let dst: Vec<_> = sub
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
                    topic: topic.clone(),
                }],
                ResolveNode::Socket(child_socket_arc) => {
                    let guard = child_socket_arc.read();
                    let SocketContext::Sub(child_sub_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "socket type mismatch");
                    };
                    child_sub_ctx.dst.clone()
                }
            };

            Ok(topic_uris)
        })
        .flatten_ok()
        .try_collect()?;

    Ok(SubSocketContext { config: sub, dst })
}

fn resolve_srv_socket_topics(
    current: TrieRef,
    srv: ServerSocket,
) -> Result<ServerSocketContext, Error> {
    let TopicUri {
        node: node_key,
        topic,
    } = &srv.listen;

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
            child_srv_ctx.listen.clone()
        }
    };

    Ok(ServerSocketContext {
        config: srv,
        listen,
    })
}

fn resolve_qry_socket_topics(
    current: TrieRef,
    qry: QuerySocket,
) -> Result<QuerySocketContext, Error> {
    let connect: Vec<_> = qry
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
                    child_qry_ctx.connect.clone()
                }
            };

            Ok(topic_uris)
        })
        .flatten_ok()
        .try_collect()?;

    Ok(QuerySocketContext {
        config: qry,
        connect,
    })
}

fn resolve_node_key(root: TrieRef, key: &Key) -> Option<ResolveNode> {
    {
        let guard = root.read();
        if !matches!(guard.context, Some(TrieContext::PlanV1(_))) {
            panic!("the search must starts at a plan node");
        }
    }
    assert!(!key.is_absolute(), "the key must be relative");

    let (prefix, node_name) = key.split_parent();
    let node_name = node_name.expect("the key should not be empty");

    let resolve: ResolveNode = match prefix {
        Some(prefix) => {
            let mut comp_iter = prefix.components();
            let mut curr = root;

            // The first component can move away from a plan node.
            if let Some(comp) = comp_iter.next() {
                curr = curr.get_child(comp)?;
            }

            // Later components cannot move away from a plan node.
            for comp in comp_iter {
                let next = {
                    let guard = curr.read();
                    if matches!(guard.context, Some(TrieContext::PlanV1(_))) {
                        return None;
                    }
                    let next = guard.children.get(comp)?;
                    next.clone()
                };
                curr = next.clone();
            }

            {
                let guard = curr.read();
                match guard.context.as_ref()? {
                    TrieContext::PlanV2(ctx) => {
                        let socket_arc = ctx.socket_map.get(node_name)?;
                        socket_arc.clone().into()
                    }
                    TrieContext::HerePlanV1(ctx) => {
                        let node_arc = ctx.node_map.get(node_name)?;
                        node_arc.clone().into()
                    }
                    _ => unreachable!(),
                }
            }
        }
        None => {
            let guard = root.read();
            let Some(TrieContext::PlanV1(ctx)) = &guard.context else {
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
    current_prefix: KeyOwned,
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
