use crate::{
    context::{
        link::{LinkContext, PubSubLinkContext, ServiceLinkContext},
        node::NodeArc,
        socket::{SocketArc, SocketContext},
        uri::NodeTopicUri,
    },
    error::Error,
    resource::{PlanResource, Resource, ResourceTreeRef},
};
use itertools::Itertools;
use ros_plan_format::{
    key::{Key, KeyOwned},
    link::TopicUri,
};
use std::{collections::VecDeque, mem};

macro_rules! bail_resolve_key_error {
    ($key:expr, $reason:expr) => {
        return Err(Error::KeyResolutionError {
            key: $key.clone().into(),
            reason: $reason.to_string(),
        });
    };
}

pub struct LinkResolver {
    queue: VecDeque<Job>,
}

impl LinkResolver {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

    pub fn traverse(&mut self, context: &mut Resource) -> Result<(), Error> {
        // Schedule the job to visit the root
        self.queue.push_back(
            VisitNodeJob {
                current: context.root.clone(),
                current_prefix: KeyOwned::new_root(),
            }
            .into(),
        );

        // Perform traversal
        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::VisitNode(job) => {
                    self.visit_node(job)?;
                }
                Job::ResolveLink(job) => {
                    self.resolve_link(context, job)?;
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

        // Schedule jobs to visit child nodes.
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

        // Schedule a job to resolve links if the node has a plan or a
        // hereplan context.
        if let Some(ctx) = &guard.context {
            match ctx {
                PlanResource::PlanFile { .. } => {
                    self.queue.push_back(
                        ResolveLinkJob {
                            current: current.clone(),
                            current_prefix,
                        }
                        .into(),
                    );
                }
                PlanResource::HerePlan(_) => {
                    self.queue.push_back(
                        ResolveLinkJob {
                            current: current.clone(),
                            current_prefix,
                        }
                        .into(),
                    );
                }
            };
        }

        Ok(())
    }

    pub fn resolve_link(
        &mut self,
        context: &mut Resource,
        job: ResolveLinkJob,
    ) -> Result<(), Error> {
        let ResolveLinkJob {
            current,
            current_prefix,
        } = job;

        // Take the link_map out of the node context.
        let mut link_map = {
            let mut guard = current.write();
            let Some(node_ctx) = &mut guard.context else {
                unreachable!();
            };

            match node_ctx {
                PlanResource::PlanFile(ctx) => mem::take(&mut ctx.context.link_map),
                PlanResource::HerePlan(ctx) => mem::take(&mut ctx.link_map),
            }
        };

        // Resolve links
        link_map
            .values_mut()
            .try_for_each(|link_ctx| -> Result<_, Error> {
                let mut guard = link_ctx.write();
                resolve_link(context, current.clone(), &mut guard)
            })?;

        // Insert links to the global table
        {
            let link_entries = link_map.iter().map(|(link_ident, link_arc)| {
                let link_key = &current_prefix / link_ident;
                let link_weak = link_arc.downgrade();
                (link_key, link_weak)
            });
            context.link_map.extend(link_entries);
        }

        // Push the resolved links back to the node context
        {
            let mut guard = current.write();
            let Some(trie_ctx) = guard.context.as_mut() else {
                unreachable!();
            };

            match trie_ctx {
                PlanResource::PlanFile(ctx) => {
                    ctx.context.link_map = link_map;
                }
                PlanResource::HerePlan(ctx) => {
                    ctx.link_map = link_map;
                }
            }
        }

        Ok(())
    }
}

fn resolve_link(
    context: &Resource,
    current: ResourceTreeRef,
    link: &mut LinkContext,
) -> Result<(), Error> {
    match link {
        LinkContext::Pubsub(link) => resolve_pubsub_link(context, current, link)?,
        LinkContext::Service(link) => resolve_service_link(context, current, link)?,
    }
    Ok(())
}

fn resolve_pubsub_link(
    context: &Resource,
    current: ResourceTreeRef,
    link: &mut PubSubLinkContext,
) -> Result<(), Error> {
    // let PubSubLink { ty, qos, src, dst } = link;

    let src: Vec<_> = link
        .config
        .src
        .iter()
        .map(|uri| {
            let TopicUri {
                node: node_key,
                topic,
            } = uri;
            let Some(resolve) = resolve_node_key(context, current.clone(), &node_key) else {
                bail_resolve_key_error!(node_key, "unable to resolve key");
            };

            let uris: Vec<_> = match resolve {
                ResolveNode::Node(node_arc) => vec![NodeTopicUri {
                    node: node_arc.downgrade(),
                    topic: topic.clone(),
                }],
                ResolveNode::Socket(socket_arc) => {
                    let guard = socket_arc.read();
                    let SocketContext::Pub(pub_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "expect a pub socket");
                    };
                    pub_ctx.src.clone().unwrap()
                }
            };

            Ok(uris)
        })
        .flatten_ok()
        .try_collect()?;

    let dst: Vec<_> = link
        .config
        .dst
        .iter()
        .map(|uri| {
            let TopicUri {
                node: node_key,
                topic,
            } = uri;
            let Some(resolve) = resolve_node_key(context, current.clone(), &node_key) else {
                bail_resolve_key_error!(node_key, "unable to resolve key");
            };

            let uris: Vec<_> = match resolve {
                ResolveNode::Node(node_arc) => vec![NodeTopicUri {
                    node: node_arc.downgrade(),
                    topic: topic.clone(),
                }],
                ResolveNode::Socket(socket_arc) => {
                    let guard = socket_arc.read();
                    let SocketContext::Sub(sub_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "expect a sub socket");
                    };
                    sub_ctx.dst.clone().unwrap()
                }
            };

            Ok(uris)
        })
        .flatten_ok()
        .try_collect()?;

    link.src = Some(src);
    link.dst = Some(dst);

    Ok(())
}

fn resolve_service_link(
    context: &Resource,
    current: ResourceTreeRef,
    link: &mut ServiceLinkContext,
) -> Result<(), Error> {
    let listen = {
        let TopicUri {
            node: node_key,
            topic,
        } = &link.config.listen;
        let Some(resolve) = resolve_node_key(context, current.clone(), &node_key) else {
            bail_resolve_key_error!(node_key, "unable to resolve key");
        };

        let uri = match resolve {
            ResolveNode::Node(node_arc) => NodeTopicUri {
                node: node_arc.downgrade(),
                topic: topic.clone(),
            },
            ResolveNode::Socket(socket_arc) => {
                let guard = socket_arc.read();
                let SocketContext::Srv(srv_ctx) = &*guard else {
                    bail_resolve_key_error!(node_key, "expect a qry socket");
                };
                srv_ctx.listen.clone().unwrap()
            }
        };

        uri
    };

    let connect: Vec<_> = link
        .config
        .connect
        .iter()
        .map(|uri| {
            let TopicUri {
                node: node_key,
                topic,
            } = uri;
            let Some(resolve) = resolve_node_key(context, current.clone(), &node_key) else {
                bail_resolve_key_error!(node_key, "unable to resolve key");
            };

            let uris: Vec<_> = match resolve {
                ResolveNode::Node(node_arc) => vec![NodeTopicUri {
                    node: node_arc.downgrade(),
                    topic: topic.clone(),
                }],
                ResolveNode::Socket(socket_arc) => {
                    let guard = socket_arc.read();
                    let SocketContext::Qry(qry_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "expect a qry socket");
                    };
                    qry_ctx.connect.clone().unwrap()
                }
            };

            Ok(uris)
        })
        .flatten_ok()
        .try_collect()?;

    link.listen = Some(listen);
    link.connect = Some(connect);

    Ok(())
}

fn resolve_node_key(
    context: &Resource,
    plan_or_hereplan_root: ResourceTreeRef,
    key: &Key,
) -> Option<ResolveNode> {
    if key.is_absolute() {
        let node_weak = context.node_map.get(key)?;
        let node_arc = node_weak.upgrade().unwrap();
        Some(node_arc.into())
    } else {
        let (prefix, node_name) = key.split_parent();
        let node_name = node_name.expect("the key should not be empty");

        let resolve: ResolveNode = match prefix {
            Some(prefix) => {
                let child = plan_or_hereplan_root.get_child(prefix)?;
                let guard = child.read();
                match guard.context.as_ref()? {
                    PlanResource::PlanFile(ctx) => {
                        let socket_arc = ctx.context.socket_map.get(node_name)?;
                        socket_arc.clone().into()
                    }
                    PlanResource::HerePlan(ctx) => {
                        let node_arc = ctx.node_map.get(node_name)?;
                        node_arc.clone().into()
                    }
                }
            }
            None => {
                let guard = plan_or_hereplan_root.read();
                let Some(node_ctx) = &guard.context else {
                    unreachable!();
                };
                let node_map = match node_ctx {
                    PlanResource::PlanFile(ctx) => &ctx.context.node_map,
                    PlanResource::HerePlan(ctx) => &ctx.node_map,
                };
                let node_arc = node_map.get(node_name)?;
                node_arc.clone().into()
            }
        };

        Some(resolve)
    }
}

pub enum Job {
    VisitNode(VisitNodeJob),
    ResolveLink(ResolveLinkJob),
}

impl From<ResolveLinkJob> for Job {
    fn from(v: ResolveLinkJob) -> Self {
        Self::ResolveLink(v)
    }
}

impl From<VisitNodeJob> for Job {
    fn from(v: VisitNodeJob) -> Self {
        Self::VisitNode(v)
    }
}

pub struct VisitNodeJob {
    current: ResourceTreeRef,
    current_prefix: KeyOwned,
}

pub struct ResolveLinkJob {
    current: ResourceTreeRef,
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
