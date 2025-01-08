use crate::{
    context::{
        GlobalContextV1, GlobalContextV2, HerePlanContextV1, HerePlanContextV2, LinkArc,
        LinkContext, NodeArc, NodeTopicUri, PlanContextV2, PlanContextV3, PubSubLinkContext,
        ServiceLinkContext, SocketArc, SocketContext, TrieContext, TrieRef,
    },
    error::Error,
};
use itertools::Itertools;
use ros_plan_format::{
    key::{Key, KeyOwned},
    link::{Link, PubSubLink, ServiceLink, TopicUri},
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

pub struct LinkResolver {
    queue: VecDeque<Job>,
}

impl LinkResolver {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

    pub fn traverse(&mut self, context: GlobalContextV1) -> Result<GlobalContextV2, Error> {
        let mut context = {
            let GlobalContextV1 {
                namespace,
                root,
                node_map,
            } = context;
            GlobalContextV2 {
                namespace,
                root,
                node_map,
                link_map: HashMap::new(),
            }
        };

        // Schedule the job to visit the root
        self.queue.push_back(
            VisitNodeJob {
                current: context.root.clone(),
                current_prefix: context.namespace.clone(),
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
                    self.resolve_link(&mut context, job)?;
                }
            }
        }

        Ok(context)
    }

    pub fn visit_node(&mut self, job: VisitNodeJob) -> Result<(), Error> {
        let VisitNodeJob {
            current,
            current_prefix,
        } = job;

        let guard = current.read();

        // Schedule jobs to visit child nodes.
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

        // Schedule a job to resolve links if the node has a plan or a
        // hereplan context.
        if let Some(ctx) = &guard.context {
            match ctx {
                TrieContext::PlanV2(_) => {
                    self.queue.push_back(
                        ResolveLinkJob {
                            current: current.clone(),
                            current_prefix,
                        }
                        .into(),
                    );
                }
                TrieContext::HerePlanV1(_) => {
                    self.queue.push_back(
                        ResolveLinkJob {
                            current: current.clone(),
                            current_prefix,
                        }
                        .into(),
                    );
                }
                _ => unreachable!(),
            };
        }

        Ok(())
    }

    pub fn resolve_link(
        &mut self,
        context: &mut GlobalContextV2,
        job: ResolveLinkJob,
    ) -> Result<(), Error> {
        let ResolveLinkJob {
            current,
            current_prefix,
        } = job;

        // Take the link_map out of the node context.
        let link_map = {
            let mut guard = current.write();
            let Some(node_ctx) = &mut guard.context else {
                unreachable!();
            };

            match node_ctx {
                TrieContext::PlanV2(plan_ctx) => std::mem::take(&mut plan_ctx.link_map),
                TrieContext::HerePlanV1(hereplan_ctx) => std::mem::take(&mut hereplan_ctx.link_map),
                _ => unreachable!(),
            }
        };

        // Resolve links
        let link_map: HashMap<_, _> = link_map
            .into_iter()
            .map(|(link_ident, link_cfg)| -> Result<_, Error> {
                let link_ctx = resolve_link(context, current.clone(), link_cfg)?;
                let link_arc: LinkArc = link_ctx.into();
                Ok((link_ident, link_arc))
            })
            .try_collect()?;

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
            let Some(node_ctx) = guard.context.take() else {
                unreachable!();
            };

            let node_ctx: TrieContext = match node_ctx {
                TrieContext::PlanV2(plan_ctx) => {
                    let PlanContextV2 {
                        path,
                        arg,
                        socket_map,
                        node_map,
                        ..
                    } = plan_ctx;

                    PlanContextV3 {
                        path,
                        arg,
                        socket_map,
                        node_map,
                        link_map,
                    }
                    .into()
                }
                TrieContext::HerePlanV1(hereplan_ctx) => {
                    let HerePlanContextV1 { node_map, .. } = hereplan_ctx;

                    HerePlanContextV2 { node_map, link_map }.into()
                }
                _ => unreachable!(),
            };
            guard.context = Some(node_ctx);
        };

        Ok(())
    }
}

fn resolve_link(
    context: &GlobalContextV2,
    current: TrieRef,
    link: Link,
) -> Result<LinkContext, Error> {
    let link_ctx: LinkContext = match link {
        Link::Pubsub(link) => resolve_pubsub_link(context, current, link)?.into(),
        Link::Service(link) => resolve_service_link(context, current, link)?.into(),
    };
    Ok(link_ctx)
}

fn resolve_pubsub_link(
    context: &GlobalContextV2,
    current: TrieRef,
    link: PubSubLink,
) -> Result<PubSubLinkContext, Error> {
    let PubSubLink { ty, qos, src, dst } = link;

    let src: Vec<_> = src
        .into_iter()
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
                    topic,
                }],
                ResolveNode::Socket(socket_arc) => {
                    let guard = socket_arc.read();
                    let SocketContext::Pub(pub_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "expect a pub socket");
                    };
                    pub_ctx.src.clone()
                }
            };

            Ok(uris)
        })
        .flatten_ok()
        .try_collect()?;

    let dst: Vec<_> = dst
        .into_iter()
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
                    topic,
                }],
                ResolveNode::Socket(socket_arc) => {
                    let guard = socket_arc.read();
                    let SocketContext::Sub(sub_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "expect a sub socket");
                    };
                    sub_ctx.dst.clone()
                }
            };

            Ok(uris)
        })
        .flatten_ok()
        .try_collect()?;

    Ok(PubSubLinkContext { src, dst, ty, qos })
}

fn resolve_service_link(
    context: &GlobalContextV2,
    current: TrieRef,
    link: ServiceLink,
) -> Result<ServiceLinkContext, Error> {
    let ServiceLink {
        ty,
        listen,
        connect,
    } = link;

    let listen = {
        let TopicUri {
            node: node_key,
            topic,
        } = listen;
        let Some(resolve) = resolve_node_key(context, current.clone(), &node_key) else {
            bail_resolve_key_error!(node_key, "unable to resolve key");
        };

        let uri = match resolve {
            ResolveNode::Node(node_arc) => NodeTopicUri {
                node: node_arc.downgrade(),
                topic,
            },
            ResolveNode::Socket(socket_arc) => {
                let guard = socket_arc.read();
                let SocketContext::Srv(srv_ctx) = &*guard else {
                    bail_resolve_key_error!(node_key, "expect a qry socket");
                };
                srv_ctx.listen.clone()
            }
        };

        uri
    };

    let connect: Vec<_> = connect
        .into_iter()
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
                    topic,
                }],
                ResolveNode::Socket(socket_arc) => {
                    let guard = socket_arc.read();
                    let SocketContext::Qry(qry_ctx) = &*guard else {
                        bail_resolve_key_error!(node_key, "expect a qry socket");
                    };
                    qry_ctx.connect.clone()
                }
            };

            Ok(uris)
        })
        .flatten_ok()
        .try_collect()?;

    Ok(ServiceLinkContext {
        ty,
        listen,
        connect,
    })
}

fn resolve_node_key(
    context: &GlobalContextV2,
    plan_or_hereplan_root: TrieRef,
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
                let mut comp_iter = prefix.components();
                let mut curr = plan_or_hereplan_root;

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
                let guard = plan_or_hereplan_root.read();
                let Some(node_ctx) = &guard.context else {
                    unreachable!();
                };
                let node_map = match node_ctx {
                    TrieContext::PlanV2(plan_ctx) => &plan_ctx.node_map,
                    TrieContext::HerePlanV1(hereplan_ctx) => &hereplan_ctx.node_map,
                    _ => unreachable!(),
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
    current: TrieRef,
    current_prefix: KeyOwned,
}

pub struct ResolveLinkJob {
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
