use crate::{
    context::{
        expr::ExprContext,
        link::{LinkContext, PubsubLinkContext, ServiceLinkContext},
        socket::SocketContext,
        uri::NodeTopicUri,
    },
    error::Error,
    resource::Resource,
    scope::{Scope, ScopeTreeRef},
    utils::{resolve_node_entity, ResolveNode},
};
use itertools::Itertools;
use ros_plan_format::{key::KeyOwned, link::TopicUri};
use std::{collections::VecDeque, mem};

macro_rules! bail_resolve_key_error {
    ($key:expr, $reason:expr) => {
        return Err(Error::KeyResolutionError {
            key: $key.clone().into(),
            reason: $reason.to_string(),
        });
    };
}

#[derive(Debug, Default)]
pub struct LinkResolver {
    queue: VecDeque<Job>,
}

impl LinkResolver {
    pub fn traverse(&mut self, context: &mut Resource) -> Result<(), Error> {
        // Schedule the job to visit the root
        self.queue.push_back(
            VisitNodeJob {
                current: context.root.clone().unwrap(),
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

    fn visit_node(&mut self, job: VisitNodeJob) -> Result<(), Error> {
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
        match guard.value {
            Scope::PlanFile { .. } => {
                self.queue.push_back(
                    ResolveLinkJob {
                        current: current.clone(),
                        current_prefix,
                    }
                    .into(),
                );
            }
            Scope::Group(_) => {
                self.queue.push_back(
                    ResolveLinkJob {
                        current: current.clone(),
                        current_prefix,
                    }
                    .into(),
                );
            }
        };

        Ok(())
    }

    fn resolve_link(&mut self, context: &mut Resource, job: ResolveLinkJob) -> Result<(), Error> {
        let ResolveLinkJob {
            current,
            current_prefix: _,
        } = job;

        // Take the link_map out of the node context.
        let mut link_map = {
            let mut guard = current.write();
            match &mut guard.value {
                Scope::PlanFile(ctx) => mem::take(&mut ctx.link_map),
                Scope::Group(ctx) => mem::take(&mut ctx.link_map),
            }
        };

        // Resolve links
        link_map
            .values_mut()
            .try_for_each(|shared| -> Result<_, Error> {
                let owned = shared.upgrade().unwrap();
                let mut guard = owned.write();
                resolve_link(context, current.clone(), &mut guard)
            })?;

        // Push the resolved links back to the node context
        {
            let mut guard = current.write();
            let trie_ctx = &mut guard.value;

            match trie_ctx {
                Scope::PlanFile(ctx) => {
                    ctx.link_map = link_map;
                }
                Scope::Group(ctx) => {
                    ctx.link_map = link_map;
                }
            }
        }

        Ok(())
    }
}

fn resolve_link(
    context: &Resource,
    current: ScopeTreeRef,
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
    current: ScopeTreeRef,
    link: &mut PubsubLinkContext,
) -> Result<(), Error> {
    let src: Vec<_> = link
        .config
        .src
        .iter()
        .map(|uri| {
            let TopicUri {
                node: node_key,
                topic,
            } = uri;
            let Some(resolve) = resolve_node_entity(context, current.clone(), node_key) else {
                bail_resolve_key_error!(node_key, "unable to resolve key");
            };

            let uris: Vec<_> = match resolve {
                ResolveNode::Node(node_arc) => vec![NodeTopicUri {
                    node: node_arc.downgrade(),
                    topic: ExprContext::new(topic.clone()),
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
            let Some(resolve) = resolve_node_entity(context, current.clone(), node_key) else {
                bail_resolve_key_error!(node_key, "unable to resolve key");
            };

            let uris: Vec<_> = match resolve {
                ResolveNode::Node(shared) => vec![NodeTopicUri {
                    node: shared.downgrade(),
                    topic: ExprContext::new(topic.clone()),
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
    current: ScopeTreeRef,
    link: &mut ServiceLinkContext,
) -> Result<(), Error> {
    let listen = {
        let TopicUri {
            node: node_key,
            topic,
        } = &link.config.listen;
        let Some(resolve) = resolve_node_entity(context, current.clone(), node_key) else {
            bail_resolve_key_error!(node_key, "unable to resolve key");
        };

        let uri = match resolve {
            ResolveNode::Node(node_arc) => NodeTopicUri {
                node: node_arc.downgrade(),
                topic: ExprContext::new(topic.clone()),
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
            let Some(resolve) = resolve_node_entity(context, current.clone(), node_key) else {
                bail_resolve_key_error!(node_key, "unable to resolve key");
            };

            let uris: Vec<_> = match resolve {
                ResolveNode::Node(node_arc) => vec![NodeTopicUri {
                    node: node_arc.downgrade(),
                    topic: ExprContext::new(topic.clone()),
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

#[derive(Debug)]
enum Job {
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

#[derive(Debug)]
struct VisitNodeJob {
    current: ScopeTreeRef,
    current_prefix: KeyOwned,
}

#[derive(Debug)]
struct ResolveLinkJob {
    current: ScopeTreeRef,
    current_prefix: KeyOwned,
}
