use crate::{
    context::plan_socket::{
        PlanClientContext, PlanPublicationContext, PlanServerContext, PlanSocketContext,
        PlanSubscriptionContext,
    },
    error::Error,
    resource::Resource,
    scope::{PlanFileScopeShared, ScopeRefExt, ScopeShared},
    utils::{
        resolve_node_client, resolve_node_publication, resolve_node_server,
        resolve_node_subscription,
    },
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
pub struct SocketResolver {
    queue: VecDeque<Job>,
}

impl SocketResolver {
    pub fn traverse(&mut self, context: &mut Resource) -> Result<(), Error> {
        self.queue.push_back(
            VisitNodeJob {
                current: context.root().unwrap().downgrade().into(),
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
                    self.resolve_sockets_in_plan(context, job)?;
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

        let owned = current.upgrade().unwrap();
        let guard = owned.read();
        {
            // Visit children
            for (suffix, subscope) in guard.subscope_iter() {
                let Ok(child_prefix) = &current_prefix / suffix else {
                    unreachable!()
                };

                self.queue.push_back(
                    VisitNodeJob {
                        current: subscope.clone(),
                        current_prefix: child_prefix,
                    }
                    .into(),
                );
            }
        }

        if let Ok(plan_shared) = current.try_into_include() {
            self.queue.push_back(
                ResolveSocketJob {
                    current: plan_shared.clone(),
                }
                .into(),
            );
        }

        Ok(())
    }

    fn resolve_sockets_in_plan(
        &self,
        resource: &mut Resource,
        job: ResolveSocketJob,
    ) -> Result<(), Error> {
        let ResolveSocketJob {
            current,
            // current_prefix: _,
        } = job;

        // Take out the socket_map from the plan context
        let owned = current.upgrade().unwrap();
        let guard = owned.read();

        // Resolve URIs in the socket_map
        for socket_shared in guard.socket_map.values() {
            let socket_owned = socket_shared.upgrade().unwrap();
            let mut socket_guard = socket_owned.write();
            resolve_socket_topics(resource, current.clone(), &mut socket_guard)?;
        }

        Ok(())
    }
}

fn resolve_socket_topics(
    resource: &mut Resource,
    current: PlanFileScopeShared,
    socket_ctx: &mut PlanSocketContext,
) -> Result<(), Error> {
    match socket_ctx {
        PlanSocketContext::Publication(pub_ctx) => {
            resolve_pub_socket_topics(resource, current, pub_ctx)?
        }
        PlanSocketContext::Subscription(sub_ctx) => {
            resolve_sub_socket_topics(resource, current, sub_ctx)?
        }
        PlanSocketContext::Server(srv_ctx) => {
            resolve_srv_socket_topics(resource, current, srv_ctx)?
        }
        PlanSocketContext::Client(qry_ctx) => {
            resolve_qry_socket_topics(resource, current, qry_ctx)?
        }
    }
    Ok(())
}

fn resolve_pub_socket_topics(
    resource: &mut Resource,
    current: PlanFileScopeShared,
    pub_: &mut PlanPublicationContext,
) -> Result<(), Error> {
    let src: Vec<_> = pub_
        .config
        .src
        .iter()
        .map(|socket_key| {
            let Some(sockets) =
                resolve_node_publication(resource, current.clone().into(), socket_key)
            else {
                bail!(
                    socket_key,
                    "the key does not resolve to a publication socket"
                );
            };
            Ok(sockets)
        })
        .flatten_ok()
        .try_collect()?;

    set_or_panic!(pub_.src, src);
    Ok(())
}

fn resolve_sub_socket_topics(
    resource: &mut Resource,
    current: PlanFileScopeShared,
    sub: &mut PlanSubscriptionContext,
) -> Result<(), Error> {
    let dst: Vec<_> = sub
        .config
        .dst
        .iter()
        .map(|socket_key| {
            let Some(sockets) =
                resolve_node_subscription(resource, current.clone().into(), socket_key)
            else {
                bail!(
                    socket_key,
                    "the key does not resolve to a subscription socket"
                );
            };
            Ok(sockets)
        })
        .flatten_ok()
        .try_collect()?;

    set_or_panic!(sub.dst, dst);
    Ok(())
}

fn resolve_srv_socket_topics(
    resource: &mut Resource,
    current: PlanFileScopeShared,
    srv: &mut PlanServerContext,
) -> Result<(), Error> {
    let socket_key = &srv.config.listen;
    let Some(socket) = resolve_node_server(resource, current.clone().into(), socket_key) else {
        bail!(socket_key, "the key does not resolve to a server socket");
    };

    set_or_panic!(srv.listen, socket);
    Ok(())
}

fn resolve_qry_socket_topics(
    resource: &mut Resource,
    current: PlanFileScopeShared,
    qry: &mut PlanClientContext,
) -> Result<(), Error> {
    let connect: Vec<_> = qry
        .config
        .connect
        .iter()
        .map(|socket_key| {
            let Some(sockets) = resolve_node_client(resource, current.clone().into(), socket_key)
            else {
                bail!(socket_key, "the key does not resolve to a client socket");
            };
            Ok(sockets)
        })
        .flatten_ok()
        .try_collect()?;

    set_or_panic!(qry.connect, connect);
    Ok(())
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
    current: ScopeShared,
    current_prefix: KeyOwned,
}

#[derive(Debug)]
pub struct ResolveSocketJob {
    current: PlanFileScopeShared,
    // current_prefix: KeyOwned,
}

// #[derive(Debug)]
// enum ResolveNode {
//     Node(NodeOwned),
//     Socket(SocketOwned),
// }

// impl From<SocketOwned> for ResolveNode {
//     fn from(v: SocketOwned) -> Self {
//         Self::Socket(v)
//     }
// }

// impl From<NodeOwned> for ResolveNode {
//     fn from(v: NodeOwned) -> Self {
//         Self::Node(v)
//     }
// }
