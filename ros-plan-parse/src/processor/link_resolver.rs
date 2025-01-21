use crate::{
    context::link::{LinkContext, PubsubLinkContext, ServiceLinkContext},
    error::Error,
    resource::Resource,
    scope::{LinkOwned, Scope, ScopeTreeRef},
    utils::{
        resolve_node_client, resolve_node_publication, resolve_node_server,
        resolve_node_subscription,
    },
};
use itertools::Itertools;
use ros_plan_format::key::KeyOwned;
use std::{collections::VecDeque, mem};

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
                {
                    let mut guard = owned.write();
                    resolve_sockets_in_link(context, current.clone(), &mut guard)?;
                }
                associate_link_on_node_sockets(owned);
                Ok(())
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

fn resolve_sockets_in_link(
    context: &Resource,
    current: ScopeTreeRef,
    link: &mut LinkContext,
) -> Result<(), Error> {
    match link {
        LinkContext::PubSub(link) => resolve_sockets_in_pubsub_link(context, current, link)?,
        LinkContext::Service(link) => resolve_sockets_in_service_link(context, current, link)?,
    }
    Ok(())
}

fn resolve_sockets_in_pubsub_link(
    context: &Resource,
    current: ScopeTreeRef,
    link: &mut PubsubLinkContext,
) -> Result<(), Error> {
    {
        let src: Vec<_> = link
            .config
            .src
            .iter()
            .map(|socket_key| {
                let Some(sockets) = resolve_node_publication(context, current.clone(), socket_key)
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

        set_or_panic!(link.src, src);
    }

    {
        let dst: Vec<_> = link
            .config
            .dst
            .iter()
            .map(|socket_key| {
                let Some(sockets) = resolve_node_subscription(context, current.clone(), socket_key)
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

        set_or_panic!(link.dst, dst);
    }
    Ok(())
}

fn resolve_sockets_in_service_link(
    context: &Resource,
    current: ScopeTreeRef,
    link: &mut ServiceLinkContext,
) -> Result<(), Error> {
    let listen = {
        let socket_key = &link.config.listen;
        let Some(socket) = resolve_node_server(context, current.clone(), socket_key) else {
            bail!(socket_key, "the key does not resolve to a server socket");
        };
        socket
    };

    let connect: Vec<_> = link
        .config
        .connect
        .iter()
        .map(|socket_key| {
            let Some(sockets) = resolve_node_client(context, current.clone(), socket_key) else {
                bail!(socket_key, "the key does not resolve to a client socket");
            };
            Ok(sockets)
        })
        .flatten_ok()
        .try_collect()?;

    set_or_panic!(link.listen, listen);
    set_or_panic!(link.connect, connect);

    Ok(())
}

fn associate_link_on_node_sockets(link_owned: LinkOwned) {
    let link_shared = link_owned.downgrade();

    let guard = link_owned.read();
    match &*guard {
        LinkContext::PubSub(pubsub) => {
            for socket_shared in pubsub.src.as_ref().unwrap() {
                let socket_owned = socket_shared.upgrade().unwrap();
                let mut guard = socket_owned.write();
                let pub_ = guard.as_publication_mut().unwrap();
                set_or_panic!(pub_.link_to, link_shared.clone());
            }

            for socket_shared in pubsub.dst.as_ref().unwrap() {
                let socket_owned = socket_shared.upgrade().unwrap();
                let mut guard = socket_owned.write();
                let sub = guard.as_subscription_mut().unwrap();
                set_or_panic!(sub.link_to, link_shared.clone());
            }
        }
        LinkContext::Service(service) => {
            {
                let socket_shared = service.listen.as_ref().unwrap();
                let socket_owned = socket_shared.upgrade().unwrap();
                let mut guard = socket_owned.write();
                let srv = guard.as_server_mut().unwrap();
                set_or_panic!(srv.link_to, link_shared.clone());
            }

            for socket_shared in service.connect.as_ref().unwrap() {
                let socket_owned = socket_shared.upgrade().unwrap();
                let mut guard = socket_owned.write();
                let cli = guard.as_client_mut().unwrap();
                set_or_panic!(cli.link_to, link_shared.clone());
            }
        }
    }
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
