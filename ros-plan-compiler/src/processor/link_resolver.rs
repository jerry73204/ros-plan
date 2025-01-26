use crate::{
    context::link::{LinkCtx, LinkShared, PubsubLinkCtx, ServiceLinkCtx},
    error::Error,
    program::Program,
    scope::{ScopeRef, ScopeRefExt, ScopeShared},
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
        self.queue.push_back(
            VisitNodeJob {
                current: program.root().into(),
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
                    self.resolve_link(program, job)?;
                }
            }
        }

        Ok(())
    }

    fn visit_node(&mut self, job: VisitNodeJob) -> Result<(), Error> {
        let VisitNodeJob {
            current: shared,
            current_prefix,
        } = job;

        shared.with_read(|guard| {
            // Schedule jobs to visit child nodes.
            for (suffix, subscope) in guard.subscope_iter() {
                let Ok(child_prefix) = &current_prefix / suffix else {
                    unreachable!()
                };

                self.queue.push_back(
                    VisitNodeJob {
                        current: subscope,
                        current_prefix: child_prefix,
                    }
                    .into(),
                );
            }

            // Schedule a job to resolve links if the node has a plan or a
            // group
            for (_suffix, subscope) in guard.subscope_iter() {
                self.queue
                    .push_back(ResolveLinkJob { current: subscope }.into());
            }

            Ok(())
        })
    }

    fn resolve_link(&mut self, program: &mut Program, job: ResolveLinkJob) -> Result<(), Error> {
        let ResolveLinkJob {
            current: scope_shared,
        } = job;
        let visitor = Visitor::new(program, &scope_shared);

        scope_shared.with_read(|scope_guard| {
            let link_map = scope_guard.link_map();

            // Resolve links
            for link_shared in link_map.values() {
                link_shared.with_write(|mut link_guard| visitor.visit_link(&mut link_guard))?;
                associate_link_on_node_sockets(link_shared);
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

    pub fn visit_link(&self, link: &mut LinkCtx) -> Result<(), Error> {
        match link {
            LinkCtx::PubSub(link) => self.visit_pubsub_link(link)?,
            LinkCtx::Service(link) => self.visit_service_link(link)?,
        }
        Ok(())
    }

    pub fn visit_pubsub_link(&self, link: &mut PubsubLinkCtx) -> Result<(), Error> {
        {
            let src: Result<Vec<_>, _> = link
                .config
                .src
                .iter()
                .map(|socket_key| {
                    self.program
                        .selector(self.current)
                        .find_node_pub(socket_key)
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
            set_or_panic!(link.src, src);
        }

        {
            let dst: Result<Vec<_>, _> = link
                .config
                .dst
                .iter()
                .map(|socket_key| {
                    self.program
                        .selector(self.current)
                        .find_node_sub(socket_key)
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
            set_or_panic!(link.dst, dst);
        }
        Ok(())
    }

    fn visit_service_link(&self, link: &mut ServiceLinkCtx) -> Result<(), Error> {
        let listen = {
            let socket_key = &link.config.listen;
            let listen = self
                .program
                .selector(self.current)
                .find_node_srv(socket_key);
            let Some(listen) = listen else {
                bail!(socket_key, "the key does not resolve to a server socket");
            };
            listen
        };

        let connect: Result<Vec<_>, _> = link
            .config
            .connect
            .iter()
            .map(|socket_key| {
                self.program
                    .selector(self.current)
                    .find_node_cli(socket_key)
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

        set_or_panic!(link.listen, listen);
        set_or_panic!(link.connect, connect);

        Ok(())
    }
}

fn associate_link_on_node_sockets(link_shared: &LinkShared) {
    link_shared.with_read(|link_guard| match &*link_guard {
        LinkCtx::PubSub(pubsub) => {
            for socket_shared in pubsub.src.as_ref().unwrap() {
                socket_shared.with_write(|mut guard| {
                    let pub_ = guard.as_pub_mut().unwrap();
                    set_or_panic!(pub_.link_to, link_shared.clone());
                });
            }

            for socket_shared in pubsub.dst.as_ref().unwrap() {
                socket_shared.with_write(|mut guard| {
                    let sub = guard.as_sub_mut().unwrap();
                    set_or_panic!(sub.link_to, link_shared.clone());
                });
            }
        }
        LinkCtx::Service(service) => {
            {
                let socket_shared = service.listen.as_ref().unwrap();
                socket_shared.with_write(|mut guard| {
                    let srv = guard.as_srv_mut().unwrap();
                    set_or_panic!(srv.link_to, link_shared.clone());
                });
            }

            for socket_shared in service.connect.as_ref().unwrap() {
                socket_shared.with_write(|mut guard| {
                    let cli = guard.as_cli_mut().unwrap();
                    set_or_panic!(cli.link_to, link_shared.clone());
                });
            }
        }
    });
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
    current: ScopeShared,
    current_prefix: KeyOwned,
}

#[derive(Debug)]
struct ResolveLinkJob {
    current: ScopeShared,
}
