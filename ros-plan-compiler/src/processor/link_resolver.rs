use crate::{
    context::link::{PubSubLinkCtx, PubSubLinkShared, ServiceLinkCtx, ServiceLinkShared},
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
        let ResolveLinkJob { current: scope } = job;
        let visitor = Visitor::new(program, &scope);

        scope.with_read(|scope| {
            for link in scope.pubsub_link().values() {
                link.with_write(|mut link| visitor.visit_pubsub_link(&mut link))?;
                associate_pubsub_link_with_node_sockets(link);
            }

            for link in scope.service_link().values() {
                link.with_write(|mut link| visitor.visit_service_link(&mut link))?;
                associate_service_link_with_node_sockets(link);
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
