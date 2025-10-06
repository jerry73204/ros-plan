use crate::{
    context::link::{PubSubLinkCtx, PubSubLinkShared, ServiceLinkCtx, ServiceLinkShared},
    error::Error,
    program::Program,
    scope::{ScopeRef, ScopeShared},
};
use itertools::Itertools;
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
        self.queue.push_back(Job {
            current: program.root_scope().into(),
        });

        // Perform traversal
        while let Some(job) = self.queue.pop_front() {
            self.visit_scope(program, job)?;
        }

        Ok(())
    }

    fn visit_scope(&mut self, program: &Program, job: Job) -> Result<(), Error> {
        let Job { current: shared } = job;

        shared.with_read(|guard| -> Result<_, Error> {
            let visitor = Visitor::new(program, &shared);

            for link in guard.pubsub_link().values() {
                link.with_write(|mut link| visitor.visit_pubsub_link(&mut link))?;
                associate_pubsub_link_with_node_sockets(link);
            }

            for link in guard.service_link().values() {
                link.with_write(|mut link| visitor.visit_service_link(&mut link))?;
                associate_service_link_with_node_sockets(link);
            }

            // Schedule a job to resolve links if the node has a plan
            // or a group
            for (_suffix, group) in guard.group() {
                self.queue.push_back(Job {
                    current: group.clone().into(),
                });
            }

            for include in guard.include().values() {
                include.with_read(|guard| {
                    let Some(plan) = &guard.plan else {
                        todo!();
                    };

                    self.queue.push_back(Job {
                        current: plan.clone().into(),
                    });
                });
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
struct Job {
    pub current: ScopeShared,
}
