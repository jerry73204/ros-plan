use crate::{
    context::plan_socket::{PlanCliCtx, PlanPubCtx, PlanSrvCtx, PlanSubCtx},
    error::Error,
    program::Program,
    scope::{PlanScopeShared, ScopeRef, ScopeShared},
};
use itertools::Itertools;
use ros_plan_format::key::KeyOwned;
use std::collections::VecDeque;

macro_rules! bail {
    ($key:expr, $reason:expr) => {
        return Err(Error::KeyResolutionError {
            key: (*$key.to_owned().get_stored().unwrap()).clone().into(),
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
    pub fn resolve(&mut self, program: &mut Program) -> Result<(), Error> {
        self.queue.push_back(
            VisitNodeJob {
                current: program.root_scope().into(),
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
                    self.resolve_sockets_in_plan(program, job)?;
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

        current.with_read(|guard| {
            for (suffix, group) in guard.group() {
                let Ok(child_prefix) = &current_prefix / suffix else {
                    unreachable!()
                };

                self.queue.push_back(
                    VisitNodeJob {
                        current: group.clone().into(),
                        current_prefix: child_prefix,
                    }
                    .into(),
                );
            }

            for (suffix, include) in guard.include() {
                include.with_read(|guard| {
                    let Some(plan) = &guard.plan else {
                        todo!();
                    };

                    let Ok(child_prefix) = &current_prefix / suffix else {
                        unreachable!()
                    };

                    self.queue.push_back(
                        VisitNodeJob {
                            current: plan.clone().into(),
                            current_prefix: child_prefix,
                        }
                        .into(),
                    );
                });
            }
        });

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
        program: &mut Program,
        job: ResolveSocketJob,
    ) -> Result<(), Error> {
        let ResolveSocketJob { current } = job;

        current.with_read(|guard| {
            for socket_shared in guard.pub_.values() {
                socket_shared.with_write(|mut socket_guard| -> Result<_, Error> {
                    let visitor = Visitor::new(program, &current);
                    visitor.visit_pub_socket(&mut socket_guard)?;
                    Ok(())
                })?;
            }

            for socket_shared in guard.sub.values() {
                socket_shared.with_write(|mut socket_guard| -> Result<_, Error> {
                    let visitor = Visitor::new(program, &current);
                    visitor.visit_sub_socket(&mut socket_guard)?;
                    Ok(())
                })?;
            }

            for socket_shared in guard.srv.values() {
                socket_shared.with_write(|mut socket_guard| -> Result<_, Error> {
                    let visitor = Visitor::new(program, &current);
                    visitor.visit_srv_socket(&mut socket_guard)?;
                    Ok(())
                })?;
            }

            for socket_shared in guard.cli.values() {
                socket_shared.with_write(|mut socket_guard| -> Result<_, Error> {
                    let visitor = Visitor::new(program, &current);
                    visitor.visit_cli_socket(&mut socket_guard)?;
                    Ok(())
                })?;
            }

            Ok(())
        })
    }
}

struct Visitor<'a> {
    program: &'a mut Program,
    current: &'a PlanScopeShared,
}

impl<'a> Visitor<'a> {
    pub fn new(program: &'a mut Program, current: &'a PlanScopeShared) -> Self {
        Self { program, current }
    }

    pub fn visit_pub_socket(&self, pub_: &mut PlanPubCtx) -> Result<(), Error> {
        let src: Result<Vec<_>, _> = pub_
            .src_key
            .iter()
            .map(|socket_key| {
                self.program
                    .selector(&self.current.clone().into())
                    .find_plan_or_node_pub(socket_key.get_stored().unwrap())
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
        set_or_panic!(pub_.src, src);
        Ok(())
    }

    pub fn visit_sub_socket(&self, sub: &mut PlanSubCtx) -> Result<(), Error> {
        let dst: Result<Vec<_>, _> = sub
            .dst_key
            .iter()
            .map(|socket_key| {
                self.program
                    .selector(&self.current.clone().into())
                    .find_plan_or_node_sub(socket_key.get_stored().unwrap())
                    .ok_or(socket_key)?
                    .to_node_sub()
                    .ok_or(socket_key)
            })
            .flatten_ok()
            .collect();
        let dst = match dst {
            Ok(dst) => dst,
            Err(bad_key) => {
                bail!(bad_key, "the key does not resolve to a publication socket");
            }
        };
        set_or_panic!(sub.dst, dst);
        Ok(())
    }

    pub fn visit_srv_socket(&self, srv: &mut PlanSrvCtx) -> Result<(), Error> {
        let socket_key = &srv.listen_key;
        let listen = (|| {
            self.program
                .selector(&self.current.clone().into())
                .find_plan_or_node_srv(socket_key.get_stored().unwrap())?
                .to_node_srv()
        })();
        let Some(listen) = listen else {
            bail!(socket_key, "the key does not resolve to a server socket");
        };

        set_or_panic!(srv.listen, listen);
        Ok(())
    }

    pub fn visit_cli_socket(&self, cli: &mut PlanCliCtx) -> Result<(), Error> {
        let connect: Result<Vec<_>, _> = cli
            .connect_key
            .iter()
            .map(|socket_key| {
                self.program
                    .selector(&self.current.clone().into())
                    .find_plan_or_node_cli(socket_key.get_stored().unwrap())
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

        set_or_panic!(cli.connect, connect);
        Ok(())
    }
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
    current: PlanScopeShared,
}
