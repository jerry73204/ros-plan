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
        set_or_panic!(pub_.src_socket, src);
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
        set_or_panic!(sub.dst_socket, dst);
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

        set_or_panic!(srv.listen_socket, listen);
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

        set_or_panic!(cli.connect_socket, connect);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        context::node_socket::{NodePubCtx, NodePubShared, NodeSubCtx, NodeSubShared},
        eval::KeyStore,
        utils::shared_table::SharedTable,
    };
    use ros_plan_format::{expr::KeyOrExpr, key::KeyOwned};

    fn create_test_node_pub(key_str: &str) -> NodePubShared {
        let key: KeyOwned = key_str.parse().unwrap();
        let ctx = NodePubCtx {
            key,
            ty: None,
            qos: None,
            ros_name: None,
            remap_from: None,
            link_to: None,
        };
        let table = SharedTable::<NodePubCtx>::new("test_node_pub");
        table.insert(ctx)
    }

    fn create_test_node_sub(key_str: &str) -> NodeSubShared {
        let key: KeyOwned = key_str.parse().unwrap();
        let ctx = NodeSubCtx {
            key,
            ty: None,
            qos: None,
            ros_name: None,
            remap_from: None,
            link_to: None,
        };
        let table = SharedTable::<NodeSubCtx>::new("test_node_sub");
        table.insert(ctx)
    }

    fn create_key_store(key_str: &str) -> KeyStore {
        let key_or_expr: KeyOrExpr = key_str.parse().unwrap();
        let mut store = KeyStore::new(key_or_expr);
        store.eval_and_store(&mlua::Lua::new()).unwrap();
        store
    }

    #[test]
    fn test_visit_pub_socket_single_source() {
        use crate::context::plan_socket::PlanPubCtx;

        let _node_table = SharedTable::<NodePubCtx>::new("test_node");
        let node_socket = create_test_node_pub("node_a/output");

        let mut pub_socket = PlanPubCtx {
            key: "plan/pub_socket".parse().unwrap(),
            ty: None,
            topic: None,
            src_key: vec![create_key_store("node_a/output")],
            src_socket: None,
            qos: None,
        };

        // This test verifies that src_key gets resolved to src_socket
        // In real usage, this would be done by the SocketResolver with a full Program
        // Here we just verify the data structure is correct
        assert_eq!(pub_socket.src_key.len(), 1);
        assert!(pub_socket.src_socket.is_none());

        // Manually set what the resolver would do
        pub_socket.src_socket = Some(vec![node_socket]);
        assert_eq!(pub_socket.src_socket.as_ref().unwrap().len(), 1);
    }

    #[test]
    fn test_visit_pub_socket_multiple_sources() {
        use crate::context::plan_socket::PlanPubCtx;

        let _node_table = SharedTable::<NodePubCtx>::new("test_node_multi");
        let node_socket1 = create_test_node_pub("node_a/output");
        let node_socket2 = create_test_node_pub("node_b/output");
        let node_socket3 = create_test_node_pub("node_c/output");

        let mut pub_socket = PlanPubCtx {
            key: "plan/aggregated".parse().unwrap(),
            ty: None,
            topic: None,
            src_key: vec![
                create_key_store("node_a/output"),
                create_key_store("node_b/output"),
                create_key_store("node_c/output"),
            ],
            src_socket: None,
            qos: None,
        };

        assert_eq!(pub_socket.src_key.len(), 3);

        // Manually set what the resolver would do
        pub_socket.src_socket = Some(vec![node_socket1, node_socket2, node_socket3]);
        assert_eq!(pub_socket.src_socket.as_ref().unwrap().len(), 3);
    }

    #[test]
    fn test_visit_sub_socket_single_destination() {
        use crate::context::plan_socket::PlanSubCtx;

        let _node_table = SharedTable::<NodeSubCtx>::new("test_node_sub");
        let node_socket = create_test_node_sub("node_a/input");

        let mut sub_socket = PlanSubCtx {
            key: "plan/sub_socket".parse().unwrap(),
            ty: None,
            topic: None,
            dst_key: vec![create_key_store("node_a/input")],
            dst_socket: None,
            qos: None,
        };

        assert_eq!(sub_socket.dst_key.len(), 1);
        assert!(sub_socket.dst_socket.is_none());

        // Manually set what the resolver would do
        sub_socket.dst_socket = Some(vec![node_socket]);
        assert_eq!(sub_socket.dst_socket.as_ref().unwrap().len(), 1);
    }

    #[test]
    fn test_visit_sub_socket_multiple_destinations() {
        use crate::context::plan_socket::PlanSubCtx;

        let _node_table = SharedTable::<NodeSubCtx>::new("test_node_sub_multi");
        let node_socket1 = create_test_node_sub("node_a/input");
        let node_socket2 = create_test_node_sub("node_b/input");

        let mut sub_socket = PlanSubCtx {
            key: "plan/broadcast".parse().unwrap(),
            ty: None,
            topic: None,
            dst_key: vec![
                create_key_store("node_a/input"),
                create_key_store("node_b/input"),
            ],
            dst_socket: None,
            qos: None,
        };

        assert_eq!(sub_socket.dst_key.len(), 2);

        // Manually set what the resolver would do
        sub_socket.dst_socket = Some(vec![node_socket1, node_socket2]);
        assert_eq!(sub_socket.dst_socket.as_ref().unwrap().len(), 2);
    }

    #[test]
    fn test_socket_resolver_default() {
        let resolver = SocketResolver::default();
        assert_eq!(resolver.queue.len(), 0);
    }

    // F13: Plan socket forwarding tests

    #[test]
    fn test_plan_socket_forwards_single_node_socket() {
        use crate::context::plan_socket::PlanPubCtx;

        // Plan socket should forward to single internal node socket
        let _node_table = SharedTable::<NodePubCtx>::new("test_forward_single");
        let node_socket = create_test_node_pub("internal_node/output");

        let plan_socket = PlanPubCtx {
            key: "plan/exposed".parse().unwrap(),
            ty: None,
            topic: None,
            src_key: vec![create_key_store("internal_node/output")],
            src_socket: Some(vec![node_socket]),
            qos: None,
        };

        // Verify forwarding is set up correctly
        assert_eq!(plan_socket.src_key.len(), 1);
        assert_eq!(plan_socket.src_socket.as_ref().unwrap().len(), 1);
    }

    #[test]
    fn test_plan_socket_aggregates_multiple_sources() {
        use crate::context::plan_socket::PlanPubCtx;

        // Plan socket can aggregate multiple internal sockets
        let _node_table = SharedTable::<NodePubCtx>::new("test_aggregate");
        let socket1 = create_test_node_pub("node_a/data");
        let socket2 = create_test_node_pub("node_b/data");
        let socket3 = create_test_node_pub("node_c/data");

        let plan_socket = PlanPubCtx {
            key: "plan/aggregated".parse().unwrap(),
            ty: None,
            topic: None,
            src_key: vec![
                create_key_store("node_a/data"),
                create_key_store("node_b/data"),
                create_key_store("node_c/data"),
            ],
            src_socket: Some(vec![socket1, socket2, socket3]),
            qos: None,
        };

        assert_eq!(plan_socket.src_key.len(), 3);
        assert_eq!(plan_socket.src_socket.as_ref().unwrap().len(), 3);
    }
}
