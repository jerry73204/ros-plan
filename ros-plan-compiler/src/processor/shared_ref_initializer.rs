use crate::{
    context::{
        link::{PubSubLinkCtx, PubSubLinkShared, ServiceLinkCtx, ServiceLinkShared},
        node::NodeShared,
        node_socket::{
            NodeCliCtx, NodeCliShared, NodePubCtx, NodePubShared, NodeSrvCtx, NodeSrvShared,
            NodeSubCtx, NodeSubShared,
        },
        plan_socket::{
            PlanCliCtx, PlanCliShared, PlanPubCtx, PlanPubShared, PlanSrvCtx, PlanSrvShared,
            PlanSubCtx, PlanSubShared,
        },
    },
    error::Error,
    scope::{GroupScope, GroupScopeShared, PlanScope, PlanScopeShared, ScopeShared},
    Program,
};
use indexmap::IndexMap;
use ros_plan_format::{link::LinkIdent, node::NodeIdent, plan_socket::PlanSocketIdent};
use std::collections::VecDeque;

#[derive(Debug, Default)]
pub struct SharedRefInitializer {
    queue: VecDeque<ScopeShared>,
}

impl SharedRefInitializer {
    pub fn initialize(&mut self, program: &Program) -> Result<(), Error> {
        {
            let Program {
                pubsub_link_tab,
                service_link_tab,
                plan_pub_tab,
                plan_sub_tab,
                plan_srv_tab,
                plan_cli_tab,
                node_pub_tab,
                node_sub_tab,
                node_srv_tab,
                node_cli_tab,
                ..
            } = program;

            for (_id, link) in pubsub_link_tab.read_inner().iter() {
                update_pubsub_link_context(program, &mut link.write())?;
            }

            for (_id, link) in service_link_tab.read_inner().iter() {
                update_service_link_context(program, &mut link.write())?;
            }

            for (_id, socket) in plan_pub_tab.read_inner().iter() {
                update_plan_pub_context(program, &mut socket.write())?;
            }

            for (_id, socket) in plan_sub_tab.read_inner().iter() {
                update_plan_sub_context(program, &mut socket.write())?;
            }

            for (_id, socket) in plan_srv_tab.read_inner().iter() {
                update_plan_srv_context(program, &mut socket.write())?;
            }

            for (_id, socket) in plan_cli_tab.read_inner().iter() {
                update_plan_cli_context(program, &mut socket.write())?;
            }

            for (_id, socket) in node_pub_tab.read_inner().iter() {
                update_node_pub_context(program, &mut socket.write())?;
            }

            for (_id, socket) in node_sub_tab.read_inner().iter() {
                update_node_sub_context(program, &mut socket.write())?;
            }

            for (_id, socket) in node_srv_tab.read_inner().iter() {
                update_node_srv_context(program, &mut socket.write())?;
            }

            for (_id, socket) in node_cli_tab.read_inner().iter() {
                update_node_cli_context(program, &mut socket.write())?;
            }
        }

        {
            let root = program.root();
            self.queue.push_back(root.into());

            while let Some(curr) = self.queue.pop_front() {
                self.visit_scope(program, curr)?;
            }
        }
        Ok(())
    }

    fn visit_scope(&mut self, program: &Program, current: ScopeShared) -> Result<(), Error> {
        match &current {
            ScopeShared::Include(scope) => update_plan_file_scope(program, scope)?,
            ScopeShared::Group(scope) => update_group_scope(program, scope)?,
        }
        Ok(())
    }
}

fn update_plan_file_scope(program: &Program, shared: &PlanScopeShared) -> Result<(), Error> {
    shared.with_write(|mut scope| {
        let PlanScope {
            node: node_map,
            pubsub_link: pubsub_link_map,
            service_link: service_link_map,
            pub_: pub_map,
            sub: sub_map,
            srv: srv_map,
            cli: cli_map,
            ..
        } = &mut *scope;

        update_node_map(program, node_map)?;
        update_pubsub_link_map(program, pubsub_link_map)?;
        update_service_link_map(program, service_link_map)?;
        update_pub_map(program, pub_map)?;
        update_sub_map(program, sub_map)?;
        update_srv_map(program, srv_map)?;
        update_cli_map(program, cli_map)?;
        Ok(())
    })
}

fn update_group_scope(program: &Program, shared: &GroupScopeShared) -> Result<(), Error> {
    shared.with_write(|mut scope| {
        let GroupScope {
            node: node_map,
            pubsub_link: pubsub_link_map,
            service_link: service_link_map,
            ..
        } = &mut *scope;

        update_node_map(program, node_map)?;
        update_pubsub_link_map(program, pubsub_link_map)?;
        update_service_link_map(program, service_link_map)?;
        Ok(())
    })
}

fn update_node_map(
    program: &Program,
    node_map: &mut IndexMap<NodeIdent, NodeShared>,
) -> Result<(), Error> {
    for shared in node_map.values_mut() {
        let Some(owned) = program.node_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_pubsub_link_map(
    program: &Program,
    link_map: &mut IndexMap<LinkIdent, PubSubLinkShared>,
) -> Result<(), Error> {
    for shared in link_map.values_mut() {
        let Some(owned) = program.pubsub_link_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_service_link_map(
    program: &Program,
    link_map: &mut IndexMap<LinkIdent, ServiceLinkShared>,
) -> Result<(), Error> {
    for shared in link_map.values_mut() {
        let Some(owned) = program.service_link_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_pub_map(
    program: &Program,
    socket_map: &mut IndexMap<PlanSocketIdent, PlanPubShared>,
) -> Result<(), Error> {
    for shared in socket_map.values_mut() {
        let Some(owned) = program.plan_pub_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_sub_map(
    program: &Program,
    socket_map: &mut IndexMap<PlanSocketIdent, PlanSubShared>,
) -> Result<(), Error> {
    for shared in socket_map.values_mut() {
        let Some(owned) = program.plan_sub_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_srv_map(
    program: &Program,
    socket_map: &mut IndexMap<PlanSocketIdent, PlanSrvShared>,
) -> Result<(), Error> {
    for shared in socket_map.values_mut() {
        let Some(owned) = program.plan_srv_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_cli_map(
    program: &Program,
    socket_map: &mut IndexMap<PlanSocketIdent, PlanCliShared>,
) -> Result<(), Error> {
    for shared in socket_map.values_mut() {
        let Some(owned) = program.plan_cli_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_pubsub_link_context(program: &Program, link: &mut PubSubLinkCtx) -> Result<(), Error> {
    let PubSubLinkCtx {
        src_socket,
        dst_socket,
        ..
    } = link;

    for uri in src_socket.iter_mut().flatten() {
        initialize_node_pub_shared(program, uri)?;
    }

    for uri in dst_socket.iter_mut().flatten() {
        initialize_node_sub_shared(program, uri)?;
    }

    Ok(())
}

fn update_service_link_context(program: &Program, link: &mut ServiceLinkCtx) -> Result<(), Error> {
    let ServiceLinkCtx {
        listen_socket,
        connect_socket,
        ..
    } = link;

    if let Some(uri) = listen_socket {
        initialize_node_srv_shared(program, uri)?;
    }

    for uri in connect_socket.iter_mut().flatten() {
        initialize_node_cli_shared(program, uri)?;
    }
    Ok(())
}

fn update_plan_pub_context(program: &Program, socket: &mut PlanPubCtx) -> Result<(), Error> {
    let PlanPubCtx { src, .. } = socket;

    for uri in src.iter_mut().flatten() {
        initialize_node_pub_shared(program, uri)?;
    }

    Ok(())
}

fn update_plan_sub_context(program: &Program, socket: &mut PlanSubCtx) -> Result<(), Error> {
    let PlanSubCtx { dst, .. } = socket;

    for uri in dst.iter_mut().flatten() {
        initialize_node_sub_shared(program, uri)?;
    }

    Ok(())
}

fn update_plan_srv_context(program: &Program, socket: &mut PlanSrvCtx) -> Result<(), Error> {
    let PlanSrvCtx { listen, .. } = socket;

    if let Some(uri) = listen {
        initialize_node_srv_shared(program, uri)?;
    }

    Ok(())
}

fn update_plan_cli_context(program: &Program, socket: &mut PlanCliCtx) -> Result<(), Error> {
    let PlanCliCtx { connect, .. } = socket;

    for uri in connect.iter_mut().flatten() {
        initialize_node_cli_shared(program, uri)?;
    }

    Ok(())
}

fn update_node_pub_context(program: &Program, socket: &mut NodePubCtx) -> Result<(), Error> {
    let NodePubCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_pubsub_link_shared(program, link_to)?;
    }
    Ok(())
}

fn update_node_sub_context(program: &Program, socket: &mut NodeSubCtx) -> Result<(), Error> {
    let NodeSubCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_pubsub_link_shared(program, link_to)?;
    }
    Ok(())
}

fn update_node_srv_context(program: &Program, socket: &mut NodeSrvCtx) -> Result<(), Error> {
    let NodeSrvCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_service_link_shared(program, link_to)?;
    }

    Ok(())
}

fn update_node_cli_context(program: &Program, socket: &mut NodeCliCtx) -> Result<(), Error> {
    let NodeCliCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_service_link_shared(program, link_to)?;
    }
    Ok(())
}

fn initialize_node_pub_shared(program: &Program, shared: &mut NodePubShared) -> Result<(), Error> {
    let Some(owned) = program.node_pub_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}

fn initialize_node_sub_shared(program: &Program, shared: &mut NodeSubShared) -> Result<(), Error> {
    let Some(owned) = program.node_sub_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}

fn initialize_node_srv_shared(program: &Program, shared: &mut NodeSrvShared) -> Result<(), Error> {
    let Some(owned) = program.node_srv_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}

fn initialize_node_cli_shared(program: &Program, shared: &mut NodeCliShared) -> Result<(), Error> {
    let Some(owned) = program.node_cli_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}

fn initialize_pubsub_link_shared(
    program: &Program,
    shared: &mut PubSubLinkShared,
) -> Result<(), Error> {
    let Some(owned) = program.pubsub_link_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}

fn initialize_service_link_shared(
    program: &Program,
    shared: &mut ServiceLinkShared,
) -> Result<(), Error> {
    let Some(owned) = program.service_link_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}
