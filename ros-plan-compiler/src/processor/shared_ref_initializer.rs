use crate::{
    context::{
        link::{LinkCtx, LinkShared, PubsubLinkCtx, ServiceLinkCtx},
        node::NodeShared,
        node_socket::{
            NodeCliCtx, NodePubCtx, NodeSocketCtx, NodeSocketShared, NodeSrvCtx, NodeSubCtx,
        },
        plan_socket::{
            PlanCliCtx, PlanPubCtx, PlanSocketCtx, PlanSocketShared, PlanSrvCtx, PlanSubCtx,
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
                link_tab,
                plan_socket_tab,
                node_socket_tab,
                ..
            } = program;

            for (_id, link) in link_tab.read_inner().iter() {
                update_link_context(program, &mut link.write())?;
            }

            for (_id, socket) in plan_socket_tab.read_inner().iter() {
                update_plan_socket_context(program, &mut socket.write())?;
            }

            for (_id, socket) in node_socket_tab.read_inner().iter() {
                update_node_socket_context(program, &mut socket.write())?;
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
            socket_map,
            node_map,
            link_map,
            ..
        } = &mut *scope;

        update_node_map(program, node_map)?;
        update_link_map(program, link_map)?;
        update_socket_map(program, socket_map)?;
        Ok(())
    })
}

fn update_group_scope(program: &Program, shared: &GroupScopeShared) -> Result<(), Error> {
    shared.with_write(|mut scope| {
        let GroupScope {
            node_map, link_map, ..
        } = &mut *scope;

        update_node_map(program, node_map)?;
        update_link_map(program, link_map)?;
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

fn update_link_map(
    program: &Program,
    link_map: &mut IndexMap<LinkIdent, LinkShared>,
) -> Result<(), Error> {
    for shared in link_map.values_mut() {
        let Some(owned) = program.link_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_socket_map(
    program: &Program,
    socket_map: &mut IndexMap<PlanSocketIdent, PlanSocketShared>,
) -> Result<(), Error> {
    for shared in socket_map.values_mut() {
        let Some(owned) = program.plan_socket_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_link_context(program: &Program, link: &mut LinkCtx) -> Result<(), Error> {
    match link {
        LinkCtx::PubSub(link) => update_pubsub_link_context(program, link)?,
        LinkCtx::Service(link) => update_service_link_context(program, link)?,
    }
    Ok(())
}

fn update_pubsub_link_context(program: &Program, link: &mut PubsubLinkCtx) -> Result<(), Error> {
    let PubsubLinkCtx { src, dst, .. } = link;

    for uri in src.iter_mut().flatten() {
        initialize_node_socket_shared(program, uri)?;
    }

    for uri in dst.iter_mut().flatten() {
        initialize_node_socket_shared(program, uri)?;
    }

    Ok(())
}

fn update_service_link_context(program: &Program, link: &mut ServiceLinkCtx) -> Result<(), Error> {
    let ServiceLinkCtx {
        listen, connect, ..
    } = link;

    if let Some(uri) = listen {
        initialize_node_socket_shared(program, uri)?;
    }

    for uri in connect.iter_mut().flatten() {
        initialize_node_socket_shared(program, uri)?;
    }
    Ok(())
}

fn update_plan_socket_context(program: &Program, socket: &mut PlanSocketCtx) -> Result<(), Error> {
    match socket {
        PlanSocketCtx::Pub(socket) => update_plan_pub_context(program, socket)?,
        PlanSocketCtx::Sub(socket) => update_plan_sub_context(program, socket)?,
        PlanSocketCtx::Srv(socket) => update_plan_server_context(program, socket)?,
        PlanSocketCtx::Cli(socket) => update_plan_client_context(program, socket)?,
    }
    Ok(())
}

fn update_plan_pub_context(program: &Program, socket: &mut PlanPubCtx) -> Result<(), Error> {
    let PlanPubCtx { src, .. } = socket;

    for uri in src.iter_mut().flatten() {
        initialize_node_socket_shared(program, uri)?;
    }

    Ok(())
}

fn update_plan_sub_context(program: &Program, socket: &mut PlanSubCtx) -> Result<(), Error> {
    let PlanSubCtx { dst, .. } = socket;

    for uri in dst.iter_mut().flatten() {
        initialize_node_socket_shared(program, uri)?;
    }

    Ok(())
}

fn update_plan_server_context(program: &Program, socket: &mut PlanSrvCtx) -> Result<(), Error> {
    let PlanSrvCtx { listen, .. } = socket;

    if let Some(uri) = listen {
        initialize_node_socket_shared(program, uri)?;
    }

    Ok(())
}

fn update_plan_client_context(program: &Program, socket: &mut PlanCliCtx) -> Result<(), Error> {
    let PlanCliCtx { connect, .. } = socket;

    for uri in connect.iter_mut().flatten() {
        initialize_node_socket_shared(program, uri)?;
    }

    Ok(())
}

fn update_node_socket_context(program: &Program, socket: &mut NodeSocketCtx) -> Result<(), Error> {
    match socket {
        NodeSocketCtx::Pub(socket) => update_node_pub_context(program, socket)?,
        NodeSocketCtx::Sub(socket) => update_node_sub_context(program, socket)?,
        NodeSocketCtx::Srv(socket) => update_node_server_context(program, socket)?,
        NodeSocketCtx::Cli(socket) => update_node_client_context(program, socket)?,
    }
    Ok(())
}

fn update_node_pub_context(program: &Program, socket: &mut NodePubCtx) -> Result<(), Error> {
    let NodePubCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_link_shared(program, link_to)?;
    }
    Ok(())
}

fn update_node_sub_context(program: &Program, socket: &mut NodeSubCtx) -> Result<(), Error> {
    let NodeSubCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_link_shared(program, link_to)?;
    }
    Ok(())
}

fn update_node_server_context(program: &Program, socket: &mut NodeSrvCtx) -> Result<(), Error> {
    let NodeSrvCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_link_shared(program, link_to)?;
    }

    Ok(())
}

fn update_node_client_context(program: &Program, socket: &mut NodeCliCtx) -> Result<(), Error> {
    let NodeCliCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_link_shared(program, link_to)?;
    }
    Ok(())
}

fn initialize_node_socket_shared(
    program: &Program,
    shared: &mut NodeSocketShared,
) -> Result<(), Error> {
    let Some(owned) = program.node_socket_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}

fn initialize_link_shared(program: &Program, shared: &mut LinkShared) -> Result<(), Error> {
    let Some(owned) = program.link_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}
