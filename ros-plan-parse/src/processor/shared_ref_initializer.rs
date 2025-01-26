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
    pub fn initialize(&mut self, resource: &Program) -> Result<(), Error> {
        {
            let Program {
                link_tab,
                plan_socket_tab,
                node_socket_tab,
                ..
            } = resource;

            for (_id, link) in link_tab.read_inner().iter() {
                update_link_context(resource, &mut link.write())?;
            }

            for (_id, socket) in plan_socket_tab.read_inner().iter() {
                update_plan_socket_context(resource, &mut socket.write())?;
            }

            for (_id, socket) in node_socket_tab.read_inner().iter() {
                update_node_socket_context(resource, &mut socket.write())?;
            }
        }

        {
            let root = resource.root().unwrap();
            self.queue.push_back(root.downgrade().into());

            while let Some(curr) = self.queue.pop_front() {
                self.visit_scope(resource, curr)?;
            }
        }
        Ok(())
    }

    fn visit_scope(&mut self, resource: &Program, current: ScopeShared) -> Result<(), Error> {
        // let mut guard = owned.write();
        match &current {
            ScopeShared::Include(scope) => update_plan_file_scope(resource, scope)?,
            ScopeShared::Group(scope) => update_group_scope(resource, scope)?,
        }
        Ok(())
    }
}

fn update_plan_file_scope(resource: &Program, shared: &PlanScopeShared) -> Result<(), Error> {
    let owned = shared.upgrade().unwrap();
    let mut scope = owned.write();

    let PlanScope {
        socket_map,
        node_map,
        link_map,
        ..
    } = &mut *scope;

    update_node_map(resource, node_map)?;
    update_link_map(resource, link_map)?;
    update_socket_map(resource, socket_map)?;
    Ok(())
}

fn update_group_scope(resource: &Program, shared: &GroupScopeShared) -> Result<(), Error> {
    let owned = shared.upgrade().unwrap();
    let mut scope = owned.write();

    let GroupScope {
        node_map, link_map, ..
    } = &mut *scope;

    update_node_map(resource, node_map)?;
    update_link_map(resource, link_map)?;
    Ok(())
}

fn update_node_map(
    resource: &Program,
    node_map: &mut IndexMap<NodeIdent, NodeShared>,
) -> Result<(), Error> {
    for shared in node_map.values_mut() {
        let Some(owned) = resource.node_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_link_map(
    resource: &Program,
    link_map: &mut IndexMap<LinkIdent, LinkShared>,
) -> Result<(), Error> {
    for shared in link_map.values_mut() {
        let Some(owned) = resource.link_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_socket_map(
    resource: &Program,
    socket_map: &mut IndexMap<PlanSocketIdent, PlanSocketShared>,
) -> Result<(), Error> {
    for shared in socket_map.values_mut() {
        let Some(owned) = resource.plan_socket_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_link_context(resource: &Program, link: &mut LinkCtx) -> Result<(), Error> {
    match link {
        LinkCtx::PubSub(link) => update_pubsub_link_context(resource, link)?,
        LinkCtx::Service(link) => update_service_link_context(resource, link)?,
    }
    Ok(())
}

fn update_pubsub_link_context(resource: &Program, link: &mut PubsubLinkCtx) -> Result<(), Error> {
    let PubsubLinkCtx { src, dst, .. } = link;

    for uri in src.iter_mut().flatten() {
        initialize_node_socket_shared(resource, uri)?;
    }

    for uri in dst.iter_mut().flatten() {
        initialize_node_socket_shared(resource, uri)?;
    }

    Ok(())
}

fn update_service_link_context(resource: &Program, link: &mut ServiceLinkCtx) -> Result<(), Error> {
    let ServiceLinkCtx {
        listen, connect, ..
    } = link;

    if let Some(uri) = listen {
        initialize_node_socket_shared(resource, uri)?;
    }

    for uri in connect.iter_mut().flatten() {
        initialize_node_socket_shared(resource, uri)?;
    }
    Ok(())
}

fn update_plan_socket_context(resource: &Program, socket: &mut PlanSocketCtx) -> Result<(), Error> {
    match socket {
        PlanSocketCtx::Publication(socket) => update_plan_pub_context(resource, socket)?,
        PlanSocketCtx::Subscription(socket) => update_plan_sub_context(resource, socket)?,
        PlanSocketCtx::Server(socket) => update_plan_server_context(resource, socket)?,
        PlanSocketCtx::Client(socket) => update_plan_client_context(resource, socket)?,
    }
    Ok(())
}

fn update_plan_pub_context(resource: &Program, socket: &mut PlanPubCtx) -> Result<(), Error> {
    let PlanPubCtx { src, .. } = socket;

    for uri in src.iter_mut().flatten() {
        initialize_node_socket_shared(resource, uri)?;
    }

    Ok(())
}

fn update_plan_sub_context(resource: &Program, socket: &mut PlanSubCtx) -> Result<(), Error> {
    let PlanSubCtx { dst, .. } = socket;

    for uri in dst.iter_mut().flatten() {
        initialize_node_socket_shared(resource, uri)?;
    }

    Ok(())
}

fn update_plan_server_context(resource: &Program, socket: &mut PlanSrvCtx) -> Result<(), Error> {
    let PlanSrvCtx { listen, .. } = socket;

    if let Some(uri) = listen {
        initialize_node_socket_shared(resource, uri)?;
    }

    Ok(())
}

fn update_plan_client_context(resource: &Program, socket: &mut PlanCliCtx) -> Result<(), Error> {
    let PlanCliCtx { connect, .. } = socket;

    for uri in connect.iter_mut().flatten() {
        initialize_node_socket_shared(resource, uri)?;
    }

    Ok(())
}

fn update_node_socket_context(resource: &Program, socket: &mut NodeSocketCtx) -> Result<(), Error> {
    match socket {
        NodeSocketCtx::Publication(socket) => update_node_pub_context(resource, socket)?,
        NodeSocketCtx::Subscription(socket) => update_node_sub_context(resource, socket)?,
        NodeSocketCtx::Server(socket) => update_node_server_context(resource, socket)?,
        NodeSocketCtx::Client(socket) => update_node_client_context(resource, socket)?,
    }
    Ok(())
}

fn update_node_pub_context(resource: &Program, socket: &mut NodePubCtx) -> Result<(), Error> {
    let NodePubCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_link_shared(resource, link_to)?;
    }
    Ok(())
}

fn update_node_sub_context(resource: &Program, socket: &mut NodeSubCtx) -> Result<(), Error> {
    let NodeSubCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_link_shared(resource, link_to)?;
    }
    Ok(())
}

fn update_node_server_context(resource: &Program, socket: &mut NodeSrvCtx) -> Result<(), Error> {
    let NodeSrvCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_link_shared(resource, link_to)?;
    }

    Ok(())
}

fn update_node_client_context(resource: &Program, socket: &mut NodeCliCtx) -> Result<(), Error> {
    let NodeCliCtx { link_to, .. } = socket;
    if let Some(link_to) = link_to {
        initialize_link_shared(resource, link_to)?;
    }
    Ok(())
}

fn initialize_node_socket_shared(
    resource: &Program,
    shared: &mut NodeSocketShared,
) -> Result<(), Error> {
    let Some(owned) = resource.node_socket_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}

fn initialize_link_shared(resource: &Program, shared: &mut LinkShared) -> Result<(), Error> {
    let Some(owned) = resource.link_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}
