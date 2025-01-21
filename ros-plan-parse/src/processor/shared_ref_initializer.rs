use crate::{
    context::{
        link::{LinkContext, PubsubLinkContext, ServiceLinkContext},
        socket::{
            PubSocketContext, QuerySocketContext, ServerSocketContext, SocketContext,
            SubSocketContext,
        },
        uri::NodeTopicUri,
    },
    error::Error,
    scope::{GroupScope, LinkShared, NodeShared, PlanFileScope, Scope, ScopeTreeRef, SocketShared},
    Resource,
};
use indexmap::IndexMap;
use ros_plan_format::{link::LinkIdent, node::NodeIdent, socket::SocketIdent};
use std::collections::VecDeque;

pub struct SharedRefInitializer {
    queue: VecDeque<ScopeTreeRef>,
}

impl SharedRefInitializer {
    pub fn initialize(&mut self, resource: &Resource) -> Result<(), Error> {
        {
            let Resource {
                link_tab,
                socket_tab,
                ..
            } = resource;

            for (_id, link) in link_tab.read_inner().iter() {
                update_link_context(resource, &mut link.write())?;
            }

            for (_id, socket) in socket_tab.read_inner().iter() {
                update_socket_context(resource, &mut socket.write())?;
            }
        }

        {
            let root = resource.root.clone().unwrap();
            self.queue.push_back(root);

            while let Some(curr) = self.queue.pop_front() {
                self.visit_scope(resource, curr)?;
            }
        }
        Ok(())
    }

    fn visit_scope(&mut self, resource: &Resource, current: ScopeTreeRef) -> Result<(), Error> {
        let mut guard = current.write();
        match &mut guard.value {
            Scope::PlanFile(scope) => update_plan_file_scope(resource, scope)?,
            Scope::Group(scope) => update_group_scope(resource, scope)?,
        }
        Ok(())
    }
}

impl Default for SharedRefInitializer {
    fn default() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }
}

fn update_plan_file_scope(resource: &Resource, scope: &mut PlanFileScope) -> Result<(), Error> {
    let PlanFileScope {
        socket_map,
        node_map,
        link_map,
        ..
    } = scope;
    update_node_map(resource, node_map)?;
    update_link_map(resource, link_map)?;
    update_socket_map(resource, socket_map)?;
    Ok(())
}

fn update_group_scope(resource: &Resource, scope: &mut GroupScope) -> Result<(), Error> {
    let GroupScope {
        node_map, link_map, ..
    } = scope;
    update_node_map(resource, node_map)?;
    update_link_map(resource, link_map)?;
    Ok(())
}

fn update_node_map(
    resource: &Resource,
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
    resource: &Resource,
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
    resource: &Resource,
    socket_map: &mut IndexMap<SocketIdent, SocketShared>,
) -> Result<(), Error> {
    for shared in socket_map.values_mut() {
        let Some(owned) = resource.socket_tab.get(shared.id()) else {
            todo!()
        };
        *shared = owned.downgrade();
    }
    Ok(())
}

fn update_link_context(resource: &Resource, link: &mut LinkContext) -> Result<(), Error> {
    match link {
        LinkContext::PubSub(link) => update_pubsub_link_context(resource, link)?,
        LinkContext::Service(link) => update_service_link_context(resource, link)?,
    }
    Ok(())
}

fn update_pubsub_link_context(
    resource: &Resource,
    link: &mut PubsubLinkContext,
) -> Result<(), Error> {
    let PubsubLinkContext { src, dst, .. } = link;

    for uri in src.iter_mut().flatten() {
        update_topic_uri(resource, uri)?;
    }

    for uri in dst.iter_mut().flatten() {
        update_topic_uri(resource, uri)?;
    }

    Ok(())
}

fn update_service_link_context(
    resource: &Resource,
    link: &mut ServiceLinkContext,
) -> Result<(), Error> {
    let ServiceLinkContext {
        listen, connect, ..
    } = link;

    if let Some(uri) = listen {
        update_topic_uri(resource, uri)?;
    }

    for uri in connect.iter_mut().flatten() {
        update_topic_uri(resource, uri)?;
    }
    Ok(())
}

fn update_socket_context(resource: &Resource, socket: &mut SocketContext) -> Result<(), Error> {
    match socket {
        SocketContext::Pub(socket) => update_pub_socket_context(resource, socket)?,
        SocketContext::Sub(socket) => update_sub_socket_context(resource, socket)?,
        SocketContext::Srv(socket) => update_server_socket_context(resource, socket)?,
        SocketContext::Qry(socket) => update_query_socket_context(resource, socket)?,
    }
    Ok(())
}

fn update_pub_socket_context(
    resource: &Resource,
    socket: &mut PubSocketContext,
) -> Result<(), Error> {
    let PubSocketContext { src, .. } = socket;

    for uri in src.iter_mut().flatten() {
        update_topic_uri(resource, uri)?;
    }

    Ok(())
}

fn update_sub_socket_context(
    resource: &Resource,
    socket: &mut SubSocketContext,
) -> Result<(), Error> {
    let SubSocketContext { dst, .. } = socket;

    for uri in dst.iter_mut().flatten() {
        update_topic_uri(resource, uri)?;
    }

    Ok(())
}

fn update_server_socket_context(
    resource: &Resource,
    socket: &mut ServerSocketContext,
) -> Result<(), Error> {
    let ServerSocketContext { listen, .. } = socket;

    if let Some(uri) = listen {
        update_topic_uri(resource, uri)?;
    }

    Ok(())
}

fn update_query_socket_context(
    resource: &Resource,
    socket: &mut QuerySocketContext,
) -> Result<(), Error> {
    let QuerySocketContext { connect, .. } = socket;

    for uri in connect.iter_mut().flatten() {
        update_topic_uri(resource, uri)?;
    }

    Ok(())
}

fn update_topic_uri(resource: &Resource, uri: &mut NodeTopicUri) -> Result<(), Error> {
    let NodeTopicUri { node: shared, .. } = uri;
    let Some(owned) = resource.node_tab.get(shared.id()) else {
        todo!()
    };
    *shared = owned.downgrade();
    Ok(())
}
