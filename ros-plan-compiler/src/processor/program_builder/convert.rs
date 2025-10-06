use crate::{
    context::{
        arg::ArgCtx,
        link::{LinkCtx, PubSubLinkCtx, ServiceLinkCtx},
        node::NodeCtx,
        node_socket::{NodeCliCtx, NodePubCtx, NodeSocketCtx, NodeSrvCtx, NodeSubCtx},
        plan_socket::{PlanCliCtx, PlanPubCtx, PlanSocketCtx, PlanSrvCtx, PlanSubCtx},
    },
    error::Error,
    eval::{BoolStore, KeyStore, TextStore, ValueStore},
    program::Program,
    scope::{GroupScope, PlanScope, ScopeMutExt},
};
use indexmap::IndexMap;
use ros_plan_format::{
    key::{Key, KeyOwned, RelativeKeyOwned},
    link::{LinkCfg, PubSubLinkCfg, ServiceLinkCfg},
    node::NodeCfg,
    node_socket::NodeSocketCfg,
    plan::Plan,
    plan_socket::PlanSocketCfg,
    subplan::{GroupCfg, IncludeCfg},
};
use std::{collections::BTreeMap, path::PathBuf};

#[derive(Debug)]
pub struct SubScopeList {
    pub include: IndexMap<RelativeKeyOwned, IncludeCfg>,
    pub group: IndexMap<RelativeKeyOwned, GroupCfg>,
}

pub fn to_plan_scope(
    program: &mut Program,
    path: PathBuf,
    scope_key: &Key,
    plan_cfg: Plan,
) -> Result<(PlanScope, SubScopeList), Error> {
    let mut pub_map = IndexMap::new();
    let mut sub_map = IndexMap::new();
    let mut srv_map = IndexMap::new();
    let mut cli_map = IndexMap::new();

    for (ident, cfg) in plan_cfg.socket.into_iter() {
        let socket_key = scope_key / &ident;
        let ctx = to_socket_context(socket_key, cfg);

        match ctx {
            PlanSocketCtx::Pub(ctx) => {
                let shared = program.plan_pub_tab.insert(ctx);
                pub_map.insert(ident, shared);
            }
            PlanSocketCtx::Sub(ctx) => {
                let shared = program.plan_sub_tab.insert(ctx);
                sub_map.insert(ident, shared);
            }
            PlanSocketCtx::Srv(ctx) => {
                let shared = program.plan_srv_tab.insert(ctx);
                srv_map.insert(ident, shared);
            }
            PlanSocketCtx::Cli(ctx) => {
                let shared = program.plan_cli_tab.insert(ctx);
                cli_map.insert(ident, shared);
            }
        }
    }

    let arg_map: IndexMap<_, _> = plan_cfg
        .arg
        .into_iter()
        .map(|(name, arg_cfg)| {
            let arg_ctx = ArgCtx::new(arg_cfg);
            (name, arg_ctx)
        })
        .collect();

    let var_map: IndexMap<_, _> = plan_cfg
        .var
        .into_iter()
        .map(|(name, var_entry)| (name, ValueStore::new(var_entry)))
        .collect();

    let mut plan_ctx = PlanScope {
        path,
        arg: arg_map,
        var: var_map,
        pub_: pub_map,
        sub: sub_map,
        srv: srv_map,
        cli: cli_map,
        node: IndexMap::new(),
        pubsub_link: IndexMap::new(),
        service_link: IndexMap::new(),
        include: IndexMap::new(),
        group: IndexMap::new(),
        key: BTreeMap::new(),
    };

    for (ident, cfg) in plan_cfg.node.into_iter() {
        let node_key = scope_key / &ident;
        let ctx = to_node_context(program, node_key, cfg);
        let shared = program.node_tab.insert(ctx);
        plan_ctx.entity_entry(ident)?.insert_node(shared);
    }

    for (ident, cfg) in plan_cfg.link.into_iter() {
        let link_key = scope_key / &ident;
        let ctx = to_link_context(link_key, cfg);
        match ctx {
            LinkCtx::PubSub(ctx) => {
                let shared = program.pubsub_link_tab.insert(ctx);
                plan_ctx.entity_entry(ident)?.insert_pubsub_link(shared);
            }
            LinkCtx::Service(ctx) => {
                let shared = program.service_link_tab.insert(ctx);
                plan_ctx.entity_entry(ident)?.insert_service_link(shared);
            }
        }
    }

    let subscope = SubScopeList {
        include: plan_cfg.include,
        group: plan_cfg.group,
    };

    Ok((plan_ctx, subscope))
}

pub fn to_group_scope(
    program: &mut Program,
    scope_key: &Key,
    group: GroupCfg,
) -> Result<(GroupScope, SubScopeList), Error> {
    let mut group_ctx = GroupScope {
        when: group.when.map(BoolStore::new),
        node: IndexMap::new(),
        pubsub_link: IndexMap::new(),
        service_link: IndexMap::new(),
        include: IndexMap::new(),
        group: IndexMap::new(),
        key: BTreeMap::new(),
    };

    for (ident, cfg) in group.node.into_iter() {
        let node_key = scope_key / &ident;
        let ctx = to_node_context(program, node_key, cfg);
        let shared = program.node_tab.insert(ctx);
        group_ctx.entity_entry(ident)?.insert_node(shared);
    }

    for (ident, cfg) in group.link.into_iter() {
        let link_key = scope_key / &ident;
        let ctx = to_link_context(link_key, cfg);
        match ctx {
            LinkCtx::PubSub(ctx) => {
                let shared = program.pubsub_link_tab.insert(ctx);
                group_ctx.entity_entry(ident)?.insert_pubsub_link(shared);
            }
            LinkCtx::Service(ctx) => {
                let shared = program.service_link_tab.insert(ctx);
                group_ctx.entity_entry(ident)?.insert_service_link(shared);
            }
        }
    }

    let subscope = SubScopeList {
        include: group.include,
        group: group.group,
    };

    Ok((group_ctx, subscope))
}

pub fn to_socket_context(key: KeyOwned, socket_ctx: PlanSocketCfg) -> PlanSocketCtx {
    match socket_ctx {
        PlanSocketCfg::Pub(config) => {
            let src_key: Vec<_> = config.src.into_iter().map(KeyStore::new).collect();
            PlanPubCtx {
                key,
                ty: config.ty,
                qos: config.qos,
                topic: config.topic.map(TextStore::new),
                src_key,
                src_socket: None,
            }
            .into()
        }
        PlanSocketCfg::Sub(config) => {
            let dst_key: Vec<_> = config.dst.into_iter().map(KeyStore::new).collect();
            PlanSubCtx {
                key,
                ty: config.ty,
                qos: config.qos,
                topic: config.topic.map(TextStore::new),
                dst_key,
                dst_socket: None,
            }
            .into()
        }
        PlanSocketCfg::Srv(config) => {
            let listen_key = KeyStore::new(config.listen);
            PlanSrvCtx {
                key,
                ty: config.ty,
                listen_key,
                listen_socket: None,
            }
            .into()
        }
        PlanSocketCfg::Cli(config) => {
            let connect_key: Vec<_> = config.connect.into_iter().map(KeyStore::new).collect();

            PlanCliCtx {
                key,
                ty: config.ty,
                connect_key,
                connect_socket: None,
            }
            .into()
        }
    }
}

pub fn to_link_context(key: KeyOwned, link_cfg: LinkCfg) -> LinkCtx {
    match link_cfg {
        LinkCfg::PubSub(link_cfg) => {
            let PubSubLinkCfg {
                ty,
                qos,
                topic,
                src,
                dst,
                when,
            } = link_cfg;
            let src_key: Vec<_> = src.into_iter().map(KeyStore::new).collect();
            let dst_key: Vec<_> = dst.into_iter().map(KeyStore::new).collect();
            PubSubLinkCtx {
                key,
                src_socket: None,
                dst_socket: None,
                ty,
                qos,
                when: when.map(BoolStore::new),
                topic: topic.map(TextStore::new),
                src_key,
                dst_key,
                derived_topic: None,
            }
            .into()
        }
        LinkCfg::Service(link_cfg) => {
            let ServiceLinkCfg {
                ty,
                listen,
                connect,
                when,
            } = link_cfg;
            let listen_key = KeyStore::new(listen);
            let connect_key: Vec<_> = connect.into_iter().map(KeyStore::new).collect();

            ServiceLinkCtx {
                key,
                listen_socket: None,
                connect_socket: None,
                ty,
                when: when.map(BoolStore::new),
                listen_key,
                connect_key,
            }
            .into()
        }
    }
}

pub fn to_node_context(program: &mut Program, node_key: KeyOwned, node_cfg: NodeCfg) -> NodeCtx {
    let param = node_cfg
        .param
        .iter()
        .map(|(name, eval)| (name.clone(), ValueStore::new(eval.clone())))
        .collect();

    let mut pub_map = IndexMap::new();
    let mut sub_map = IndexMap::new();
    let mut srv_map = IndexMap::new();
    let mut cli_map = IndexMap::new();

    for (name, socket_cfg) in node_cfg.socket.into_iter() {
        let socket_key = &node_key / &name;
        let ctx = to_node_socket_context(socket_key, socket_cfg.clone());

        match ctx {
            NodeSocketCtx::Pub(ctx) => {
                let shared = program.node_pub_tab.insert(ctx);
                pub_map.insert(name, shared);
            }
            NodeSocketCtx::Sub(ctx) => {
                let shared = program.node_sub_tab.insert(ctx);
                sub_map.insert(name, shared);
            }
            NodeSocketCtx::Srv(ctx) => {
                let shared = program.node_srv_tab.insert(ctx);
                srv_map.insert(name, shared);
            }
            NodeSocketCtx::Cli(ctx) => {
                let shared = program.node_cli_tab.insert(ctx);
                cli_map.insert(name, shared);
            }
        }
    }

    NodeCtx {
        key: node_key,
        pkg: node_cfg.pkg.map(TextStore::new),
        exec: node_cfg.exec.map(TextStore::new),
        plugin: node_cfg.plugin.map(TextStore::new),
        param,
        pub_: pub_map,
        sub: sub_map,
        srv: srv_map,
        cli: cli_map,
    }
}

pub fn to_node_socket_context(key: KeyOwned, cfg: NodeSocketCfg) -> NodeSocketCtx {
    match cfg {
        NodeSocketCfg::Pub(cfg) => NodePubCtx {
            key,
            ty: cfg.ty,
            qos: cfg.qos,
            ros_name: cfg.ros_name.map(TextStore::new),
            remap_from: cfg.from.map(KeyStore::new),
            link_to: None,
        }
        .into(),
        NodeSocketCfg::Sub(cfg) => NodeSubCtx {
            key,
            ty: cfg.ty,
            qos: cfg.qos,
            ros_name: cfg.ros_name.map(TextStore::new),
            remap_from: cfg.from.map(KeyStore::new),
            link_to: None,
        }
        .into(),
        NodeSocketCfg::Srv(cfg) => NodeSrvCtx {
            key,
            ty: cfg.ty,
            ros_name: cfg.ros_name.map(TextStore::new),
            remap_from: cfg.from.map(KeyStore::new),
            link_to: None,
        }
        .into(),
        NodeSocketCfg::Cli(cfg) => NodeCliCtx {
            key,
            ty: cfg.ty,
            ros_name: cfg.ros_name.map(TextStore::new),
            remap_from: cfg.from.map(KeyStore::new),
            link_to: None,
        }
        .into(),
    }
}
