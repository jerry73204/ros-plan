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
    scope::{GroupScope, PlanScope, PlanScopeShared, ScopeMutExt, ScopeShared},
    utils::{find_plan_file_from_pkg, read_yaml_file, shared_table::SharedTable},
};
use indexmap::IndexMap;
use ros_plan_format::{
    argument::ArgEntry,
    expr::{BoolExpr, ValueOrExpr},
    key::{Key, KeyOwned, RelativeKeyOwned},
    link::{LinkCfg, PubSubLinkCfg, ServiceLinkCfg},
    node::NodeCfg,
    node_socket::NodeSocketCfg,
    parameter::ParamName,
    plan::Plan,
    plan_socket::PlanSocketCfg,
    subplan::{GroupCfg, IncludeCfg},
};
use std::{
    collections::{BTreeMap, HashSet, VecDeque},
    path::{Path, PathBuf},
};

#[derive(Debug, Default)]
pub struct PlanVisitor {
    queue: VecDeque<Job>,
}

impl PlanVisitor {
    pub fn traverse(&mut self, path: &Path) -> Result<Program, Error> {
        let mut context = Program {
            node_tab: SharedTable::default(),
            pubsub_link_tab: SharedTable::default(),
            service_link_tab: SharedTable::default(),
            node_pub_tab: SharedTable::default(),
            node_sub_tab: SharedTable::default(),
            node_srv_tab: SharedTable::default(),
            node_cli_tab: SharedTable::default(),
            plan_pub_tab: SharedTable::default(),
            plan_sub_tab: SharedTable::default(),
            plan_srv_tab: SharedTable::default(),
            plan_cli_tab: SharedTable::default(),
            include_tab: SharedTable::default(),
            group_tab: SharedTable::default(),
        };

        self.insert_root_plan(&mut context, path, IndexMap::new())?;

        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::InsertGroup(job) => {
                    self.insert_group(&mut context, job)?;
                }
                Job::InsertPlanFile(job) => {
                    self.insert_plan(&mut context, job)?;
                }
            }
        }

        Ok(context)
    }

    fn insert_root_plan(
        &mut self,
        context: &mut Program,
        plan_path: &Path,
        assign_args: IndexMap<ParamName, ValueOrExpr>,
    ) -> Result<(), Error> {
        let scope_key = Key::root();

        // Read plan file
        let plan: Plan = read_yaml_file(plan_path)?;

        // Check arg assignment
        check_arg_assignment(&plan.arg, &assign_args)?;

        // Create the context for the inserted plan
        let (plan_ctx, subplan) = to_plan_scope(
            context,
            plan_path.to_path_buf(),
            scope_key,
            plan,
            None,
            assign_args,
        )?;

        // Insert the plan context to the root node.
        let root_shared = context.include_tab.insert(plan_ctx);

        // Schedule subplan insertion jobs
        self.schedule_scope_insertion(
            root_shared.clone(),
            root_shared.clone().into(),
            Key::root(),
            subplan,
        )?;

        Ok(())
    }

    fn insert_plan(&mut self, program: &mut Program, job: InsertPlanFileJob) -> Result<(), Error> {
        let InsertPlanFileJob {
            // plan_parent: _,
            current,
            current_prefix,
            child_suffix,
            child_plan_path,
            assign_args,
            when,
        } = job;

        let Ok(child_prefix) = &current_prefix / &child_suffix else {
            todo!();
        };

        // Read plan file
        let plan: Plan = read_yaml_file(&child_plan_path)?;

        // Check arg assignment
        check_arg_assignment(&plan.arg, &assign_args)?;

        // Create the context for the inserted plan
        let (plan_ctx, subplan) = to_plan_scope(
            program,
            child_plan_path.clone(),
            &child_prefix,
            plan,
            when,
            assign_args,
        )?;
        let plan_shared = program.include_tab.insert(plan_ctx);

        // Create the child node for the plan
        current.with_write(|mut guard| -> Result<_, Error> {
            guard
                .scope_entry(child_suffix)?
                .insert_include(plan_shared.clone());
            Ok(())
        })?;

        // Schedule subplan insertion jobs
        self.schedule_scope_insertion(
            plan_shared.clone(),
            plan_shared.clone().into(),
            &child_prefix,
            subplan,
        )?;

        Ok(())
    }

    fn insert_group(&mut self, context: &mut Program, job: InsertGroupJob) -> Result<(), Error> {
        let InsertGroupJob {
            plan_parent,
            current,
            current_prefix,
            child_suffix,
            child_group,
        } = job;

        let Ok(child_prefix) = &current_prefix / &child_suffix else {
            unreachable!();
        };

        // Create the context
        let (group_scope, subplan) = to_group_scope(context, &child_prefix, child_group)?;
        let group_shared = context.group_tab.insert(group_scope);

        // Schedule subplan insertion jobs

        {
            current.with_write(|mut guard| -> Result<_, Error> {
                guard
                    .scope_entry(child_suffix)?
                    .insert_group(group_shared.clone());
                Ok(())
            })?;
        }

        self.schedule_scope_insertion(
            plan_parent.clone(),
            group_shared.clone().into(),
            &child_prefix,
            subplan,
        )?;

        Ok(())
    }

    fn schedule_scope_insertion(
        &mut self,
        plan_parent: PlanScopeShared,
        current: ScopeShared,
        current_prefix: &Key,
        subscope: SubplanTable,
    ) -> Result<(), Error> {
        let SubplanTable { include, group } = subscope;

        for (subplan_suffix, entry) in include {
            self.schedule_plan_insertion(
                plan_parent.clone(),
                current.clone(),
                current_prefix,
                &subplan_suffix,
                entry,
            )?;
        }

        for (subplan_suffix, entry) in group {
            self.schedule_group_insertion(
                plan_parent.clone(),
                current.clone(),
                current_prefix,
                &subplan_suffix,
                entry,
            )?;
        }

        Ok(())
    }

    fn schedule_plan_insertion(
        &mut self,
        plan_parent: PlanScopeShared,
        current: ScopeShared,
        current_prefix: &Key,
        child_suffix: &Key,
        subplan: IncludeCfg,
    ) -> Result<(), Error> {
        let subplan_suffix = child_suffix.to_owned();

        // Resolve the path of the plan file
        let subplan_path = match (subplan.path, subplan.pkg, subplan.file) {
            (Some(path), None, None) => {
                let subplan_path = path;
                let subplan_path = if subplan_path.is_relative() {
                    plan_parent.with_read(|plan_ctx| {
                        let Some(parent_dir) = plan_ctx.path.parent() else {
                            unreachable!("the plan file must have a parent directory");
                        };
                        parent_dir.join(subplan_path)
                    })
                } else {
                    subplan_path.to_owned()
                };
                subplan_path
            }
            (None, Some(pkg), Some(file)) => find_plan_file_from_pkg(&pkg, &file)?,
            _ => return Err(Error::InvalidIncludePath),
        };

        self.queue.push_back(
            InsertPlanFileJob {
                // plan_parent,
                current,
                child_plan_path: subplan_path,
                current_prefix: current_prefix.to_owned(),
                child_suffix: subplan_suffix,
                assign_args: subplan.arg,
                when: subplan.when,
            }
            .into(),
        );

        Ok(())
    }

    fn schedule_group_insertion(
        &mut self,
        plan_parent: PlanScopeShared,
        current: ScopeShared,
        current_prefix: &Key,
        child_suffix: &Key,
        group: GroupCfg,
    ) -> Result<(), Error> {
        let subplan_suffix = child_suffix.to_owned();

        self.queue.push_back(
            InsertGroupJob {
                plan_parent,
                current,
                child_group: group,
                current_prefix: current_prefix.to_owned(),
                child_suffix: subplan_suffix,
            }
            .into(),
        );
        Ok(())
    }
}

#[derive(Debug)]
enum Job {
    InsertGroup(InsertGroupJob),
    InsertPlanFile(InsertPlanFileJob),
}

impl From<InsertPlanFileJob> for Job {
    fn from(v: InsertPlanFileJob) -> Self {
        Self::InsertPlanFile(v)
    }
}

impl From<InsertGroupJob> for Job {
    fn from(v: InsertGroupJob) -> Self {
        Self::InsertGroup(v)
    }
}

#[derive(Debug)]
struct InsertGroupJob {
    plan_parent: PlanScopeShared,
    current: ScopeShared,
    current_prefix: KeyOwned,
    child_suffix: KeyOwned,
    child_group: GroupCfg,
}

#[derive(Debug)]
struct InsertPlanFileJob {
    // plan_parent: TrieRef,
    current: ScopeShared,
    current_prefix: KeyOwned,
    child_suffix: KeyOwned,
    child_plan_path: PathBuf,
    assign_args: IndexMap<ParamName, ValueOrExpr>,
    when: Option<BoolExpr>,
}

#[derive(Debug)]
struct SubplanTable {
    pub include: IndexMap<RelativeKeyOwned, IncludeCfg>,
    pub group: IndexMap<RelativeKeyOwned, GroupCfg>,
}

fn to_plan_scope(
    program: &mut Program,
    path: PathBuf,
    scope_key: &Key,
    plan_cfg: Plan,
    when: Option<BoolExpr>,
    assign_arg: IndexMap<ParamName, ValueOrExpr>,
) -> Result<(PlanScope, SubplanTable), Error> {
    // Check if there is an argument assignment that does not exist.
    for name in assign_arg.keys() {
        if !plan_cfg.arg.contains_key(name) {
            return Err(Error::ArgumentNotFound { name: name.clone() });
        }
    }

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
            let assign = assign_arg.get(&name).cloned();
            let arg_ctx = ArgCtx::new(arg_cfg, assign);
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
        when: when.map(BoolStore::new),
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

    let subplan = SubplanTable {
        include: plan_cfg.include,
        group: plan_cfg.group,
    };

    Ok((plan_ctx, subplan))
}

fn to_group_scope(
    program: &mut Program,
    scope_key: &Key,
    group: GroupCfg,
) -> Result<(GroupScope, SubplanTable), Error> {
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

    let subplan = SubplanTable {
        include: group.include,
        group: group.group,
    };

    Ok((group_ctx, subplan))
}

fn to_socket_context(key: KeyOwned, socket_ctx: PlanSocketCfg) -> PlanSocketCtx {
    match socket_ctx {
        PlanSocketCfg::Pub(config) => PlanPubCtx {
            config,
            src: None,
            key,
        }
        .into(),
        PlanSocketCfg::Sub(config) => PlanSubCtx {
            config,
            dst: None,
            key,
        }
        .into(),
        PlanSocketCfg::Srv(config) => PlanSrvCtx {
            config,
            listen: None,
            key,
        }
        .into(),
        PlanSocketCfg::Cli(config) => PlanCliCtx {
            config,
            connect: None,
            key,
        }
        .into(),
    }
}

fn to_link_context(key: KeyOwned, link_cfg: LinkCfg) -> LinkCtx {
    match link_cfg {
        LinkCfg::PubSub(link_cfg) => {
            let PubSubLinkCfg {
                ty,
                qos,
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
                src_key,
                dst_key,
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

fn check_arg_assignment(
    spec: &IndexMap<ParamName, ArgEntry>,
    assign: &IndexMap<ParamName, ValueOrExpr>,
) -> Result<(), Error> {
    let spec_names: HashSet<_> = spec.keys().collect();
    let assigned_names: HashSet<_> = assign.keys().collect();

    // Check if there are unassigned required args
    for name in &spec_names - &assigned_names {
        if spec[name].default.is_none() {
            return Err(Error::RequiredArgumentNotAssigned {
                name: name.to_owned(),
            });
        }
    }

    // Check if there are assigned args not found in spec.
    if let Some(name) = (&assigned_names - &spec_names).into_iter().next() {
        return Err(Error::ArgumentNotFound { name: name.clone() });
    }

    // Check type compatibility for assigned args
    for name in &spec_names & &assigned_names {
        match &assign[name] {
            ValueOrExpr::Value(value) => {
                let assigned_ty = value.ty();
                let expect_ty = spec[name].ty;

                if assigned_ty != expect_ty {
                    return Err(Error::TypeMismatch {
                        expect: expect_ty,
                        found: assigned_ty,
                    });
                }
            }
            ValueOrExpr::Expr { .. } => {
                // Cannot check without evaluation. Skip it.
            }
        }
    }

    Ok(())
}

fn to_node_context(program: &mut Program, node_key: KeyOwned, node_cfg: NodeCfg) -> NodeCtx {
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

fn to_node_socket_context(key: KeyOwned, cfg: NodeSocketCfg) -> NodeSocketCtx {
    match cfg {
        NodeSocketCfg::Pub(cfg) => NodePubCtx {
            key,
            config: cfg,
            link_to: None,
        }
        .into(),
        NodeSocketCfg::Sub(cfg) => NodeSubCtx {
            key,
            config: cfg,
            link_to: None,
        }
        .into(),
        NodeSocketCfg::Srv(cfg) => NodeSrvCtx {
            key,
            config: cfg,
            link_to: None,
        }
        .into(),
        NodeSocketCfg::Cli(cfg) => NodeCliCtx {
            key,
            config: cfg,
            link_to: None,
        }
        .into(),
    }
}
