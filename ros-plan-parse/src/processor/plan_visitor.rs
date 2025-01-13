use crate::{
    context::{
        arg::ArgContext,
        expr::ExprContext,
        link::{LinkContext, PubsubLinkContext, ServiceLinkContext},
        node::{NodeContext, ProcessContext, RosNodeContext},
        socket::{
            PubSocketContext, QuerySocketContext, ServerSocketContext, SocketContext,
            SubSocketContext,
        },
    },
    error::Error,
    resource::Resource,
    scope::{GroupScope, PlanFileScope, Scope, ScopeTree, ScopeTreeRef},
    utils::{find_plan_file_from_pkg, read_toml_file, shared_table::SharedTable},
};
use indexmap::IndexMap;
use ros_plan_format::{
    argument::{ArgCfg, ArgEntry},
    expr::ValueOrExpr,
    key::{Key, KeyOwned},
    link::LinkCfg,
    node::NodeCfg,
    parameter::ParamName,
    plan::Plan,
    socket::SocketCfg,
    subplan::{GroupCfg, SubplanCfg, SubplanTable},
};
use std::{
    collections::{HashSet, VecDeque},
    path::{Path, PathBuf},
};

#[derive(Debug, Default)]
pub struct PlanVisitor {
    queue: VecDeque<Job>,
}

impl PlanVisitor {
    pub fn traverse(&mut self, path: &Path) -> Result<Resource, Error> {
        let mut context = Resource {
            root: None,
            node_tab: SharedTable::default(),
            link_tab: SharedTable::default(),
            socket_tab: SharedTable::default(),
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
        context: &mut Resource,
        plan_path: &Path,
        assign_args: IndexMap<ParamName, ValueOrExpr>,
    ) -> Result<(), Error> {
        // Read plan file
        let plan: Plan = read_toml_file(plan_path)?;

        // Check arg assignment
        check_arg_assignment(&plan.arg, &assign_args)?;

        // Create the context for the inserted plan
        let (plan_ctx, subplan_tab) =
            to_plan_context(context, plan_path.to_path_buf(), plan, None, assign_args)?;

        // Insert the plan context to the root node.
        let root = ScopeTreeRef::new(ScopeTree::new(plan_ctx.into()));
        context.root = Some(root.clone());

        // Schedule subplan insertion jobs
        for (subplan_suffix, subplan) in subplan_tab.0 {
            self.schedule_subplan_insertion(
                root.clone(),
                root.clone(),
                Key::root(),
                &subplan_suffix,
                subplan,
            )?;
        }

        Ok(())
    }

    fn insert_plan(&mut self, context: &mut Resource, job: InsertPlanFileJob) -> Result<(), Error> {
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
        let plan: Plan = read_toml_file(&child_plan_path)?;

        // Check arg assignment
        check_arg_assignment(&plan.arg, &assign_args)?;

        // Create the context for the inserted plan
        let (plan_ctx, subplan_tab) =
            to_plan_context(context, child_plan_path.clone(), plan, when, assign_args)?;

        // Create the child node for the plan
        let plan_child = current.insert(child_suffix, plan_ctx.into())?;

        // Insert the plan to the global table
        // {
        //     let prev = context.plan_map.insert(child_plan_path, plan_child.clone());
        //     if prev.is_some() {}
        // }

        // Schedule subplan insertion jobs
        for (subplan_suffix, subplan) in subplan_tab.0 {
            self.schedule_subplan_insertion(
                plan_child.clone(),
                plan_child.clone(),
                &child_prefix,
                &subplan_suffix,
                subplan,
            )?;
        }

        Ok(())
    }

    fn insert_group(&mut self, context: &mut Resource, job: InsertGroupJob) -> Result<(), Error> {
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
        let (group_scope, subplan_tab) = to_group_scope(context, child_group);

        // Schedule subplan insertion jobs
        {
            let group_child = current.insert(child_suffix, group_scope.into())?;

            for (subplan_suffix, subplan) in subplan_tab.0 {
                self.schedule_subplan_insertion(
                    plan_parent.clone(),
                    group_child.clone(),
                    &child_prefix,
                    &subplan_suffix,
                    subplan,
                )?;
            }
        }
        Ok(())
    }

    fn schedule_subplan_insertion(
        &mut self,
        plan_parent: ScopeTreeRef,
        current: ScopeTreeRef,
        current_prefix: &Key,
        child_suffix: &Key,
        child_subplan: SubplanCfg,
    ) -> Result<(), Error> {
        let subplan_suffix = child_suffix.to_owned();

        match child_subplan {
            SubplanCfg::File(subplan) => {
                // Resolve the path of the plan file
                let subplan_path = {
                    let subplan_path = &subplan.path;
                    let subplan_path = if subplan_path.is_relative() {
                        let guard = plan_parent.read();
                        let Scope::PlanFile(plan_ctx) = &guard.value else {
                            unreachable!("the plan context must be initialized");
                        };
                        let Some(parent_dir) = plan_ctx.path.parent() else {
                            unreachable!("the plan file must have a parent directory");
                        };
                        parent_dir.join(subplan_path)
                    } else {
                        subplan_path.to_owned()
                    };
                    subplan_path
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
            }
            SubplanCfg::Pkg(subplan) => {
                let path = find_plan_file_from_pkg(&subplan.pkg, &subplan.file)?;

                self.queue.push_back(
                    InsertPlanFileJob {
                        // plan_parent,
                        current,
                        child_plan_path: path,
                        current_prefix: current_prefix.to_owned(),
                        child_suffix: subplan_suffix,
                        assign_args: subplan.arg,
                        when: subplan.when,
                    }
                    .into(),
                );
            }
            SubplanCfg::Group(group) => {
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
            }
        }
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
    plan_parent: ScopeTreeRef,
    current: ScopeTreeRef,
    current_prefix: KeyOwned,
    child_suffix: KeyOwned,
    child_group: GroupCfg,
}

#[derive(Debug)]
struct InsertPlanFileJob {
    // plan_parent: TrieRef,
    current: ScopeTreeRef,
    current_prefix: KeyOwned,
    child_suffix: KeyOwned,
    child_plan_path: PathBuf,
    assign_args: IndexMap<ParamName, ValueOrExpr>,
    when: Option<ValueOrExpr>,
}

fn to_plan_context(
    resource: &mut Resource,
    path: PathBuf,
    plan_cfg: Plan,
    when: Option<ValueOrExpr>,
    assign_arg: IndexMap<ParamName, ValueOrExpr>,
) -> Result<(PlanFileScope, SubplanTable), Error> {
    // Check if there is an argument assignment that does not exist.
    for name in assign_arg.keys() {
        if !plan_cfg.arg.contains_key(name) {
            return Err(Error::ArgumentNotFound { name: name.clone() });
        }
    }

    let node_map: IndexMap<_, _> = plan_cfg
        .node
        .0
        .into_iter()
        .map(|(ident, cfg)| {
            let ctx = to_node_context(cfg);
            let shared = resource.node_tab.insert(ctx);
            (ident, shared)
        })
        .collect();

    let socket_map: IndexMap<_, _> = plan_cfg
        .socket
        .0
        .into_iter()
        .map(|(ident, cfg)| {
            let ctx = to_socket_context(cfg);
            let shared = resource.socket_tab.insert(ctx);
            (ident, shared)
        })
        .collect();

    let link_map: IndexMap<_, _> = plan_cfg
        .link
        .0
        .into_iter()
        .map(|(ident, cfg)| {
            let ctx = to_link_context(cfg);
            let shared = resource.link_tab.insert(ctx);
            (ident, shared)
        })
        .collect();

    let arg_map: IndexMap<_, _> = plan_cfg
        .arg
        .into_iter()
        .map(|(name, arg_cfg)| {
            let assign = assign_arg.get(&name).cloned();
            let arg_ctx = ArgContext::new(arg_cfg, assign);
            (name, arg_ctx)
        })
        .collect();

    let var_map: IndexMap<_, _> = plan_cfg
        .var
        .into_iter()
        .map(|(name, var_entry)| (name, ExprContext::new(var_entry)))
        .collect();

    let plan_ctx = PlanFileScope {
        path,
        arg_map,
        var_map,
        socket_map,
        node_map,
        link_map,
        when: when.map(ExprContext::new),
    };

    Ok((plan_ctx, plan_cfg.subplan))
}

fn to_group_scope(resource: &mut Resource, group: GroupCfg) -> (GroupScope, SubplanTable) {
    let local_node_map: IndexMap<_, _> = group
        .node
        .0
        .into_iter()
        .map(|(ident, cfg)| {
            let ctx = to_node_context(cfg);
            let shared = resource.node_tab.insert(ctx);
            (ident, shared)
        })
        .collect();
    let local_link_map: IndexMap<_, _> = group
        .link
        .0
        .into_iter()
        .map(|(ident, cfg)| {
            let ctx = to_link_context(cfg);
            let shared = resource.link_tab.insert(ctx);
            (ident, shared)
        })
        .collect();

    let ctx = GroupScope {
        when: group.when.map(ExprContext::new),
        node_map: local_node_map,
        link_map: local_link_map,
    };
    (ctx, group.subplan)
}

fn to_socket_context(socket_ctx: SocketCfg) -> SocketContext {
    match socket_ctx {
        SocketCfg::Pub(config) => PubSocketContext { config, src: None }.into(),
        SocketCfg::Sub(config) => SubSocketContext { config, dst: None }.into(),
        SocketCfg::Srv(config) => ServerSocketContext {
            config,
            listen: None,
        }
        .into(),
        SocketCfg::Qry(config) => QuerySocketContext {
            config,
            connect: None,
        }
        .into(),
    }
}

fn to_link_context(link: LinkCfg) -> LinkContext {
    match link {
        LinkCfg::Pubsub(link) => PubsubLinkContext {
            config: link,
            src: None,
            dst: None,
        }
        .into(),
        LinkCfg::Service(link) => ServiceLinkContext {
            config: link,
            listen: None,
            connect: None,
        }
        .into(),
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
        if let ArgCfg::Required { .. } = &spec[name].slot {
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

                let expect_ty = match spec[name].slot {
                    ArgCfg::Required { ty } => ty,
                    ArgCfg::Optional { ref default } => default.ty(),
                };

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

fn to_node_context(node_cfg: NodeCfg) -> NodeContext {
    match node_cfg {
        NodeCfg::Ros(node_cfg) => RosNodeContext {
            param: {
                node_cfg
                    .param
                    .clone()
                    .into_iter()
                    .map(|(name, eval)| (name, ExprContext::new(eval)))
                    .collect()
            },
            config: node_cfg,
        }
        .into(),
        NodeCfg::Proc(node_cfg) => ProcessContext { config: node_cfg }.into(),
    }
}
