use crate::{
    context::{
        link::{LinkArc, LinkContext, PubSubLinkContext, ServiceLinkContext},
        node::{NodeArc, NodeContext},
        socket::{
            PubSocketContext, QuerySocketContext, ServerSocketContext, SocketArc, SocketContext,
            SubSocketContext,
        },
    },
    error::Error,
    eval::eval_plan,
    resource::{HerePlanContext, PlanContext, PlanResource, TrieContext, TrieRef},
    utils::{find_plan_file_from_pkg, read_toml_file},
};
use indexmap::IndexMap;
use ros_plan_format::{
    eval::Value,
    key::{Key, KeyOwned},
    link::Link,
    parameter::ParamName,
    plan::Plan,
    socket::Socket,
    subplan::{HerePlan, Subplan, SubplanTable},
};
use std::{
    collections::VecDeque,
    path::{Path, PathBuf},
};

pub struct PlanVisitor {
    queue: VecDeque<Job>,
}

impl PlanVisitor {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

    pub fn traverse(
        &mut self,
        path: &Path,
        args: Option<IndexMap<ParamName, Value>>,
    ) -> Result<PlanResource, Error> {
        let root = TrieRef::default();
        let assign_args = args.unwrap_or_default();

        let mut context = PlanResource {
            root: root.clone(),
            // plan_map: HashMap::new(),
            node_map: IndexMap::new(),
            link_map: IndexMap::new(),
        };

        self.insert_root_plan(&mut context, root.clone(), path, assign_args)?;

        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::InsertHerePlan(job) => {
                    self.insert_hereplan(&mut context, job)?;
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
        context: &mut PlanResource,
        root: TrieRef,
        plan_path: &Path,
        assign_args: IndexMap<ParamName, Value>,
    ) -> Result<(), Error> {
        // Read plan file
        let mut plan: Plan = read_toml_file(&plan_path)?;
        eval_plan(&mut plan, assign_args)?;

        // Create the context for the inserted plan
        let (plan_ctx, subplan_tab) = to_plan_context(plan_path.to_path_buf(), plan);

        // Insert node entries to the global context
        {
            let node_entries = plan_ctx.node_map.iter().map(|(node_ident, node_arc)| {
                let node_weak = node_arc.downgrade();
                let node_key = Key::root() / node_ident;
                (node_key, node_weak)
            });
            context.node_map.extend(node_entries);
        }

        // Insert the plan context to the root node.
        {
            let mut guard = root.write();
            assert!(guard.context.is_none());
            guard.context = Some(plan_ctx.into());
        }

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

    fn insert_plan(
        &mut self,
        context: &mut PlanResource,
        job: InsertPlanFileJob,
    ) -> Result<(), Error> {
        let InsertPlanFileJob {
            // plan_parent: _,
            current,
            current_prefix,
            child_suffix,
            child_plan_path,
            assign_args,
        } = job;

        let Ok(child_prefix) = &current_prefix / &child_suffix else {
            todo!();
        };

        // Read plan file
        let mut plan: Plan = read_toml_file(&child_plan_path)?;
        eval_plan(&mut plan, assign_args)?;

        // Create the context for the inserted plan
        let (plan_ctx, subplan_tab) = to_plan_context(child_plan_path.clone(), plan);

        // Insert node entries to the global context
        {
            let node_entries = plan_ctx.node_map.iter().map(|(node_ident, node_arc)| {
                let node_weak = node_arc.downgrade();
                let node_key = &child_prefix / node_ident;
                (node_key, node_weak)
            });
            context.node_map.extend(node_entries);
        }

        // Create the child node for the plan
        let plan_child = current.insert(&child_suffix, plan_ctx.into())?;

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

    fn insert_hereplan(
        &mut self,
        context: &mut PlanResource,
        job: InsertHerePlanJob,
    ) -> Result<(), Error> {
        let InsertHerePlanJob {
            plan_parent,
            current,
            current_prefix,
            child_suffix,
            child_hereplan,
        } = job;

        let Ok(child_prefix) = &current_prefix / &child_suffix else {
            unreachable!();
        };

        // Create the context
        let (hereplan_ctx, subplan_tab) = to_hereplan_context(child_hereplan);

        // Insert node entries to the global context
        {
            let node_entries = hereplan_ctx.node_map.iter().map(|(node_ident, node_arc)| {
                let node_weak = node_arc.downgrade();
                let node_key = &child_prefix / node_ident;
                (node_key, node_weak)
            });
            context.node_map.extend(node_entries);
        }

        // Schedule subplan insertion jobs
        {
            let hereplan_child = current.insert(&child_suffix, hereplan_ctx.into())?;

            for (subplan_suffix, subplan) in subplan_tab.0 {
                self.schedule_subplan_insertion(
                    plan_parent.clone(),
                    hereplan_child.clone(),
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
        plan_parent: TrieRef,
        current: TrieRef,
        current_prefix: &Key,
        child_suffix: &Key,
        child_subplan: Subplan,
    ) -> Result<(), Error> {
        let subplan_suffix = child_suffix.to_owned();

        match child_subplan {
            Subplan::File(subplan) => {
                // Resolve the path of the plan file
                let subplan_path = {
                    let subplan_path = &subplan.path;
                    let subplan_path = if subplan_path.is_relative() {
                        let guard = plan_parent.read();
                        let Some(TrieContext::Plan(plan_ctx)) = &guard.context else {
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

                let assign_args: IndexMap<_, _> = subplan
                    .arg
                    .into_iter()
                    .map(|(arg_name, assign)| {
                        let value = assign.into_value().expect("value is not evaluated");
                        (arg_name, value)
                    })
                    .collect();

                self.queue.push_back(
                    InsertPlanFileJob {
                        // plan_parent,
                        current,
                        child_plan_path: subplan_path,
                        current_prefix: current_prefix.to_owned(),
                        child_suffix: subplan_suffix,
                        assign_args,
                    }
                    .into(),
                );
            }
            Subplan::Pkg(subplan) => {
                let path = find_plan_file_from_pkg(&subplan.pkg, &subplan.file)?;
                let assign_args: IndexMap<_, _> = subplan
                    .arg
                    .into_iter()
                    .map(|(arg_name, assign)| {
                        let value = assign.into_value().expect("value is not evaluated");
                        (arg_name, value)
                    })
                    .collect();

                self.queue.push_back(
                    InsertPlanFileJob {
                        // plan_parent,
                        current,
                        child_plan_path: path,
                        current_prefix: current_prefix.to_owned(),
                        child_suffix: subplan_suffix,
                        assign_args,
                    }
                    .into(),
                );
            }
            Subplan::Here(here_plan) => {
                self.queue.push_back(
                    InsertHerePlanJob {
                        plan_parent,
                        current,
                        child_hereplan: here_plan,
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

enum Job {
    InsertHerePlan(InsertHerePlanJob),
    InsertPlanFile(InsertPlanFileJob),
}

impl From<InsertPlanFileJob> for Job {
    fn from(v: InsertPlanFileJob) -> Self {
        Self::InsertPlanFile(v)
    }
}

impl From<InsertHerePlanJob> for Job {
    fn from(v: InsertHerePlanJob) -> Self {
        Self::InsertHerePlan(v)
    }
}

pub struct InsertHerePlanJob {
    plan_parent: TrieRef,
    current: TrieRef,
    current_prefix: KeyOwned,
    child_suffix: KeyOwned,
    child_hereplan: HerePlan,
}

pub struct InsertPlanFileJob {
    // plan_parent: TrieRef,
    current: TrieRef,
    current_prefix: KeyOwned,
    child_suffix: KeyOwned,
    child_plan_path: PathBuf,
    assign_args: IndexMap<ParamName, Value>,
}

fn to_plan_context(path: PathBuf, plan_cfg: Plan) -> (PlanContext, SubplanTable) {
    let local_node_map: IndexMap<_, _> = plan_cfg
        .node
        .0
        .into_iter()
        .map(|(node_ident, node_cfg)| {
            let node_arc: NodeArc = NodeContext { config: node_cfg }.into();
            (node_ident, node_arc)
        })
        .collect();

    let local_socket_map: IndexMap<_, _> = plan_cfg
        .socket
        .0
        .into_iter()
        .map(|(socket_ident, socket_cfg)| {
            let socket_arc: SocketArc = to_socket_context(socket_cfg).into();
            (socket_ident, socket_arc)
        })
        .collect();

    let local_link_map: IndexMap<_, _> = plan_cfg
        .link
        .0
        .into_iter()
        .map(|(link_ident, link_cfg)| {
            let link_arc: LinkArc = to_link_context(link_cfg).into();
            (link_ident, link_arc)
        })
        .collect();

    let plan_ctx = PlanContext {
        path,
        arg: plan_cfg.arg,
        socket_map: local_socket_map,
        node_map: local_node_map,
        link_map: local_link_map,
    };

    (plan_ctx, plan_cfg.subplan)
}

fn to_hereplan_context(hereplan: HerePlan) -> (HerePlanContext, SubplanTable) {
    let local_node_map: IndexMap<_, _> = hereplan
        .node
        .0
        .into_iter()
        .map(|(node_ident, node_cfg)| {
            let node_arc: NodeArc = NodeContext { config: node_cfg }.into();
            (node_ident, node_arc)
        })
        .collect();
    let local_link_map: IndexMap<_, _> = hereplan
        .link
        .0
        .into_iter()
        .map(|(link_ident, link_cfg)| {
            let link_arc: LinkArc = to_link_context(link_cfg).into();
            (link_ident, link_arc)
        })
        .collect();

    let ctx = HerePlanContext {
        node_map: local_node_map,
        link_map: local_link_map,
    };
    (ctx, hereplan.subplan)
}

fn to_socket_context(socket_ctx: Socket) -> SocketContext {
    match socket_ctx {
        Socket::Pub(config) => PubSocketContext { config, src: None }.into(),
        Socket::Sub(config) => SubSocketContext { config, dst: None }.into(),
        Socket::Srv(config) => ServerSocketContext {
            config,
            listen: None,
        }
        .into(),
        Socket::Qry(config) => QuerySocketContext {
            config,
            connect: None,
        }
        .into(),
    }
}

fn to_link_context(link: Link) -> LinkContext {
    match link {
        Link::Pubsub(link) => PubSubLinkContext {
            config: link,
            src: None,
            dst: None,
        }
        .into(),
        Link::Service(link) => ServiceLinkContext {
            config: link,
            listen: None,
            connect: None,
        }
        .into(),
    }
}
