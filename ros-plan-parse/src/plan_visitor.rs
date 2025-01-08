use crate::{
    context::{
        GlobalContextV1, HerePlanContextV1, NodeArc, NodeContext, PlanContextV1, TrieContext,
        TrieRef,
    },
    error::Error,
    eval::eval_plan,
    utils::{find_plan_file_from_pkg, read_toml_file},
};
use ros_plan_format::{
    eval::Value,
    key::{Key, KeyOwned},
    parameter::ParamName,
    plan::Plan,
    subplan::{HerePlan, Subplan},
};
use std::{
    collections::{HashMap, VecDeque},
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
        args: Option<HashMap<ParamName, Value>>,
    ) -> Result<GlobalContextV1, Error> {
        let root = TrieRef::default();
        let assign_args = args.unwrap_or_default();

        let mut context = GlobalContextV1 {
            namespace: KeyOwned::new_root(),
            root: root.clone(),
            node_map: HashMap::new(),
        };

        self.insert_plan(
            &mut context,
            InsertPlanFileJob {
                plan_parent: root.clone(),
                current: root.clone(),
                current_prefix: KeyOwned::new_root(),
                child_suffix: KeyOwned::new_empty(),
                child_plan_path: path.to_owned(),
                assign_args,
            },
        )?;

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

    fn insert_plan(
        &mut self,
        context: &mut GlobalContextV1,
        job: InsertPlanFileJob,
    ) -> Result<(), Error> {
        let InsertPlanFileJob {
            plan_parent: _,
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
        let plan_ctx = {
            let local_node_map: HashMap<_, _> = plan
                .node
                .0
                .into_iter()
                .map(|(node_ident, node_cfg)| {
                    let node_arc: NodeArc = NodeContext { config: node_cfg }.into();
                    (node_ident, node_arc)
                })
                .collect();

            PlanContextV1 {
                path: child_plan_path.to_owned(),
                socket_map: plan.socket.0,
                arg: plan.arg,
                node_map: local_node_map,
                link_map: plan.link.0,
            }
        };

        // Insert node entries to the global context
        {
            let node_entries = plan_ctx.node_map.iter().map(|(node_ident, node_arc)| {
                let node_weak = node_arc.downgrade();
                let node_key = &child_prefix / node_ident;
                (node_key, node_weak)
            });
            context.node_map.extend(node_entries);
        }

        let plan_child = current.insert(&child_suffix, plan_ctx.into())?;

        // Schedule subplan insertion jobs
        for (subplan_suffix, subplan) in plan.subplan.0 {
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
        context: &mut GlobalContextV1,
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
            todo!();
        };

        // Create the context
        let hereplan_ctx = {
            let local_node_map: HashMap<_, _> = child_hereplan
                .node
                .0
                .into_iter()
                .map(|(node_ident, node_cfg)| {
                    let node_arc: NodeArc = NodeContext { config: node_cfg }.into();
                    (node_ident, node_arc)
                })
                .collect();

            HerePlanContextV1 {
                node_map: local_node_map,
                link_map: child_hereplan.link.0,
            }
        };

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

            for (subplan_suffix, subplan) in child_hereplan.subplan.0 {
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
                        let Some(TrieContext::PlanV1(plan_ctx)) = &guard.context else {
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

                let assign_args: HashMap<_, _> = subplan
                    .arg
                    .into_iter()
                    .map(|(arg_name, assign)| {
                        let value = assign.into_value().expect("value is not evaluated");
                        (arg_name, value)
                    })
                    .collect();

                self.queue.push_back(
                    InsertPlanFileJob {
                        plan_parent,
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
                let assign_args: HashMap<_, _> = subplan
                    .arg
                    .into_iter()
                    .map(|(arg_name, assign)| {
                        let value = assign.into_value().expect("value is not evaluated");
                        (arg_name, value)
                    })
                    .collect();

                self.queue.push_back(
                    InsertPlanFileJob {
                        plan_parent,
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
    plan_parent: TrieRef,
    current: TrieRef,
    current_prefix: KeyOwned,
    child_suffix: KeyOwned,
    child_plan_path: PathBuf,
    assign_args: HashMap<ParamName, Value>,
}
