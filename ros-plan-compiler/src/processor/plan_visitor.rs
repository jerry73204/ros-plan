mod convert;

use crate::{
    error::Error,
    eval::{BoolStore, TextStore, ValueStore},
    program::Program,
    scope::{
        IncludeCtx, IncludeLocation, IncludeShared, PkgFileLocation, PlanScopeShared, ScopeMutExt,
        ScopeShared,
    },
    utils::{read_yaml_file, shared_table::SharedTable},
};
use convert::{to_group_scope, to_plan_scope, SubScopeList};
use indexmap::IndexMap;
use ros_plan_format::{
    key::{Key, KeyOwned},
    plan::Plan,
    subplan::GroupCfg,
};
use std::{
    collections::VecDeque,
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
            plan_tab: SharedTable::default(),
            group_tab: SharedTable::default(),
            include_tab: SharedTable::default(),
        };

        self.load_root_plan(&mut context, path)?;

        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::InsertGroup(job) => {
                    self.insert_group(&mut context, job)?;
                }
                Job::LoadPlanFile(job) => {
                    self.load_plan(&mut context, job)?;
                }
            }
        }

        Ok(context)
    }

    fn load_root_plan(&mut self, program: &mut Program, plan_path: &Path) -> Result<(), Error> {
        let scope_key = Key::root();

        let Some(plan_dir) = plan_path.parent() else {
            todo!();
        };

        // Read plan file
        let plan: Plan = read_yaml_file(plan_path)?;

        // Create the context for the inserted plan
        let (plan_ctx, subscope) =
            to_plan_scope(program, plan_path.to_path_buf(), scope_key, plan)?;

        // Insert the plan context to the root node.
        let plan_shared = program.plan_tab.insert(plan_ctx);

        let include_ctx = IncludeCtx {
            location: IncludeLocation::Path(plan_path.to_path_buf()),
            when: None,
            assign_arg: IndexMap::new(),
            plan: Some(plan_shared.clone()),
        };
        let _include_shared = program.include_tab.insert(include_ctx);

        // Schedule subscope insertion jobs
        self.schedule_insertion_jobs(
            program,
            plan_dir,
            plan_shared.clone(),
            plan_shared.clone().into(),
            Key::root(),
            subscope,
        )?;

        Ok(())
    }

    fn load_plan(&mut self, program: &mut Program, job: LoadPlanFile) -> Result<(), Error> {
        let LoadPlanFile {
            cwd,
            include: include_shared,
            prefix,
        } = job;

        // Resolve the included file location
        let plan_path = include_shared.with_read(|include_guard| -> Result<_, Error> {
            include_guard.location.resolve_plan(&cwd)
        })?;

        // If the plan path cannot be resolved, defer the job.
        let Some(plan_path) = plan_path else {
            return Ok(());
        };

        // Create the plan scope.
        let plan_cfg: Plan = read_yaml_file(&plan_path)?;
        let (plan_ctx, subscope) = to_plan_scope(program, plan_path.clone(), &prefix, plan_cfg)?;
        let plan_shared = program.plan_tab.insert(plan_ctx);

        // Save the plan scope in the include context.
        include_shared.with_write(|mut include_guard| {
            include_guard.plan = Some(plan_shared.clone());
        });

        // Schedule scope insertion jobs for the newly created plan.
        self.schedule_insertion_jobs(
            program,
            &cwd,
            plan_shared.clone(),
            plan_shared.into(),
            &prefix,
            subscope,
        )?;

        Ok(())
    }

    fn insert_group(&mut self, program: &mut Program, job: InsertGroup) -> Result<(), Error> {
        let InsertGroup {
            cwd,
            plan: plan_parent,
            parent_scope: current,
            parent_prefix: current_prefix,
            insert_suffix: child_suffix,
            group_cfg: child_group,
        } = job;

        let Ok(child_prefix) = &current_prefix / &child_suffix else {
            unreachable!();
        };

        // Create the context
        let (group_scope, subscope) = to_group_scope(program, &child_prefix, child_group)?;
        let group_shared = program.group_tab.insert(group_scope);

        // Schedule subscope insertion jobs
        current.with_write(|mut guard| -> Result<_, Error> {
            guard
                .scope_entry(child_suffix)?
                .insert_group(group_shared.clone());
            Ok(())
        })?;

        self.schedule_insertion_jobs(
            program,
            &cwd,
            plan_parent.clone(),
            group_shared.clone().into(),
            &child_prefix,
            subscope,
        )?;

        Ok(())
    }

    fn schedule_insertion_jobs(
        &mut self,
        program: &mut Program,
        cwd: &Path,
        plan_parent: PlanScopeShared,
        current: ScopeShared,
        current_prefix: &Key,
        subscope: SubScopeList,
    ) -> Result<(), Error> {
        current.with_write(|mut plan_guard| -> Result<_, Error> {
            let SubScopeList {
                include: include_list,
                group: group_list,
            } = subscope;

            // Create group insertion jobs.
            for (suffix, group_cfg) in group_list {
                let job = InsertGroup {
                    cwd: cwd.to_path_buf(),
                    plan: plan_parent.clone(),
                    parent_scope: current.clone(),
                    parent_prefix: current_prefix.to_owned(),
                    insert_suffix: suffix.into(),
                    group_cfg,
                };
                self.queue.push_back(job.into());
            }

            // Schedule plan file inclusion jobs
            for (suffix, child_cfg) in include_list {
                let child_prefix = (current_prefix / suffix.as_key()).unwrap();

                let include_ctx = {
                    let location = match (child_cfg.path, child_cfg.pkg, child_cfg.file) {
                        (Some(path), None, None) => IncludeLocation::Path(path),
                        (None, Some(pkg), Some(file)) => {
                            IncludeLocation::PkgFile(PkgFileLocation {
                                pkg: TextStore::new(pkg),
                                file: TextStore::new(file),
                            })
                        }
                        _ => todo!(),
                    };

                    let arg_assign: IndexMap<_, _> = child_cfg
                        .arg
                        .into_iter()
                        .map(|(name, value)| (name, ValueStore::new(value)))
                        .collect();

                    IncludeCtx {
                        location,
                        when: child_cfg.when.map(BoolStore::new),
                        assign_arg: arg_assign,
                        plan: None,
                    }
                };

                // Insert the include reference to the program.
                let include_shared = program.include_tab.insert(include_ctx);
                plan_guard
                    .scope_entry(suffix.into())?
                    .insert_include(include_shared.clone());

                // Schedule the job that loads the included file.
                let job = LoadPlanFile {
                    cwd: cwd.to_path_buf(),
                    include: include_shared,
                    prefix: child_prefix,
                };
                self.queue.push_back(job.into());
            }

            Ok(())
        })
    }
}

#[derive(Debug)]
enum Job {
    InsertGroup(InsertGroup),
    LoadPlanFile(LoadPlanFile),
}

impl From<LoadPlanFile> for Job {
    fn from(v: LoadPlanFile) -> Self {
        Self::LoadPlanFile(v)
    }
}

impl From<InsertGroup> for Job {
    fn from(v: InsertGroup) -> Self {
        Self::InsertGroup(v)
    }
}

#[derive(Debug)]
struct InsertGroup {
    pub cwd: PathBuf,
    pub plan: PlanScopeShared,
    pub parent_scope: ScopeShared,
    pub parent_prefix: KeyOwned,
    pub insert_suffix: KeyOwned,
    pub group_cfg: GroupCfg,
}

#[derive(Debug)]
struct LoadPlanFile {
    pub cwd: PathBuf,
    pub include: IncludeShared,
    pub prefix: KeyOwned,
}

// fn check_arg_assignment(
//     spec: &IndexMap<ParamName, ArgEntry>,
//     assign: &IndexMap<ParamName, ValueOrExpr>,
// ) -> Result<(), Error> {
//     let spec_names: HashSet<_> = spec.keys().collect();
//     let assigned_names: HashSet<_> = assign.keys().collect();

//     // Check if there are unassigned required args
//     for name in &spec_names - &assigned_names {
//         if spec[name].default.is_none() {
//             return Err(Error::RequiredArgumentNotAssigned {
//                 name: name.to_owned(),
//             });
//         }
//     }

//     // Check if there are assigned args not found in spec.
//     if let Some(name) = (&assigned_names - &spec_names).into_iter().next() {
//         return Err(Error::ArgumentNotFound { name: name.clone() });
//     }

//     // Check type compatibility for assigned args
//     for name in &spec_names & &assigned_names {
//         match &assign[name] {
//             ValueOrExpr::Value(value) => {
//                 let assigned_ty = value.ty();
//                 let expect_ty = spec[name].ty;

//                 if assigned_ty != expect_ty {
//                     return Err(Error::TypeMismatch {
//                         expect: expect_ty,
//                         found: assigned_ty,
//                     });
//                 }
//             }
//             ValueOrExpr::Expr { .. } => {
//                 // Cannot check without evaluation. Skip it.
//             }
//         }
//     }

//     Ok(())
// }
