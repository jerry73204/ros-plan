mod convert;

use crate::{
    error::Error,
    eval::{BoolStore, TextStore, ValueStore},
    program::Program,
    scope::{
        IncludeCtx, IncludeLocation, IncludeShared, PathLocation, PkgFileLocation, PlanScopeShared,
        ScopeMutExt, ScopeShared,
    },
    utils::read_yaml_file,
};
use convert::{to_group_scope, to_plan_scope, SubScopeList};
use indexmap::IndexMap;
use ros_plan_format::{
    key::{Key, KeyOwned},
    plan::Plan,
    subplan::GroupCfg,
};
use std::{collections::VecDeque, path::Path};

#[derive(Debug, Default)]
pub struct ProgramBuilder {
    queue: VecDeque<Job>,
    deferred_include: Vec<IncludeShared>,
}

impl ProgramBuilder {
    pub fn load_root_include(&mut self, path: &Path) -> Result<(Program, IncludeShared), Error> {
        let mut program = Program::default();
        let root_include = create_root_include(&mut program, path)?;
        Ok((program, root_include))
    }

    pub fn expand_include(
        &mut self,
        program: &mut Program,
        include: IncludeShared,
    ) -> Result<Vec<IncludeShared>, Error> {
        assert!(program.include_tab.contains(&include));

        self.queue.push_back(LoadPlanFile { include }.into());

        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::InsertGroup(job) => {
                    self.insert_group(program, *job)?;
                }
                Job::LoadPlanFile(job) => {
                    self.load_plan(program, job)?;
                }
            }
        }

        let deferred_include = std::mem::take(&mut self.deferred_include);

        Ok(deferred_include)
    }

    fn load_plan(&mut self, program: &mut Program, job: LoadPlanFile) -> Result<(), Error> {
        let LoadPlanFile {
            include: include_shared,
        } = job;

        // Resolve the included file location
        let (prefix, plan_path) = include_shared.with_read(|guard| -> Result<_, Error> {
            assert!(guard.plan.is_none());
            let prefix = guard.key.clone();
            let plan_path = guard.location.resolve_absolute_path()?;
            Ok((prefix, plan_path))
        })?;

        // If the plan path cannot be resolved, push the include to the deferred set.
        let Some(plan_path) = plan_path else {
            self.deferred_include.push(include_shared);
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
        self.schedule_jobs(
            program,
            plan_shared.clone(),
            plan_shared.into(),
            &prefix,
            subscope,
        )?;

        Ok(())
    }

    fn insert_group(&mut self, program: &mut Program, job: InsertGroup) -> Result<(), Error> {
        let InsertGroup {
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

        self.schedule_jobs(
            program,
            plan_parent.clone(),
            group_shared.clone().into(),
            &child_prefix,
            subscope,
        )?;

        Ok(())
    }

    fn schedule_jobs(
        &mut self,
        program: &mut Program,
        plan_parent: PlanScopeShared,
        curr_scope: ScopeShared,
        curr_prefix: &Key,
        subscope: SubScopeList,
    ) -> Result<(), Error> {
        let plan_dir = plan_parent.with_read(|guard| guard.path.parent().unwrap().to_path_buf());

        curr_scope.with_write(|mut plan_guard| -> Result<_, Error> {
            let SubScopeList {
                include: include_list,
                group: group_list,
            } = subscope;

            // Create group insertion jobs.
            for (suffix, group_cfg) in group_list {
                let job = InsertGroup {
                    plan: plan_parent.clone(),
                    parent_scope: curr_scope.clone(),
                    parent_prefix: curr_prefix.to_owned(),
                    insert_suffix: suffix.into(),
                    group_cfg,
                };
                self.queue.push_back(job.into());
            }

            // Schedule plan file inclusion jobs
            for (suffix, child_cfg) in include_list {
                let child_prefix = (curr_prefix / suffix.as_key()).unwrap();

                let include_ctx = {
                    let location = match (child_cfg.path, child_cfg.pkg, child_cfg.file) {
                        (Some(path), None, None) => IncludeLocation::Path(PathLocation {
                            parent_dir: plan_dir.clone(),
                            path,
                        }),
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
                        key: child_prefix,
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
                    include: include_shared,
                };
                self.queue.push_back(job.into());
            }

            Ok(())
        })
    }
}

#[derive(Debug)]
enum Job {
    InsertGroup(Box<InsertGroup>),
    LoadPlanFile(LoadPlanFile),
}

impl From<LoadPlanFile> for Job {
    fn from(v: LoadPlanFile) -> Self {
        Self::LoadPlanFile(v)
    }
}

impl From<InsertGroup> for Job {
    fn from(v: InsertGroup) -> Self {
        Self::InsertGroup(Box::new(v))
    }
}

#[derive(Debug)]
struct InsertGroup {
    pub plan: PlanScopeShared,
    pub parent_scope: ScopeShared,
    pub parent_prefix: KeyOwned,
    pub insert_suffix: KeyOwned,
    pub group_cfg: GroupCfg,
}

#[derive(Debug)]
struct LoadPlanFile {
    pub include: IncludeShared,
}

fn create_root_include(program: &mut Program, plan_path: &Path) -> Result<IncludeShared, Error> {
    // Read plan file
    let cwd = std::env::current_dir().unwrap();

    let include_ctx = IncludeCtx {
        key: KeyOwned::new_root(),
        location: IncludeLocation::Path(PathLocation {
            parent_dir: cwd,
            path: plan_path.to_path_buf(),
        }),
        when: None,
        assign_arg: IndexMap::new(),
        plan: None,
    };
    let include_shared = program.include_tab.insert(include_ctx);

    Ok(include_shared)
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
