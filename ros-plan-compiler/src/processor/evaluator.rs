use crate::{
    context::{
        arg::ArgCtx,
        node::{NodeCtx, NodeShared},
    },
    error::Error,
    eval::{new_lua, ValueStore, ValueToLua},
    program::Program,
    scope::{GroupScopeShared, PlanScopeShared, ScopeRef},
};
use indexmap::IndexMap;
use mlua::prelude::*;
use ros_plan_format::{expr::Value, key::KeyOwned, node::NodeIdent, parameter::ParamName};
use std::collections::VecDeque;

#[derive(Debug, Default)]
pub struct Evaluator {
    queue: VecDeque<Job>,
}

impl Evaluator {
    pub fn eval(
        &mut self,
        program: &mut Program,
        args: IndexMap<ParamName, Value>,
    ) -> Result<(), Error> {
        {
            let root = program.root();

            // Assign arguments provided from caller
            root.with_write(|mut guard| -> Result<_, Error> {
                assign_arg_table(&mut guard.arg, args)?;
                Ok(())
            })?;

            // Traverse all nodes in the tree
            self.queue.push_back(Job::PlanFile { current: root });
        }

        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::PlanFile { current } => self.eval_plan_file(&current)?,
                Job::Group { current, lua } => self.eval_group(&lua, &current)?,
            }
        }

        Ok(())
    }

    fn eval_plan_file(&mut self, current: &PlanScopeShared) -> Result<(), Error> {
        // Create a new Lua context
        let lua = new_lua()?;

        current.with_write(|mut guard| -> Result<_, Error> {
            // Populate arguments into the global scope
            load_arg_table(&lua, &mut guard.arg)?;

            // Populate local variables into the global scope
            load_var_table(&lua, &mut guard.var)?;

            // Lock global variables.
            lua.globals().set_readonly(true);

            // Evaluate the node table
            eval_node_map(&lua, &mut guard.node)?;

            // Evaluate the link table
            // store_eval_link_map(&lua, &mut scope_guard.link_map)?;

            // Evaluate the socket table
            // store_eval_socket_map(&lua, &mut scope_guard.socket_map)?;

            // Evaluate assigned arguments and `when` conditions on
            // subscopes.
            eval_include_table(&lua, &guard.include)?;
            eval_group_table(&lua, &guard.group)?;

            Ok(())
        })?;

        // Schedule jobs to visit child scopes
        current.with_read(|guard| -> Result<_, Error> {
            self.schedule_subplan_jobs(&lua, &*guard)?;
            Ok(())
        })?;

        Ok(())
    }

    fn eval_group(&mut self, lua: &Lua, current: &GroupScopeShared) -> Result<(), Error> {
        current.with_write(|mut guard| -> Result<(), Error> {
            // Evaluate the node table
            eval_node_map(lua, &mut guard.node)?;

            // Evaluate the link table
            // store_eval_link_map(lua, &mut scope_guard.link_map)?;

            // Evaluate assigned arguments and `when` condition on
            // subscopes
            eval_include_table(lua, &guard.include)?;
            eval_group_table(lua, &guard.group)?;

            Ok(())
        })?;

        // Schedule jobs to visit child scopes
        current.with_read(|scope_guard| -> Result<(), Error> {
            self.schedule_subplan_jobs(lua, &*scope_guard)?;
            Ok(())
        })?;

        Ok(())
    }

    fn schedule_subplan_jobs<S>(&mut self, lua: &Lua, guard: &S) -> Result<(), Error>
    where
        S: ScopeRef,
    {
        for child in guard.group().values() {
            let job = Job::Group {
                current: child.clone(),
                lua: lua.clone(),
            };
            self.queue.push_back(job);
        }

        for child in guard.include().values() {
            let job = Job::PlanFile {
                current: child.clone(),
            };
            self.queue.push_back(job);
        }
        Ok(())
    }
}

#[derive(Debug)]
enum Job {
    PlanFile { current: PlanScopeShared },
    Group { current: GroupScopeShared, lua: Lua },
}

fn load_arg_table(lua: &Lua, arg_table: &mut IndexMap<ParamName, ArgCtx>) -> Result<(), Error> {
    let globals = lua.globals();
    globals.set_readonly(false);

    for (name, arg) in arg_table {
        let ArgCtx {
            ty,
            default,
            assign,
            ..
        } = arg;

        let value = if let Some(assign) = assign {
            assign.get_stored()?
        } else if let Some(default) = default {
            default.eval_and_store(lua)?
        } else {
            return Err(Error::RequiredArgumentNotAssigned { name: name.clone() });
        };

        if value.ty() != *ty {
            return Err(Error::TypeMismatch {
                expect: *ty,
                found: value.ty(),
            });
        }

        lua.globals().set(name.as_ref(), ValueToLua(value))?;
    }

    globals.set_readonly(true);
    Ok(())
}

fn load_var_table(lua: &Lua, var_table: &mut IndexMap<ParamName, ValueStore>) -> Result<(), Error> {
    let globals = lua.globals();

    // Evaluate local variables and populate them to the global scope
    for (name, entry) in var_table {
        // Check if the variable is already defined
        if globals.contains_key(name.as_str())? {
            return Err(Error::MultipleDefinitionOfVariable(name.clone()));
        }

        // Lock the global variables during evaluation
        globals.set_readonly(true);
        entry.eval_and_store(lua)?;
        globals.set_readonly(false);
        globals.set(name.as_ref(), ValueToLua(entry.get_stored()?))?;
    }
    globals.set_readonly(true);

    Ok(())
}

fn assign_arg_table(
    arg_table: &mut IndexMap<ParamName, ArgCtx>,
    assign_arg: IndexMap<ParamName, Value>,
) -> Result<(), Error> {
    for (_, entry) in arg_table.iter_mut() {
        entry.assign = None;
    }

    for (name, value) in assign_arg {
        let Some(arg_ctx) = arg_table.get_mut(&name) else {
            return Err(Error::ArgumentNotFound { name });
        };

        let store = ValueStore::new(value.clone().into());
        arg_ctx.assign = Some(store);
    }

    Ok(())
}

fn eval_node_ctx(node: &mut NodeCtx, lua: &Lua) -> Result<(), Error> {
    if let Some(pkg) = &mut node.pkg {
        pkg.eval_and_store(lua)?;
    }
    if let Some(exec) = &mut node.exec {
        exec.eval_and_store(lua)?;
    }
    if let Some(plugin) = &mut node.plugin {
        plugin.eval_and_store(lua)?;
    }

    for param in node.param.values_mut() {
        param.eval_and_store(lua)?;
    }

    Ok(())
}

fn eval_node_map(lua: &Lua, node_map: &mut IndexMap<NodeIdent, NodeShared>) -> Result<(), Error> {
    for (_key, shared) in node_map {
        let owned = shared.upgrade().unwrap();
        let mut guard = owned.write();
        eval_node_ctx(&mut guard, lua)?;
    }

    Ok(())
}

fn eval_arg_assignment(
    lua: &Lua,
    arg_table: &mut IndexMap<ParamName, ArgCtx>,
) -> Result<(), Error> {
    for (_name, arg) in arg_table {
        let ArgCtx { ty, assign, .. } = arg;

        if let Some(assign) = assign {
            assign.eval_and_store(lua)?;

            let value = assign.get_stored()?;
            if *ty != value.ty() {
                return Err(Error::TypeMismatch {
                    expect: *ty,
                    found: value.ty(),
                });
            }
        }
    }
    Ok(())
}

fn eval_include_table(
    lua: &Lua,
    children: &IndexMap<KeyOwned, PlanScopeShared>,
) -> Result<(), Error> {
    for shared in children.values() {
        let owned = shared.upgrade().unwrap();
        let mut scope = owned.write();

        if let Some(when) = &mut scope.when {
            when.eval_and_store(lua)?;

            if !*when.get_stored()? {
                return Err(Error::EvaluationError {
                    error: "cannot evaluate to a boolean value".to_string(),
                });
            };
        };
        eval_arg_assignment(lua, &mut scope.arg)?;
    }

    Ok(())
}

fn eval_group_table(
    lua: &Lua,
    children: &IndexMap<KeyOwned, GroupScopeShared>,
) -> Result<(), Error> {
    for shared in children.values() {
        let owned = shared.upgrade().unwrap();
        let mut scope = owned.write();

        if let Some(when) = &mut scope.when {
            when.eval_and_store(lua)?;
            if !*when.get_stored()? {
                return Err(Error::EvaluationError {
                    error: "cannot evaluate to a boolean value".to_string(),
                });
            };
        };
    }

    Ok(())
}
