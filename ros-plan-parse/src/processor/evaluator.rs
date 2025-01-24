mod eval;
mod lua;
mod store_eval;

use crate::{
    context::{arg::ArgContext, expr::ExprContext},
    error::Error,
    resource::Resource,
    scope::{ScopeKind, ScopeTreeRef},
};
use indexmap::IndexMap;
use lua::{new_lua, ValueToLua};
use mlua::prelude::*;
use ros_plan_format::{expr::Value, parameter::ParamName};
use std::collections::VecDeque;
use store_eval::{store_eval_node_map, store_eval_subplan_table, StoreEval};

#[derive(Debug, Default)]
pub struct Evaluator {
    queue: VecDeque<Job>,
}

impl Evaluator {
    pub fn eval_resource(
        &mut self,
        resource: &mut Resource,
        args: IndexMap<ParamName, Value>,
    ) -> Result<(), Error> {
        {
            let root = resource
                .root
                .as_ref()
                .expect("the resource tree must be constructed before evaluation");

            // Assign arguments provided from caller
            {
                let mut root_scope = root
                    .as_plan_file_mut()
                    .expect("the root scope must be a plan file variant");
                let arg_map = &mut root_scope.arg_map;
                assign_arg_table(arg_map, args)?;
            }

            // Traverse all nodes in the tree
            self.queue.push_back(Job::PlanFile {
                current: root.clone(),
            });
        }

        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::PlanFile { current } => self.eval_plan_file(&current)?,
                Job::Group { current, lua } => self.eval_group(&lua, current)?,
            }
        }

        Ok(())
    }

    fn eval_plan_file(&mut self, current: &ScopeTreeRef) -> Result<(), Error> {
        // Create a new Lua context
        let lua = new_lua()?;

        {
            let mut current_guard = current.write();
            let scope_guard = current_guard
                .value
                .as_plan_file_mut()
                .expect("expect a plan file variant");

            // Populate arguments into the global scope
            load_arg_table(&lua, &mut scope_guard.arg_map)?;

            // Populate local variables into the global scope
            load_var_table(&lua, &mut scope_guard.var_map)?;

            // Lock global variables.
            lua.globals().set_readonly(true);

            // Evaluate the node table
            store_eval_node_map(&lua, &mut scope_guard.node_map)?;

            // Evaluate the link table
            // store_eval_link_map(&lua, &mut scope_guard.link_map)?;

            // Evaluate the socket table
            // store_eval_socket_map(&lua, &mut scope_guard.socket_map)?;

            // Evaluate assigned arguments and `when` conditions on
            // subscopes.
            store_eval_subplan_table(&lua, &current_guard.children)?;
        }

        // Schedule jobs to visit child scopes
        self.schedule_subplan_jobs(&lua, current)?;

        Ok(())
    }

    fn eval_group(&mut self, lua: &Lua, current: ScopeTreeRef) -> Result<(), Error> {
        {
            let mut current_guard = current.write();
            let scope_guard = current_guard
                .value
                .as_group_mut()
                .expect("expect a plan file variant");

            // Evaluate the node table
            store_eval_node_map(lua, &mut scope_guard.node_map)?;

            // Evaluate the link table
            // store_eval_link_map(lua, &mut scope_guard.link_map)?;

            // Evaluate assigned arguments and `when` condition on
            // subscopes
            store_eval_subplan_table(lua, &current_guard.children)?;
        }

        // Schedule jobs to visit child scopes
        self.schedule_subplan_jobs(lua, &current)?;
        Ok(())
    }

    fn schedule_subplan_jobs(&mut self, lua: &Lua, current: &ScopeTreeRef) -> Result<(), Error> {
        let guard = current.read();
        for child in guard.children.values() {
            let job = match child.kind() {
                ScopeKind::PlanFile => Job::PlanFile {
                    current: child.clone(),
                },
                ScopeKind::Group => Job::Group {
                    current: child.clone(),
                    lua: lua.clone(),
                },
            };
            self.queue.push_back(job);
        }
        Ok(())
    }
}

#[derive(Debug)]
enum Job {
    PlanFile { current: ScopeTreeRef },
    Group { current: ScopeTreeRef, lua: Lua },
}

fn load_arg_table(lua: &Lua, arg_table: &mut IndexMap<ParamName, ArgContext>) -> Result<(), Error> {
    let globals = lua.globals();
    globals.set_readonly(false);

    for (name, arg) in arg_table {
        let ArgContext {
            ty,
            default,
            assign,
            ..
        } = arg;

        let value = if let Some(assign) = assign {
            assign
                .result
                .as_ref()
                .expect("the assign field must be evaluated beforehand")
        } else if let Some(default) = default {
            default.store_eval(lua)?;
            default.result.as_ref().unwrap()
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

fn load_var_table(
    lua: &Lua,
    var_table: &mut IndexMap<ParamName, ExprContext>,
) -> Result<(), Error> {
    let globals = lua.globals();

    // Evaluate local variables and populate them to the global scope
    for (name, entry) in var_table {
        // Check if the variable is already defined
        if globals.contains_key(name.as_str())? {
            return Err(Error::MultipleDefinitionOfVariable(name.clone()));
        }

        // Lock the global variables during evaluation
        globals.set_readonly(true);
        entry.store_eval(lua)?;
        globals.set_readonly(false);
        globals.set(name.as_ref(), ValueToLua(entry.result.as_ref().unwrap()))?;
    }
    globals.set_readonly(true);

    Ok(())
}

fn assign_arg_table(
    arg_table: &mut IndexMap<ParamName, ArgContext>,
    assign_arg: IndexMap<ParamName, Value>,
) -> Result<(), Error> {
    for (_, entry) in arg_table.iter_mut() {
        entry.assign = None;
    }

    for (name, value) in assign_arg {
        let Some(arg_ctx) = arg_table.get_mut(&name) else {
            return Err(Error::ArgumentNotFound { name });
        };

        let mut ctx = ExprContext::new(value.clone().into());
        ctx.result = Some(value);
        arg_ctx.assign = Some(ctx);
    }

    Ok(())
}
