mod eval;
mod lua;
mod store_eval;

use crate::{
    context::{arg::ArgContext, expr::ExprContext},
    error::Error,
    resource::{Resource, ResourceTreeRef, ScopeKind},
};
use indexmap::IndexMap;
use lua::{new_lua, ValueToLua};
use mlua::prelude::*;
use ros_plan_format::{expr::Value, parameter::ParamName};
use std::collections::VecDeque;
use store_eval::{
    store_eval_link_map, store_eval_node_map, store_eval_root_arg_table, store_eval_socket_map,
    store_eval_subplan_table, StoreEval,
};

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
            override_arg_table(arg_map, args)?;
            store_eval_root_arg_table(arg_map)?;
        }

        // Traverse all nodes in the tree
        self.queue.push_back(Job::PlanFile {
            current: root.clone(),
        });

        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::PlanFile { current } => self.eval_plan_file(&current)?,
                Job::Group { current, lua } => self.eval_group(&lua, current)?,
            }
        }

        Ok(())
    }

    fn eval_plan_file(&mut self, current: &ResourceTreeRef) -> Result<(), Error> {
        // Create a new Lua context
        let lua = new_lua()?;

        {
            let mut current_guard = current.write();
            let scope_guard = current_guard
                .value
                .as_plan_file_mut()
                .expect("expect a plan file variant");

            // Populate arguments into the global scope
            load_arg_table(&lua, &scope_guard.arg_map)?;

            // Populate local variables into the global scope
            load_var_table(&lua, &mut scope_guard.var_map)?;

            // Lock global variables.
            lua.globals().set_readonly(true);

            // Evaluate the node table
            store_eval_node_map(&lua, &mut scope_guard.node_map)?;

            // Evaluate the link table
            store_eval_link_map(&lua, &mut scope_guard.link_map)?;

            // Evaluate the socket table
            store_eval_socket_map(&lua, &mut scope_guard.socket_map)?;

            // Evaluate assigned arguments and `when` conditions on
            // subscopes.
            store_eval_subplan_table(&lua, &current_guard.children)?;
        }

        // Schedule jobs to visit child scopes
        self.schedule_subplan_jobs(&lua, current)?;

        Ok(())
    }

    fn eval_group(&mut self, lua: &Lua, current: ResourceTreeRef) -> Result<(), Error> {
        {
            let mut current_guard = current.write();
            let scope_guard = current_guard
                .value
                .as_group_mut()
                .expect("expect a plan file variant");

            // Evaluate the node table
            store_eval_node_map(lua, &mut scope_guard.node_map)?;

            // Evaluate the link table
            store_eval_link_map(lua, &mut scope_guard.link_map)?;

            // Evaluate assigned arguments and `when` condition on
            // subscopes
            store_eval_subplan_table(lua, &current_guard.children)?;
        }

        // Schedule jobs to visit child scopes
        self.schedule_subplan_jobs(lua, &current)?;
        Ok(())
    }

    fn schedule_subplan_jobs(&mut self, lua: &Lua, current: &ResourceTreeRef) -> Result<(), Error> {
        let guard = current.read();
        for (_key, child) in &guard.children {
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
    PlanFile { current: ResourceTreeRef },
    Group { current: ResourceTreeRef, lua: Lua },
}

fn load_arg_table(lua: &Lua, arg_table: &IndexMap<ParamName, ArgContext>) -> Result<(), Error> {
    let globals = lua.globals();
    globals.set_readonly(false);

    for (name, arg) in arg_table {
        let value = arg.result.as_ref().unwrap();
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

fn override_arg_table(
    arg_table: &mut IndexMap<ParamName, ArgContext>,
    override_arg: IndexMap<ParamName, Value>,
) -> Result<(), Error> {
    for (_, entry) in arg_table.iter_mut() {
        entry.override_ = None;
    }

    for (name, override_) in override_arg {
        let Some(arg_ctx) = arg_table.get_mut(&name) else {
            return Err(Error::ArgumentNotFound { name });
        };
        arg_ctx.override_ = Some(override_.into());
    }

    Ok(())
}
