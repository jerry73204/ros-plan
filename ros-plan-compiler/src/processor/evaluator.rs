use crate::{
    context::{
        arg::ArgCtx,
        link::{PubSubLinkCtx, PubSubLinkShared, ServiceLinkCtx, ServiceLinkShared},
        node::{NodeCtx, NodeShared},
        node_socket::{NodeCliShared, NodePubShared, NodeSrvShared, NodeSubShared},
        plan_socket::{PlanCliShared, PlanPubShared, PlanSrvShared, PlanSubShared},
    },
    error::Error,
    eval::{new_lua, ValueStore, ValueToLua},
    program::Program,
    scope::{GroupScopeShared, IncludeShared, ScopeRef},
};
use indexmap::IndexMap;
use mlua::prelude::*;
use ros_plan_format::{
    expr::Value, key::KeyOwned, link::LinkIdent, node::NodeIdent, node_socket::NodeSocketIdent,
    parameter::ParamName, plan_socket::PlanSocketIdent,
};
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
        // Get the root include context.
        let root_include = program.root_include();

        // Assign arguments to the root include context
        let assign_arg = args
            .into_iter()
            .map(|(name, value)| (name, ValueStore::new(value.into())))
            .collect();
        root_include.with_write(|mut guard| {
            guard.assign_arg = assign_arg;
        });

        // Insert the root include to the job queue.
        self.queue.push_back(Job::Include {
            include: root_include,
        });

        // Process jobs
        while let Some(job) = self.queue.pop_front() {
            match job {
                Job::Include { include: current } => self.eval_include(&current)?,
                Job::Group {
                    group: current,
                    lua,
                } => self.eval_group(&lua, &current)?,
            }
        }

        Ok(())
    }

    fn eval_include(&mut self, include: &IncludeShared) -> Result<(), Error> {
        // Create a new Lua context
        let lua = new_lua()?;

        let plan = include.with_read(|include_ctx| -> Result<_, Error> {
            // If the plan scope is not loaded yet, skip the
            // evaluation.
            let Some(plan) = &include_ctx.plan else {
                return Ok(None);
            };

            // Populate arguments into the global scope
            plan.with_write(|mut plan_ctx| {
                load_arg_table(&lua, &mut plan_ctx.arg, &include_ctx.assign_arg)
            })?;

            Ok(Some(plan.clone()))
        })?;

        let Some(plan) = plan else {
            return Ok(());
        };

        plan.with_write(|mut guard| -> Result<_, Error> {
            // Populate local variables into the global scope
            load_var_table(&lua, &mut guard.var)?;

            // Lock global variables.
            lua.globals().set_readonly(true);

            // Evaluate the node table
            eval_node_map(&lua, &mut guard.node)?;

            // Evaluate the link table
            eval_pubsub_link_map(&lua, &mut guard.pubsub_link)?;
            eval_service_link_map(&lua, &mut guard.service_link)?;

            // Evaluate the socket table
            eval_plan_pub_map(&lua, &guard.pub_)?;
            eval_plan_sub_map(&lua, &guard.sub)?;
            eval_plan_srv_map(&lua, &guard.srv)?;
            eval_plan_cli_map(&lua, &guard.cli)?;

            // Evaluate assigned arguments and `when` conditions on
            // subscopes.
            eval_include_table(&lua, &guard.include)?;
            eval_group_table(&lua, &guard.group)?;

            Ok(())
        })?;

        // Schedule jobs to visit child scopes
        plan.with_read(|guard| -> Result<_, Error> {
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
            eval_pubsub_link_map(&lua, &mut guard.pubsub_link)?;
            eval_service_link_map(&lua, &mut guard.service_link)?;

            // Evaluate assigned arguments and `when` condition on
            // subscopes
            eval_include_table(lua, &mut guard.include)?;
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
                group: child.clone(),
                lua: lua.clone(),
            };
            self.queue.push_back(job);
        }

        for include in guard.include().values() {
            let job = Job::Include {
                include: include.clone(),
            };
            self.queue.push_back(job);
        }

        Ok(())
    }
}

#[derive(Debug)]
enum Job {
    Include { include: IncludeShared },
    Group { group: GroupScopeShared, lua: Lua },
}

fn load_arg_table(
    lua: &Lua,
    arg_table: &mut IndexMap<ParamName, ArgCtx>,
    assign_arg: &IndexMap<ParamName, ValueStore>,
) -> Result<(), Error> {
    let globals = lua.globals();
    globals.set_readonly(false);

    for (name, arg) in arg_table {
        let ArgCtx { ty, default, .. } = arg;

        let value = if let Some(value) = assign_arg.get(name) {
            value.get_stored()?
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

fn eval_node(node: &mut NodeCtx, lua: &Lua) -> Result<(), Error> {
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

    eval_node_pub_map(lua, &node.pub_)?;
    eval_node_sub_map(lua, &node.sub)?;
    eval_node_srv_map(lua, &node.srv)?;
    eval_node_cli_map(lua, &node.cli)?;

    Ok(())
}

fn eval_node_map(lua: &Lua, node_map: &mut IndexMap<NodeIdent, NodeShared>) -> Result<(), Error> {
    for (_key, shared) in node_map {
        let owned = shared.upgrade().unwrap();
        let mut guard = owned.write();
        eval_node(&mut guard, lua)?;
    }

    Ok(())
}

fn eval_pubsub_link_map(
    lua: &Lua,
    link_map: &mut IndexMap<LinkIdent, PubSubLinkShared>,
) -> Result<(), Error> {
    for (_key, link) in link_map {
        link.with_write(|mut guard| eval_pubsub_link(lua, &mut guard))?;
    }

    Ok(())
}

fn eval_service_link_map(
    lua: &Lua,
    link_map: &mut IndexMap<LinkIdent, ServiceLinkShared>,
) -> Result<(), Error> {
    for (_key, link) in link_map {
        link.with_write(|mut guard| eval_service_link(lua, &mut guard))?;
    }

    Ok(())
}

fn eval_pubsub_link(lua: &Lua, link: &mut PubSubLinkCtx) -> Result<(), Error> {
    let PubSubLinkCtx {
        when,
        src_key,
        dst_key,
        ..
    } = link;

    if let Some(when) = when {
        when.eval_and_store(lua)?;
    };

    for key in src_key {
        key.eval_and_store(lua)?;
    }

    for key in dst_key {
        key.eval_and_store(lua)?;
    }

    Ok(())
}

fn eval_service_link(lua: &Lua, link: &mut ServiceLinkCtx) -> Result<(), Error> {
    let ServiceLinkCtx {
        when,
        listen_key,
        connect_key,
        ..
    } = link;

    if let Some(when) = when {
        when.eval_and_store(lua)?;
    };

    listen_key.eval_and_store(lua)?;

    for key in connect_key {
        key.eval_and_store(lua)?;
    }

    Ok(())
}

fn eval_include_table(
    lua: &Lua,
    children: &IndexMap<KeyOwned, IncludeShared>,
) -> Result<(), Error> {
    for include in children.values() {
        include.with_write(|mut guard| -> Result<_, Error> {
            if let Some(when) = &mut guard.when {
                when.eval_and_store(lua)?;
            };
            for (_name, value) in &mut guard.assign_arg {
                value.eval_and_store(lua)?;
            }
            Ok(())
        })?;
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
        };
    }

    Ok(())
}

fn eval_plan_pub_map(
    lua: &Lua,
    map: &IndexMap<PlanSocketIdent, PlanPubShared>,
) -> Result<(), Error> {
    for (_key, shared) in map {
        shared.with_write(|mut guard| -> Result<_, Error> {
            for key in &mut guard.src_key {
                key.eval_and_store(lua)?;
            }
            Ok(())
        })?;
    }

    Ok(())
}

fn eval_plan_sub_map(
    lua: &Lua,
    map: &IndexMap<PlanSocketIdent, PlanSubShared>,
) -> Result<(), Error> {
    for (_key, shared) in map {
        shared.with_write(|mut guard| -> Result<_, Error> {
            for key in &mut guard.dst_key {
                key.eval_and_store(lua)?;
            }
            Ok(())
        })?;
    }

    Ok(())
}

fn eval_plan_srv_map(
    lua: &Lua,
    map: &IndexMap<PlanSocketIdent, PlanSrvShared>,
) -> Result<(), Error> {
    for (_key, shared) in map {
        shared.with_write(|mut guard| -> Result<_, Error> {
            guard.listen_key.eval_and_store(lua)?;
            Ok(())
        })?;
    }

    Ok(())
}

fn eval_plan_cli_map(
    lua: &Lua,
    map: &IndexMap<PlanSocketIdent, PlanCliShared>,
) -> Result<(), Error> {
    for (_key, shared) in map {
        shared.with_write(|mut guard| -> Result<_, Error> {
            for key in &mut guard.connect_key {
                key.eval_and_store(lua)?;
            }
            Ok(())
        })?;
    }

    Ok(())
}

fn eval_node_pub_map(
    lua: &Lua,
    map: &IndexMap<NodeSocketIdent, NodePubShared>,
) -> Result<(), Error> {
    for (_key, shared) in map {
        shared.with_write(|mut guard| -> Result<_, Error> {
            if let Some(remap_from) = &mut guard.remap_from {
                remap_from.eval_and_store(lua)?;
            }
            Ok(())
        })?;
    }

    Ok(())
}

fn eval_node_sub_map(
    lua: &Lua,
    map: &IndexMap<NodeSocketIdent, NodeSubShared>,
) -> Result<(), Error> {
    for (_key, shared) in map {
        shared.with_write(|mut guard| -> Result<_, Error> {
            if let Some(remap_from) = &mut guard.remap_from {
                remap_from.eval_and_store(lua)?;
            }
            Ok(())
        })?;
    }

    Ok(())
}

fn eval_node_srv_map(
    lua: &Lua,
    map: &IndexMap<NodeSocketIdent, NodeSrvShared>,
) -> Result<(), Error> {
    for (_key, shared) in map {
        shared.with_write(|mut guard| -> Result<_, Error> {
            if let Some(remap_from) = &mut guard.remap_from {
                remap_from.eval_and_store(lua)?;
            }
            Ok(())
        })?;
    }

    Ok(())
}

fn eval_node_cli_map(
    lua: &Lua,
    map: &IndexMap<NodeSocketIdent, NodeCliShared>,
) -> Result<(), Error> {
    for (_key, shared) in map {
        shared.with_write(|mut guard| -> Result<_, Error> {
            if let Some(remap_from) = &mut guard.remap_from {
                remap_from.eval_and_store(lua)?;
            }
            Ok(())
        })?;
    }

    Ok(())
}
