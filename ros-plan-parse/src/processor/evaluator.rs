mod utils;

use crate::{
    context::{
        arg::ArgContext,
        link::{LinkArc, LinkContext},
        node::{NodeArc, NodeContext},
        socket::{SocketArc, SocketContext},
    },
    error::Error,
    resource::{Resource, ResourceTreeRef, Scope, ScopeKind},
};
use indexmap::IndexMap;
use mlua::prelude::*;
use ros_plan_format::{
    expr::{Expr, Value, ValueOrExpr},
    link::LinkIdent,
    node::NodeIdent,
    parameter::ParamName,
    socket::SocketIdent,
};
use std::collections::VecDeque;
use utils::{new_lua, ValueFromLua, ValueToLua};

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
            eval_root_arg_table(arg_map)?;
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
        let lua = new_lua()?;

        {
            let Some(mut plan_res) = current.as_plan_file_mut() else {
                unreachable!()
            };

            // Check multiple definition of arguments and variables.
            for name in plan_res.var_map.keys() {
                if plan_res.arg_map.contains_key(name) {
                    return Err(Error::MultipleDefinitionOfVariable(name.clone()));
                }
            }

            // Populate argument into global scope
            load_arg_table(&lua, &plan_res.arg_map)?;

            // Evaluate local variables and populate them to the global scope
            for (name, entry) in &mut plan_res.var_map {
                // Lock the global variables during evaluation
                lua.globals().set_readonly(true);
                // may_eval_value(&lua, entry)?;
                entry.eval(&lua)?;
                lua.globals().set_readonly(false);
                lua.globals()
                    .set(name.as_ref(), ValueToLua(entry.result.as_ref().unwrap()))?;
            }

            // Lock global variables.
            lua.globals().set_readonly(true);

            // Evaluate the node table
            eval_node_map(&lua, &mut plan_res.node_map)?;

            // Evaluate the link table
            eval_link_map(&lua, &mut plan_res.link_map)?;

            // Evaluate the socket table
            eval_socket_map(&lua, &mut plan_res.socket_map)?;
        }

        // Evaluate assigned arguments and `when` condition on
        // subscopes
        {
            let current = current.read();

            for (_name, child) in &current.children {
                let mut child = child.write();
                match &mut child.value {
                    Scope::PlanFile(scope) => {
                        if let Some(when) = &mut scope.when {
                            when.eval_bool(&lua)?;
                        };
                        eval_arg_table(&lua, &mut scope.arg_map)?;
                    }
                    Scope::Group(scope) => {
                        if let Some(when) = &mut scope.when {
                            when.eval_bool(&lua)?;
                        };
                    }
                }
            }
        }

        // Schedule jobs to visit child scopes
        {
            let guard = current.read();

            for (_name, child) in &guard.children {
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
        }

        Ok(())
    }

    fn eval_group(&mut self, lua: &Lua, current: ResourceTreeRef) -> Result<(), Error> {
        {
            let mut group = current.as_group_mut().unwrap();
            eval_node_map(lua, &mut group.node_map)?;
            eval_link_map(lua, &mut group.link_map)?;
        }

        {
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
    for (name, arg) in arg_table {
        lua.globals()
            .set(name.as_ref(), ValueToLua(arg.result.as_ref().unwrap()))?;
    }

    Ok(())
}

fn eval_node_map(lua: &Lua, node_map: &mut IndexMap<NodeIdent, NodeArc>) -> Result<(), Error> {
    for (_key, node_arc) in node_map {
        let mut node = node_arc.write();

        match &mut *node {
            NodeContext::Ros(node) => {
                for param in node.param.values_mut() {
                    param.eval(lua)?;
                }
            }
            NodeContext::Proc(_node) => {}
        }
    }

    Ok(())
}

fn eval_link_map(lua: &Lua, link_map: &mut IndexMap<LinkIdent, LinkArc>) -> Result<(), Error> {
    for (_key, link) in link_map {
        let mut link = link.write();

        match &mut *link {
            LinkContext::Pubsub(link) => {
                for uri in link.src.as_mut().unwrap() {
                    uri.topic.eval(lua)?;
                }
                for uri in link.dst.as_mut().unwrap() {
                    uri.topic.eval(lua)?;
                }
            }
            LinkContext::Service(link) => {
                link.listen.as_mut().unwrap().topic.eval(lua)?;
                for uri in link.connect.as_mut().unwrap() {
                    uri.topic.eval(lua)?;
                }
            }
        }
    }

    Ok(())
}

fn eval_socket_map(
    lua: &Lua,
    socket_map: &mut IndexMap<SocketIdent, SocketArc>,
) -> Result<(), Error> {
    for (_key, socket) in socket_map {
        let mut socket = socket.write();
        match &mut *socket {
            SocketContext::Pub(socket) => {
                for uri in socket.src.as_mut().unwrap() {
                    uri.topic.eval(lua)?;
                }
            }
            SocketContext::Sub(socket) => {
                for uri in socket.dst.as_mut().unwrap() {
                    uri.topic.eval(lua)?;
                }
            }
            SocketContext::Srv(socket) => {
                socket.listen.as_mut().unwrap().topic.eval(lua)?;
            }
            SocketContext::Qry(socket) => {
                for uri in socket.connect.as_mut().unwrap() {
                    uri.topic.eval(lua)?;
                }
            }
        }
    }

    Ok(())
}

fn eval_root_arg_table(arg_table: &mut IndexMap<ParamName, ArgContext>) -> Result<(), Error> {
    for (name, arg) in arg_table {
        assert!(
            arg.assign.is_none(),
            "arguments in the root scope must not be assigned"
        );
        let value = match (&arg.override_, &arg.default) {
            (Some(override_), _) => override_
                .as_value()
                .expect("the argument in the root scope must be assigned with constant values"),
            (None, Some(default)) => default,
            (None, None) => return Err(Error::RequiredArgumentNotAssigned { name: name.clone() }),
        };
        arg.result = Some(value.clone());
    }
    Ok(())
}

fn eval_arg_table(lua: &Lua, arg_table: &mut IndexMap<ParamName, ArgContext>) -> Result<(), Error> {
    for (name, arg) in arg_table {
        eval_arg_context(lua, name, arg)?;
    }
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

fn eval_expr(lua: &Lua, code: &Expr) -> Result<Value, Error> {
    let ValueFromLua(value): ValueFromLua = lua.load(code.as_str()).eval()?;
    Ok(value)
}

pub fn eval_value_or_expr(lua: &Lua, field: &ValueOrExpr) -> Result<Value, Error> {
    let value = match field {
        ValueOrExpr::Value(value) => value.clone(),
        ValueOrExpr::Expr { eval } => eval_expr(lua, eval)?,
    };
    Ok(value)
}

pub fn eval_arg_context(lua: &Lua, name: &ParamName, arg: &mut ArgContext) -> Result<(), Error> {
    let ArgContext {
        ty,
        default,
        assign,
        override_,
        result,
        ..
    } = arg;

    let value = if let Some(override_) = override_ {
        eval_value_or_expr(lua, override_)?
    } else if let Some(assign) = assign {
        eval_value_or_expr(lua, assign)?
    } else if let Some(default) = default {
        default.clone()
    } else {
        return Err(Error::RequiredArgumentNotAssigned { name: name.clone() });
    };

    if value.ty() != *ty {
        return Err(Error::TypeMismatch {
            expect: *ty,
            found: value.ty(),
        });
    }

    *result = Some(value);
    Ok(())
}
