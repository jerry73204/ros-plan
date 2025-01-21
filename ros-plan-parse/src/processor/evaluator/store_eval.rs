use std::collections::BTreeMap;

use super::eval::Eval;
use crate::{
    context::{arg::ArgContext, expr::ExprContext, node::NodeContext},
    error::Error,
    scope::{NodeShared, Scope, ScopeTreeRef},
};
use indexmap::IndexMap;
use mlua::prelude::*;
use ros_plan_format::{key::KeyOwned, node::NodeIdent, parameter::ParamName};

pub trait StoreEval {
    fn store_eval(&mut self, lua: &Lua) -> Result<(), Error>;
}

impl StoreEval for ExprContext {
    fn store_eval(&mut self, lua: &Lua) -> Result<(), Error> {
        let ExprContext {
            ref default,
            ref override_,
            result,
        } = self;

        let value = if let Some(override_) = override_ {
            override_.clone()
        } else {
            default.eval(lua)?
        };
        *result = Some(value);
        Ok(())
    }
}

impl StoreEval for NodeContext {
    fn store_eval(&mut self, lua: &Lua) -> Result<(), Error> {
        for param in self.param.values_mut() {
            param.store_eval(lua)?;
        }

        Ok(())
    }
}

pub fn store_eval_node_map(
    lua: &Lua,
    node_map: &mut IndexMap<NodeIdent, NodeShared>,
) -> Result<(), Error> {
    for (_key, shared) in node_map {
        let owned = shared.upgrade().unwrap();
        let mut guard = owned.write();
        guard.store_eval(lua)?;
    }

    Ok(())
}

// pub fn store_eval_link_map(
//     lua: &Lua,
//     link_map: &mut IndexMap<LinkIdent, LinkShared>,
// ) -> Result<(), Error> {
//     for (_key, shared) in link_map {
//         let owned = shared.upgrade().unwrap();
//         let mut guard = owned.write();
//         guard.store_eval(lua)?;
//     }

//     Ok(())
// }

// pub fn store_eval_socket_map(
//     lua: &Lua,
//     socket_map: &mut IndexMap<SocketIdent, SocketShared>,
// ) -> Result<(), Error> {
//     for (_key, shared) in socket_map {
//         let owned = shared.upgrade().unwrap();
//         let mut guard = owned.write();
//         guard.store_eval(lua)?;
//     }

//     Ok(())
// }

pub fn store_eval_root_arg_table(
    arg_table: &mut IndexMap<ParamName, ArgContext>,
) -> Result<(), Error> {
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

pub fn store_eval_arg_table(
    lua: &Lua,
    arg_table: &mut IndexMap<ParamName, ArgContext>,
) -> Result<(), Error> {
    for (name, arg) in arg_table {
        store_eval_arg_context(lua, name, arg)?;
    }
    Ok(())
}

pub fn store_eval_subplan_table(
    lua: &Lua,
    children: &BTreeMap<KeyOwned, ScopeTreeRef>,
) -> Result<(), Error> {
    for child in children.values() {
        let mut child = child.write();
        match &mut child.value {
            Scope::PlanFile(scope) => {
                if let Some(when) = &mut scope.when {
                    when.store_eval(lua)?;

                    if !when.result.as_ref().unwrap().is_bool() {
                        return Err(Error::EvaluationError {
                            error: "cannot evaluate to a boolean value".to_string(),
                        });
                    };
                };
                store_eval_arg_table(lua, &mut scope.arg_map)?;
            }
            Scope::Group(scope) => {
                if let Some(when) = &mut scope.when {
                    when.store_eval(lua)?;
                    if !when.result.as_ref().unwrap().is_bool() {
                        return Err(Error::EvaluationError {
                            error: "cannot evaluate to a boolean value".to_string(),
                        });
                    };
                };
            }
        }
    }

    Ok(())
}

pub fn store_eval_arg_context(
    lua: &Lua,
    name: &ParamName,
    arg_ctx: &mut ArgContext,
) -> Result<(), Error> {
    let ArgContext {
        ty,
        default,
        assign,
        override_,
        result,
        ..
    } = arg_ctx;

    let value = if let Some(override_) = override_ {
        override_.eval(lua)?
    } else if let Some(assign) = assign {
        assign.eval(lua)?
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

    *result = Some(value.clone());
    Ok(())
}
