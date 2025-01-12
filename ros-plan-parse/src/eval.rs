use crate::{
    context::{
        arg::ArgContext,
        link::{LinkArc, LinkContext},
        node::{NodeArc, NodeContext},
        socket::{SocketArc, SocketContext},
    },
    error::Error,
    resource::{Resource, ResourceTreeRef, Scope, ScopeKind},
    utils::find_pkg_dir,
};
use indexmap::IndexMap;
use itertools::Itertools;
use mlua::prelude::*;
use ros_plan_format::{
    expr::{Expr, Value, ValueOrExpr, ValueType},
    link::LinkIdent,
    node::NodeIdent,
    parameter::ParamName,
    socket::SocketIdent,
};
use serde::Serialize;
use std::{collections::VecDeque, sync::Arc};

#[derive(Debug, Clone, Serialize)]
pub struct EvalSlot {
    pub default: ValueOrExpr,
    pub override_: Option<Value>,
    pub result: Option<Value>,
}

impl EvalSlot {
    pub fn new(default: ValueOrExpr) -> Self {
        Self {
            default,
            override_: None,
            result: None,
        }
    }

    pub fn eval(&mut self, lua: &Lua) -> Result<(), Error> {
        let Self {
            ref default,
            ref override_,
            result,
        } = self;

        let value = if let Some(override_) = override_ {
            override_.clone()
        } else {
            may_eval_value(lua, default)?
        };
        *result = Some(value);
        Ok(())
    }

    pub fn eval_bool(&mut self, lua: &Lua) -> Result<(), Error> {
        self.eval(lua)?;

        let ty = self.result.as_ref().unwrap().ty();
        if ty != ValueType::Bool {
            return Err(Error::EvaluationError {
                error: format!("`{}` does not evaluate to a boolean value", self.default),
            });
        }

        Ok(())
    }

    pub fn ty(&self) -> Option<ValueType> {
        Some(self.result.as_ref()?.ty())
    }
}

pub struct Evaluator {
    queue: VecDeque<Job>,
}

impl Evaluator {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

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
                Job::Group { current, lua } => self.eval_group(lua, current)?,
            }
        }

        Ok(())
    }

    fn eval_plan_file(&mut self, current: &ResourceTreeRef) -> Result<(), Error> {
        let lua = Arc::new(new_lua()?);

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

    fn eval_group(&mut self, lua: Arc<Lua>, current: ResourceTreeRef) -> Result<(), Error> {
        {
            let mut group = current.as_group_mut().unwrap();
            eval_node_map(&lua, &mut group.node_map)?;
            eval_link_map(&lua, &mut group.link_map)?;
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

enum Job {
    PlanFile {
        current: ResourceTreeRef,
    },
    Group {
        current: ResourceTreeRef,
        lua: Arc<Lua>,
    },
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
        eval_arg(lua, name, arg)?;
    }
    Ok(())
}

fn load_arg_table(lua: &Lua, arg_table: &IndexMap<ParamName, ArgContext>) -> Result<(), Error> {
    for (name, arg) in arg_table {
        lua.globals()
            .set(name.as_ref(), ValueToLua(arg.result.as_ref().unwrap()))?;
    }

    Ok(())
}

fn override_arg_table(
    arg_table: &mut IndexMap<ParamName, ArgContext>,
    override_arg: IndexMap<ParamName, Value>,
) -> Result<(), Error> {
    for (name, override_) in override_arg {
        let Some(arg_ctx) = arg_table.get_mut(&name) else {
            return Err(Error::ArgumentNotFound { name });
        };
        arg_ctx.override_ = Some(override_.into());
    }

    Ok(())
}

fn eval_value(lua: &Lua, code: &Expr) -> Result<Value, Error> {
    let ValueFromLua(value) = lua.load(code.as_str()).eval()?;
    Ok(value)
}

fn may_eval_value(lua: &Lua, field: &ValueOrExpr) -> Result<Value, Error> {
    let value = match field {
        ValueOrExpr::Value(value) => value.clone(),
        ValueOrExpr::Expr { eval } => eval_value(lua, eval)?,
    };
    Ok(value)
}

const TYPE_KEY: &str = "type";

struct ValueToLua<'a>(&'a Value);

impl IntoLua for ValueToLua<'_> {
    fn into_lua(self, lua: &Lua) -> LuaResult<LuaValue> {
        fn convert_array<T: IntoLua + Clone>(
            lua: &Lua,
            vec: &[T],
            ty: ValueType,
        ) -> LuaResult<LuaValue> {
            let table = lua.create_sequence_from(vec.iter().cloned())?;
            if table.is_empty() {
                table.set(TYPE_KEY, ty.to_string())?;
            }
            lua.convert(table)
        }

        match self.0 {
            Value::Bool(val) => val.into_lua(lua),
            Value::Integer(val) => val.into_lua(lua),
            Value::Double(val) => val.into_lua(lua),
            Value::String(val) => val.as_str().into_lua(lua),
            Value::ByteArray { bytes } => {
                let buffer = lua.create_buffer(bytes.as_slice())?;
                lua.convert(buffer)
            }
            // Arrays are converted to Lua tables with an optional "_type" field.
            Value::BoolArray(vec) => convert_array(lua, vec, ValueType::Bool),
            Value::IntegerArray(vec) => convert_array(lua, vec, ValueType::Integer),
            Value::DoubleArray(vec) => convert_array(lua, vec, ValueType::Double),
            Value::StringArray(vec) => convert_array(lua, vec, ValueType::String),
        }
    }
}

pub fn eval_arg(lua: &Lua, name: &ParamName, arg: &mut ArgContext) -> Result<(), Error> {
    let ArgContext {
        ty,
        default,
        assign,
        override_,
        result,
        ..
    } = arg;

    let value = if let Some(override_) = override_ {
        may_eval_value(lua, override_)?
    } else if let Some(assign) = assign {
        may_eval_value(lua, assign)?
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

struct ValueFromLua(Value);

impl FromLua for ValueFromLua {
    fn from_lua(value: LuaValue, _lua: &Lua) -> LuaResult<Self> {
        let param = match value {
            LuaValue::Boolean(val) => val.into(),
            LuaValue::Integer(val) => (val as i64).into(),
            LuaValue::Number(val) => val.into(),
            LuaValue::String(string) => string.to_str()?.to_string().into(),
            LuaValue::Buffer(buffer) => buffer.to_vec().into(),
            LuaValue::Table(table) => {
                // Guess the element type from the `_type` key in the
                // table. If it does not exist, guess from the first element.
                let ty = if table.contains_key(TYPE_KEY)? {
                    let ty_name: String = table.get(TYPE_KEY)?;
                    let ty: ValueType = ty_name
                        .parse()
                        .map_err(|_| LuaError::external(format!("unknown type `{ty_name}`")))?;
                    ty
                } else {
                    let first: LuaValue = table.get(1)?;
                    match first {
                        LuaValue::Boolean(_) => ValueType::Bool,
                        LuaValue::Integer(_) => ValueType::Integer,
                        LuaValue::Number(_) => ValueType::Double,
                        LuaValue::String(_) => ValueType::String,
                        LuaNil => {
                            return Err(LuaError::external(format!(
                                "unable to guess element type of \
                                 an empty array without `{TYPE_KEY}` key"
                            )));
                        }
                        _ => {
                            return Err(LuaError::external(format!(
                                "array element with type {} is not supported",
                                first.type_name()
                            )));
                        }
                    }
                };
                let len = table.len()? as usize;

                macro_rules! convert {
                    ($variant:ident) => {{
                        let array: Vec<_> = (1..=len)
                            .map(|ix| {
                                if !table.contains_key(ix)? {
                                    return Err(LuaError::external(format!(
                                        "the element at index {ix} is missing"
                                    )));
                                }
                                let ValueFromLua(val) = table.get(ix)?;
                                let Value::$variant(val) = val else {
                                    return Err(LuaError::external(
                                        "inconsistent array element type",
                                    ));
                                };
                                Ok(val)
                            })
                            .try_collect()?;
                        let array: Value = array.into();
                        array
                    }};
                }

                let array: Value = match ty {
                    ValueType::Bool => convert!(Bool),
                    ValueType::Integer => convert!(Integer),
                    ValueType::Double => convert!(Double),
                    ValueType::String => convert!(String),
                    _ => return Err(LuaError::external(format!("unknown type `{ty}`"))),
                };
                array
            }
            _ => {
                return Err(LuaError::external(format!(
                    "evaluation to `{}` type is not supported",
                    value.type_name()
                )));
            }
        };
        Ok(Self(param))
    }
}

fn add_function<A, R, F>(lua: &Lua, fn_name: &str, f: F) -> LuaResult<()>
where
    A: FromLuaMulti,
    R: IntoLuaMulti,
    F: Fn(&Lua, A) -> LuaResult<R> + mlua::MaybeSend + 'static,
{
    let func = lua.create_function(f)?;
    lua.globals().set(fn_name, func)?;
    Ok(())
}

fn new_lua() -> Result<Lua, Error> {
    // Create a Lua interpreter
    let lua = Lua::new();
    lua.sandbox(true)?;

    // Add global functions
    add_function(&lua, "pkg_dir", |_, pkg: String| {
        find_pkg_dir(&pkg).map_err(|err| LuaError::external(format!("{err}")))
    })?;

    Ok(lua)
}
