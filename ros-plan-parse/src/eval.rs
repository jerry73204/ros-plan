use crate::{error::Error, utils::find_pkg_dir};
use indexmap::IndexMap;
use itertools::Itertools;
use mlua::prelude::*;
use ros_plan_format::{
    eval::{Eval, Value, ValueOrEval, ValueType},
    link::Link,
    node::Node,
    parameter::{ArgEntry, ParamName},
    socket::Socket,
    subplan::Subplan,
    Plan,
};

pub fn eval_plan(plan: &mut Plan, args: IndexMap<ParamName, Value>) -> Result<(), Error> {
    // Create a Lua interpreter
    let lua = Lua::new();
    lua.sandbox(true)?;

    // Add global functions
    add_function(&lua, "pkg_dir", |_, pkg: String| {
        find_pkg_dir(&pkg).map_err(|err| LuaError::external(format!("{err}")))
    })?;

    // Check multiple definition of arguments and variables.
    for name in plan.var.keys() {
        if plan.arg.contains_key(name) {
            return Err(Error::MultipleDefinitionOfVariable(name.clone()));
        }
    }

    // Populate argument into global scope
    load_arg_table(&lua, &plan.arg, &args)?;

    // Evaluate local variables and populate them to the global scope
    for (name, entry) in &mut plan.var {
        // Lock the global variables during evaluation
        lua.globals().set_readonly(true);
        may_eval_value(&lua, entry)?;
        lua.globals().set_readonly(false);
        lua.globals()
            .set(name.as_ref(), ValueToLua(entry.as_value().unwrap()))?;
    }

    // Lock the global variables.
    lua.globals().set_readonly(true);

    // Evaluate the node table
    for (_key, node) in &mut plan.node.0 {
        match node {
            Node::Ros(node) => {
                for param in node.param.values_mut() {
                    may_eval_value(&lua, param)?;
                }
            }
            Node::Proc(_node) => {}
        }
    }

    // Evaluate the link table
    for (_key, link) in &mut plan.link.0 {
        match link {
            Link::Pubsub(link) => {
                for uri in &mut link.src {
                    may_eval_value(&lua, &mut uri.topic)?;
                }
                for uri in &mut link.dst {
                    may_eval_value(&lua, &mut uri.topic)?;
                }
            }
            Link::Service(link) => {
                may_eval_value(&lua, &mut link.listen.topic)?;
                for uri in &mut link.connect {
                    may_eval_value(&lua, &mut uri.topic)?;
                }
            }
        }
    }

    // Evaluate the socket table
    for (_key, socket) in &mut plan.socket.0 {
        match socket {
            Socket::Pub(socket) => {
                for uri in &mut socket.src {
                    may_eval_value(&lua, &mut uri.topic)?;
                }
            }
            Socket::Sub(socket) => {
                for uri in &mut socket.dst {
                    may_eval_value(&lua, &mut uri.topic)?;
                }
            }
            Socket::Srv(socket) => {
                may_eval_value(&lua, &mut socket.listen.topic)?;
            }
            Socket::Qry(socket) => {
                for uri in &mut socket.connect {
                    may_eval_value(&lua, &mut uri.topic)?;
                }
            }
        }
    }

    // Evaluate the subplan table
    for (_key, subplan) in &mut plan.subplan.0 {
        match subplan {
            Subplan::File(subplan) => {
                if let Some(when) = &mut subplan.when {
                    may_eval_bool(&lua, when)?;
                }
                for entry in subplan.arg.values_mut() {
                    may_eval_value(&lua, entry)?;
                }
            }
            Subplan::Pkg(subplan) => {
                if let Some(when) = &mut subplan.when {
                    may_eval_bool(&lua, when)?;
                }
                for entry in subplan.arg.values_mut() {
                    may_eval_value(&lua, entry)?;
                }
            }
            Subplan::Here(subplan) => {
                if let Some(when) = &mut subplan.when {
                    may_eval_bool(&lua, when)?;
                }
            }
        }
    }

    Ok(())
}

fn load_arg_table(
    lua: &Lua,
    arg_table: &IndexMap<ParamName, ArgEntry>,
    assigned: &IndexMap<ParamName, Value>,
) -> Result<(), Error> {
    for (name, entry) in arg_table {
        // Get the assigned value if any
        let asigned_value = assigned.get(name);

        // Use the assigned or default value if provided
        let value = match entry {
            ArgEntry::Required { ty, .. } => {
                // Check if the value is assigned for the required
                // argument.
                let Some(assigned_value) = asigned_value else {
                    return Err(Error::ArgumentNotAssigned { name: name.clone() });
                };

                // Check if the type is consistent.
                if assigned_value.ty() != *ty {
                    return Err(Error::ArgumentTypeMismatch {
                        name: name.clone(),
                        expect: *ty,
                        found: assigned_value.ty(),
                    });
                }

                assigned_value
            }
            ArgEntry::Optional { default, .. } => match asigned_value {
                Some(assigned_value) => {
                    // If the value is assigned, check whether the
                    // type is consistent with the default value.
                    if assigned_value.ty() != default.ty() {
                        return Err(Error::ArgumentTypeMismatch {
                            name: name.clone(),
                            expect: default.ty(),
                            found: assigned_value.ty(),
                        });
                    }
                    assigned_value
                }
                None => default,
            },
        };

        // Insert to the global variable table.
        lua.globals().set(name.as_ref(), ValueToLua(value))?;
    }

    Ok(())
}

fn eval_value(lua: &Lua, code: &Eval) -> Result<Value, Error> {
    let ValueFromLua(value) = lua.load(code.as_str()).eval()?;
    Ok(value)
}

fn may_eval_value(lua: &Lua, field: &mut ValueOrEval) -> Result<(), Error> {
    if let ValueOrEval::Eval { eval } = field {
        *field = eval_value(lua, eval)?.into();
    }
    Ok(())
}

fn may_eval_bool(lua: &Lua, field: &mut ValueOrEval) -> Result<bool, Error> {
    let yes = match field {
        ValueOrEval::Eval { eval } => {
            let value = eval_value(lua, eval)?;
            let Some(yes) = value.to_bool() else {
                return Err(Error::EvaluationError {
                    error: format!("`{eval}` field does not evaluate to a boolean value"),
                });
            };
            *field = value.into();
            yes
        }
        ValueOrEval::Value(value) => {
            let Some(yes) = value.to_bool() else {
                return Err(Error::EvaluationError {
                    error: format!(
                        "expect a boolean value, but a {} value is provided",
                        value.ty()
                    ),
                });
            };
            yes
        }
    };

    Ok(yes)
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
