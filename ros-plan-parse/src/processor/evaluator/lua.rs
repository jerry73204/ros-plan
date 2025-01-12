use crate::{error::Error, utils::find_pkg_dir};
use itertools::Itertools;
use mlua::prelude::*;
use ros_plan_format::expr::{Value, ValueType};

const TYPE_KEY: &str = "type";

pub struct ValueToLua<'a>(pub &'a Value);

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

pub struct ValueFromLua(pub Value);

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

pub fn add_function<A, R, F>(lua: &Lua, fn_name: &str, f: F) -> LuaResult<()>
where
    A: FromLuaMulti,
    R: IntoLuaMulti,
    F: Fn(&Lua, A) -> LuaResult<R> + mlua::MaybeSend + 'static,
{
    let func = lua.create_function(f)?;
    lua.globals().set(fn_name, func)?;
    Ok(())
}

pub fn new_lua() -> Result<Lua, Error> {
    // Create a Lua interpreter
    let lua = Lua::new();
    lua.sandbox(true)?;

    // Add global functions
    add_function(&lua, "pkg_dir", |_, pkg: String| {
        find_pkg_dir(&pkg).map_err(|err| LuaError::external(format!("{err}")))
    })?;

    Ok(lua)
}
