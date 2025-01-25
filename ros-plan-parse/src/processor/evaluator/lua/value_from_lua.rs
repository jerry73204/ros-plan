use crate::error::Error;
use itertools::Itertools;
use mlua::prelude::*;
use ros_plan_format::{
    expr::{Value, ValueType},
    key::KeyOwned,
};

pub enum ValueFromLua {
    Bool(bool),
    I64(i64),
    F64(f64),
    String(String),
    EmptyList,
    BoolList(Vec<bool>),
    I64List(Vec<i64>),
    F64List(Vec<f64>),
    StringList(Vec<String>),
    Buffer(Vec<u8>),
}

impl From<Vec<u8>> for ValueFromLua {
    fn from(v: Vec<u8>) -> Self {
        Self::Buffer(v)
    }
}

impl From<Vec<String>> for ValueFromLua {
    fn from(v: Vec<String>) -> Self {
        Self::StringList(v)
    }
}

impl From<Vec<f64>> for ValueFromLua {
    fn from(v: Vec<f64>) -> Self {
        Self::F64List(v)
    }
}

impl From<Vec<i64>> for ValueFromLua {
    fn from(v: Vec<i64>) -> Self {
        Self::I64List(v)
    }
}

impl From<Vec<bool>> for ValueFromLua {
    fn from(v: Vec<bool>) -> Self {
        Self::BoolList(v)
    }
}

impl From<String> for ValueFromLua {
    fn from(v: String) -> Self {
        Self::String(v)
    }
}

impl From<f64> for ValueFromLua {
    fn from(v: f64) -> Self {
        Self::F64(v)
    }
}

impl From<i64> for ValueFromLua {
    fn from(v: i64) -> Self {
        Self::I64(v)
    }
}

impl From<bool> for ValueFromLua {
    fn from(v: bool) -> Self {
        Self::Bool(v)
    }
}

impl FromLua for ValueFromLua {
    fn from_lua(value: LuaValue, _lua: &Lua) -> LuaResult<Self> {
        let value: Self = match value {
            LuaValue::Boolean(val) => val.into(),
            LuaValue::Integer(val) => (val as i64).into(),
            LuaValue::Number(val) => val.into(),
            LuaValue::Buffer(buffer) => buffer.to_vec().into(),
            LuaValue::String(string) => string.to_str()?.to_string().into(),
            LuaValue::Table(table) => {
                let len = table.len()? as usize;

                if len == 0 {
                    Self::EmptyList
                } else {
                    let ty = {
                        let first: LuaValue = table.get(1)?;
                        match first {
                            LuaValue::Boolean(_) => ValueType::Bool,
                            LuaValue::Integer(_) => ValueType::I64,
                            LuaValue::Number(_) => ValueType::F64,
                            LuaValue::String(_) => ValueType::String,
                            LuaNil => {
                                return Err(LuaError::external(format!(
                                    "unable to guess element type of an empty array"
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

                    macro_rules! convert {
                        ($variant:ident) => {{
                            let array: Vec<_> = (1..=len)
                                .map(|ix| {
                                    if !table.contains_key(ix)? {
                                        return Err(LuaError::external(format!(
                                            "the element at index {ix} is missing"
                                        )));
                                    }
                                    let val: ValueFromLua = table.get(ix)?;
                                    let ValueFromLua::$variant(val) = val else {
                                        return Err(LuaError::external(
                                            "inconsistent array element type",
                                        ));
                                    };
                                    Ok(val)
                                })
                                .try_collect()?;
                            let array: Self = array.into();
                            array
                        }};
                    }

                    let array: Self = match ty {
                        ValueType::Bool => convert!(Bool),
                        ValueType::I64 => convert!(I64),
                        ValueType::F64 => convert!(F64),
                        ValueType::String => convert!(String),
                        _ => return Err(LuaError::external(format!("unknown type `{ty}`"))),
                    };
                    array
                }
            }
            _ => {
                return Err(LuaError::external(format!(
                    "evaluation to `{}` type is not supported",
                    value.type_name()
                )));
            }
        };

        Ok(value)
    }
}

impl ValueFromLua {
    pub fn into_ros_value(self, ty: ValueType) -> Result<Value, Error> {
        let value: Value = match (ty, self) {
            (ValueType::Bool, Self::Bool(val)) => val.into(),
            (ValueType::I64, Self::I64(val)) => val.into(),
            (ValueType::F64, Self::F64(val)) => val.into(),
            (ValueType::String, Self::String(val)) => val.into(),
            (ValueType::Key, Self::String(val)) => {
                let key: KeyOwned = val.parse().map_err(|_| Error::InvalidKey { key: val })?;
                key.into()
            }
            (ValueType::BoolList, Self::BoolList(vec)) => vec.into(),
            (ValueType::I64List, Self::I64List(vec)) => vec.into(),
            (ValueType::F64List, Self::F64List(vec)) => vec.into(),
            (ValueType::StringList, Self::StringList(vec)) => vec.into(),
            (ValueType::Binary, Self::Buffer(vec)) => vec.into(),
            (ValueType::BoolList, Self::EmptyList) => Value::BoolList(vec![]),
            (ValueType::I64List, Self::EmptyList) => Value::I64List(vec![]),
            (ValueType::F64List, Self::EmptyList) => Value::F64List(vec![]),
            (ValueType::StringList, Self::EmptyList) => Value::StringList(vec![]),
            _ => return Err(LuaError::external("unable to convert Lua value to ROS value").into()),
        };
        Ok(value)
    }
}
