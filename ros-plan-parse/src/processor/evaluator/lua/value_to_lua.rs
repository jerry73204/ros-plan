use mlua::prelude::*;
use ros_plan_format::expr::Value;

pub struct ValueToLua<'a>(pub &'a Value);

impl IntoLua for ValueToLua<'_> {
    fn into_lua(self, lua: &Lua) -> LuaResult<LuaValue> {
        fn convert_array<T: IntoLua + Clone>(lua: &Lua, vec: &[T]) -> LuaResult<LuaValue> {
            let table = lua.create_sequence_from(vec.iter().cloned())?;
            lua.convert(table)
        }

        match self.0 {
            Value::Bool(val) => val.into_lua(lua),
            Value::I64(val) => val.into_lua(lua),
            Value::F64(val) => val.into_lua(lua),
            Value::String(val) => val.as_str().into_lua(lua),
            Value::Key(val) => val.as_str().into_lua(lua),
            Value::Binary(bytes) => {
                let buffer = lua.create_buffer(bytes.as_slice())?;
                lua.convert(buffer)
            }
            // Arrays are converted to Lua tables with an optional "_type" field.
            Value::BoolList(vec) => convert_array(lua, vec),
            Value::I64List(vec) => convert_array(lua, vec),
            Value::F64List(vec) => convert_array(lua, vec),
            Value::StringList(vec) => convert_array(lua, vec),
        }
    }
}
