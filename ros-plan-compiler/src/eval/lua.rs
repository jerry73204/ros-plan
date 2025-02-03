mod value_from_lua;
mod value_to_lua;

pub use value_from_lua::ValueFromLua;
pub use value_to_lua::ValueToLua;

use crate::error::Error;
use mlua::prelude::*;
use ros_utils::find_pkg_dir;

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
