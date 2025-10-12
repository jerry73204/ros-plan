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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_lua_creates_sandboxed_instance() {
        let result = new_lua();
        assert!(result.is_ok());
    }

    #[test]
    fn lua_basic_arithmetic() {
        let lua = new_lua().unwrap();
        let result: i64 = lua.load("return 1 + 2").eval().unwrap();
        assert_eq!(result, 3);

        let result: f64 = lua.load("return 2.5 * 2").eval().unwrap();
        assert_eq!(result, 5.0);
    }

    #[test]
    fn lua_string_operations() {
        let lua = new_lua().unwrap();
        let result: String = lua.load("return 'hello' .. ' ' .. 'world'").eval().unwrap();
        assert_eq!(result, "hello world");
    }

    #[test]
    fn lua_boolean_logic() {
        let lua = new_lua().unwrap();
        let result: bool = lua.load("return true and false").eval().unwrap();
        assert!(!result);

        let result: bool = lua.load("return true or false").eval().unwrap();
        assert!(result);
    }

    #[test]
    fn lua_comparisons() {
        let lua = new_lua().unwrap();
        let result: bool = lua.load("return 10 > 5").eval().unwrap();
        assert!(result);

        let result: bool = lua.load("return 5 == 5").eval().unwrap();
        assert!(result);
    }

    #[test]
    fn lua_variables() {
        let lua = new_lua().unwrap();
        let result: i64 = lua
            .load("local x = 10\nlocal y = 20\nreturn x + y")
            .eval()
            .unwrap();
        assert_eq!(result, 30);
    }

    #[test]
    fn lua_tables() {
        let lua = new_lua().unwrap();
        let result: i64 = lua
            .load("local t = {1, 2, 3}\nreturn t[1] + t[2] + t[3]")
            .eval()
            .unwrap();
        assert_eq!(result, 6);
    }

    #[test]
    fn lua_functions() {
        let lua = new_lua().unwrap();
        let code = r#"
            local function add(a, b)
                return a + b
            end
            return add(5, 7)
        "#;
        let result: i64 = lua.load(code).eval().unwrap();
        assert_eq!(result, 12);
    }

    #[test]
    fn lua_conditionals() {
        let lua = new_lua().unwrap();
        let code = r#"
            local x = 10
            if x > 5 then
                return "greater"
            else
                return "lesser"
            end
        "#;
        let result: String = lua.load(code).eval().unwrap();
        assert_eq!(result, "greater");
    }

    #[test]
    fn lua_loops() {
        let lua = new_lua().unwrap();
        let code = r#"
            local sum = 0
            for i = 1, 5 do
                sum = sum + i
            end
            return sum
        "#;
        let result: i64 = lua.load(code).eval().unwrap();
        assert_eq!(result, 15);
    }

    #[test]
    fn lua_is_sandboxed() {
        let lua = new_lua().unwrap();
        // Sandboxed Lua should not have access to io, os, etc.
        let result = lua.load("return io").eval::<LuaValue>();
        assert!(result.is_ok());
        assert!(matches!(result.unwrap(), LuaValue::Nil));
    }

    #[test]
    fn add_function_creates_global() {
        let lua = Lua::new();
        let result = add_function(&lua, "test_fn", |_, ()| Ok(42i64));
        assert!(result.is_ok());

        let value: i64 = lua.load("return test_fn()").eval().unwrap();
        assert_eq!(value, 42);
    }

    #[test]
    fn add_function_with_arguments() {
        let lua = Lua::new();
        let result = add_function(&lua, "multiply", |_, (a, b): (i64, i64)| Ok(a * b));
        assert!(result.is_ok());

        let value: i64 = lua.load("return multiply(6, 7)").eval().unwrap();
        assert_eq!(value, 42);
    }
}
