use super::lua::ValueFromLua;
use crate::error::Error;
use mlua::prelude::*;
use ros_plan_format::expr::{Expr, Value, ValueOrExpr};

pub trait Eval {
    fn eval(&self, lua: &Lua) -> Result<Value, Error>;
}

impl Eval for Expr {
    fn eval(&self, lua: &Lua) -> Result<Value, Error> {
        let ValueFromLua(value): ValueFromLua = lua.load(self.as_str()).eval()?;
        Ok(value)
    }
}

impl Eval for ValueOrExpr {
    fn eval(&self, lua: &Lua) -> Result<Value, Error> {
        let value = match self {
            ValueOrExpr::Value(value) => value.clone(),
            ValueOrExpr::Expr { eval: expr } => expr.eval(lua)?,
        };
        Ok(value)
    }
}
