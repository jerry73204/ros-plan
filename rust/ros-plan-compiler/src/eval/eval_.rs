use super::lua::ValueFromLua;
use crate::error::Error;
use mlua::prelude::*;
use ros_plan_format::{
    expr::{BoolExpr, Expr, KeyOrExpr, TextOrExpr, Value, ValueOrExpr},
    key::KeyOwned,
};

pub trait Eval {
    type Output;

    fn eval(&self, lua: &Lua) -> Result<Self::Output, Error>;
}

impl Eval for Expr {
    type Output = ValueFromLua;

    fn eval(&self, lua: &Lua) -> Result<Self::Output, Error> {
        let value: ValueFromLua = lua.load(self.as_str()).eval()?;
        Ok(value)
    }
}

impl Eval for ValueOrExpr {
    type Output = Value;

    fn eval(&self, lua: &Lua) -> Result<Self::Output, Error> {
        let value = match self {
            ValueOrExpr::Value(value) => value.clone(),
            ValueOrExpr::Expr { ty, expr } => {
                let value = expr.eval(lua)?.into_ros_value(*ty)?;
                if *ty != value.ty() {
                    return Err(Error::TypeMismatch {
                        expect: *ty,
                        found: value.ty(),
                    });
                }

                value
            }
        };
        Ok(value)
    }
}

impl Eval for TextOrExpr {
    type Output = String;

    fn eval(&self, lua: &Lua) -> Result<Self::Output, Error> {
        let value = match self {
            TextOrExpr::Text(text) => text.clone(),
            TextOrExpr::Expr(expr) => {
                let value = expr.eval(lua)?;
                let ValueFromLua::String(text) = value else {
                    return Err(LuaError::external("unable to convert to a string").into());
                };
                text
            }
        };
        Ok(value)
    }
}

impl Eval for KeyOrExpr {
    type Output = KeyOwned;

    fn eval(&self, lua: &Lua) -> Result<Self::Output, Error> {
        let value = match self {
            KeyOrExpr::Key(key) => key.clone(),
            KeyOrExpr::Expr(expr) => {
                let value = expr.eval(lua)?;
                let ValueFromLua::String(text) = value else {
                    return Err(LuaError::external("unable to convert to string").into());
                };
                let Ok(key) = text.parse() else {
                    return Err(LuaError::external("unable to convert to a key").into());
                };
                key
            }
        };
        Ok(value)
    }
}

impl Eval for BoolExpr {
    type Output = bool;

    fn eval(&self, lua: &Lua) -> Result<Self::Output, Error> {
        let value: LuaValue = lua.load(self.as_str()).eval()?;
        let LuaValue::Boolean(yes) = value else {
            return Err(Error::EvaluationError {
                error: "expect a boolean value".to_string(),
            });
        };
        Ok(yes)
    }
}
