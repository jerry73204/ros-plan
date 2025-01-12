use crate::{error::Error, processor::evaluator::eval_value_or_expr};
use mlua::prelude::*;
use ros_plan_format::expr::{Value, ValueOrExpr, ValueType};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct ExprContext {
    pub default: ValueOrExpr,
    pub override_: Option<Value>,
    pub result: Option<Value>,
}

impl ExprContext {
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
            eval_value_or_expr(lua, default)?
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
