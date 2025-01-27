use crate::{error::Error, eval::Eval};
use mlua::prelude::*;
use ros_plan_format::expr::{BoolExpr, KeyOrExpr, TextOrExpr, ValueOrExpr};
use serde::{Deserialize, Serialize};

pub type TextStore = EvalStore<TextOrExpr>;
pub type KeyStore = EvalStore<KeyOrExpr>;
pub type ValueStore = EvalStore<ValueOrExpr>;
pub type BoolStore = EvalStore<BoolExpr>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EvalStore<D>
where
    D: Eval,
    D::Output: Clone,
{
    evaluable: D,

    #[serde(skip)]
    override_: Option<D::Output>,

    #[serde(skip)]
    result: Option<D::Output>,
}

impl<D> EvalStore<D>
where
    D: Eval,
    D::Output: Clone,
{
    pub fn new(default: D) -> Self {
        Self {
            evaluable: default,
            override_: None,
            result: None,
        }
    }

    pub fn eval_and_store(&mut self, lua: &Lua) -> Result<&D::Output, Error> {
        let Self {
            evaluable: ref default,
            ref override_,
            result,
        } = self;

        let value = if let Some(override_) = override_ {
            (*override_).clone()
        } else {
            default.eval(lua)?
        };
        let output = result.insert(value);
        Ok(output)
    }

    pub fn set_override(&mut self, value: D::Output) {
        self.override_ = Some(value);
        self.result = None;
    }

    pub fn unset_override(&mut self) {
        self.override_ = None;
        self.result = None;
    }

    pub fn get_stored(&self) -> Result<&D::Output, Error> {
        self.result
            .as_ref()
            .ok_or(Error::ReferredBeforeEvaluationError)
    }
}
