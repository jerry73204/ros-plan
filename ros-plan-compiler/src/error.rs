use ros_plan_format::{expr::ValueType, key::KeyOwned, parameter::ParamName};
use std::{io, path::PathBuf};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("unable to open plan file `{path}`: {error}")]
    OpenPlanFileError { path: PathBuf, error: io::Error },

    #[error("unable to parse plan file `{path}`: {error}")]
    ParsePlanFileError {
        path: PathBuf,
        error: serde_yaml::Error,
    },

    #[error("repeated publication or subscription key {0}")]
    RepeatedPubSubKey(KeyOwned),

    #[error("repeated service or query key {0}")]
    RepeatedSrvQryKey(KeyOwned),

    #[error("the variable `{0}` is already defined")]
    MultipleDefinitionOfVariable(ParamName),

    #[error("unable to insert the key `{inserted}` because it conflicts with `{offender}`")]
    ConflictingKeys {
        offender: KeyOwned,
        inserted: KeyOwned,
    },

    #[error("invalid subplan name `{key}`: {reason}")]
    InvalidSubplanName { key: KeyOwned, reason: String },

    #[error("unable to resolve directory for package `{pkg}`: {message}")]
    PackageResolutionError { pkg: String, message: String },

    #[error("unable to resolve plan file `{file}` in package `{pkg}`: {message}")]
    PlanFileResolutionError {
        pkg: String,
        file: String,
        message: String,
    },

    #[error("unable to resolve key `{key}`: {reason}")]
    KeyResolutionError { key: KeyOwned, reason: String },

    #[error("the argument `{name}` is required but is not assigned")]
    RequiredArgumentNotAssigned { name: ParamName },

    #[error("expect `{expect}` type, but found `{found}` type")]
    TypeMismatch { expect: ValueType, found: ValueType },

    #[error("`{key}` is not a valid key")]
    InvalidKey { key: String },

    #[error("argument `{name}` is assigned but is not found")]
    ArgumentNotFound { name: ParamName },

    #[error("evaluation error: {error:?}")]
    EvaluationError { error: String },

    #[error("either a `path` or a pair of `pkg` and `path` is expected")]
    InvalidIncludePath,
}

impl From<mlua::Error> for Error {
    fn from(error: mlua::Error) -> Self {
        Error::EvaluationError {
            error: format!("{error}"),
        }
    }
}
