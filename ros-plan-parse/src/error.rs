use ros_plan_format::{eval::ValueType, key::KeyOwned, parameter::ParamName};
use std::{io, path::PathBuf};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("unable to open plan file `{path}`: {error}")]
    OpenPlanFileError { path: PathBuf, error: io::Error },

    #[error("unable to parse plan file `{path}`: {error}")]
    ParsePlanFileError {
        path: PathBuf,
        error: toml::de::Error,
    },

    #[error("repeated publication or subscription key {0}")]
    RepeatedPubSubKey(KeyOwned),

    #[error("repeated service or query key {0}")]
    RepeatedSrvQryKey(KeyOwned),

    #[error("the variable `{0}` is already defined")]
    MultipleDefinitionOfVariable(ParamName),

    #[error("unable to insert the key `{new}` because it conflicts with `{old}`")]
    ConflictingKeys { old: KeyOwned, new: KeyOwned },

    #[error("unable to insert a key with absolute path `{key}`")]
    AbsoluteKeyNotAllowed { key: KeyOwned },

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
    ArgumentNotAssigned { name: ParamName },

    #[error("expect `{expect}` type for argument `{name}`, but found `{found}` type")]
    ArgumentTypeMismatch {
        name: ParamName,
        expect: ValueType,
        found: ValueType,
    },

    #[error("evaluation error: {error:?}")]
    EvaluationError { error: String },
}

impl From<mlua::Error> for Error {
    fn from(error: mlua::Error) -> Self {
        Error::EvaluationError {
            error: format!("{error}"),
        }
    }
}
