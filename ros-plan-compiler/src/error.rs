use ros_plan_format::{expr::ValueType, key::KeyOwned, parameter::ParamName};
use ros_utils::PackageResolutionError;
use std::{io, path::PathBuf};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("unable to read file `{path}`: {error}")]
    ReadFileError { path: PathBuf, error: io::Error },

    #[error("unable to write file `{path}`: {error}")]
    WriteFileError { path: PathBuf, error: io::Error },

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

    #[error("{0}")]
    PackageResolutionError(#[from] PackageResolutionError),

    #[error("unable to resolve plan file `{file}` in package `{pkg}`: {message}")]
    PlanFileResolutionError {
        pkg: String,
        file: String,
        message: String,
    },

    #[error("unable to resolve key `{key}`: {reason}")]
    KeyResolutionError { key: KeyOwned, reason: String },

    #[error("link `{link}` has {source_count} sources but no explicit 'topic' attribute\n  Links with multiple sources must specify a topic name.\n\n  Suggestion: Add a topic field like:\n    link:\n      {link}: !pubsub\n        topic: /shared_topic  # Add this\n        src: [...]\n        dst: [...]")]
    MultipleSourcesRequireExplicitTopic { link: KeyOwned, source_count: usize },

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

    #[error("the value is referred before evaluation")]
    ReferredBeforeEvaluationError,

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
