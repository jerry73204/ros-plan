use crate::key::KeyOwned;

#[derive(Debug, thiserror::Error)]
#[error("cannot create a identifier from `{string}`")]
pub struct IdentifierCreationError {
    pub string: String,
}

#[derive(Debug, thiserror::Error)]
pub enum ParseArgDefError {
    #[error("invalid argument name `{name}`")]
    InvalidName { name: String },
    #[error("invalid argument type `{ty}`")]
    InvalidType { ty: String },
    #[error("unable to parse argument definition `{expr}`")]
    MalformedDecl { expr: String },
}

#[derive(Debug, thiserror::Error)]
pub enum InvalidArgumentDeclaration {
    #[error("argument value type mismatches")]
    ValueTypeMismatch,

    #[error("argument value type is not supported")]
    ValueTypeNotSupported,

    #[error("repeated argument definition `{name}`")]
    RepeatedDefinition { name: String },
}

#[derive(Debug, thiserror::Error)]
pub enum InvalidNodeDeclaration {
    #[error("repeated node definition `{name}`")]
    RepeatedDefinition { name: String },
}

#[derive(Debug, thiserror::Error)]
pub enum InvalidLinkDeclaration {
    #[error("repeated link definition `{name}`")]
    RepeatedDefinition { name: String },
}

#[derive(Debug, thiserror::Error)]
pub enum InvalidSocketDeclaration {
    #[error("repeated socket definition `{name}`")]
    RepeatedDefinition { name: String },
}

#[derive(Debug, thiserror::Error)]
pub enum InvalidSubplanDeclaration {
    #[error("repeated subplan definition `{key}`")]
    RepeatedDefinition { key: KeyOwned },

    #[error(
        "Subplan keys must be disjoint, but the subplan key \
         `{short}` is the prefix of another subplan key `{long}`"
    )]
    NonDisjointKeysNotAllowed { short: KeyOwned, long: KeyOwned },
}

#[derive(Debug, thiserror::Error)]
pub enum ParseParamDefError {
    #[error("invalid parameter name `{name}`")]
    InvalidName { name: String },
    #[error("invalid parameter type `{ty}`")]
    InvalidType { ty: String },
    #[error("unable to parse parameter definition `{expr}`")]
    MalformedDecl { expr: String },
}

#[derive(Debug, thiserror::Error)]
#[error("invalid parameter declaration: {reason}")]
pub enum InvalidParameterDeclaration {
    #[error("parameter value type mismatches")]
    ValueTypeMismatch,

    #[error("parameter value type is not supported")]
    ValueTypeNotSupported,

    #[error("repeated parameter definition `{name}`")]
    RepeatedDefinition { name: String },

    #[error("the argument must be either specified with a required or with a default value")]
    InvalidArgumentDefinition,
}

#[derive(Debug, thiserror::Error)]
pub enum InvalidParameterValue {
    #[error("empty value is not allowed")]
    EmptyValueNotAllowed,

    #[error("expect a single type, but multiple types are assigned")]
    MultipleTypesNotAllowed,
}

#[derive(Debug, thiserror::Error)]
pub enum KeyCreationError {
    #[error("cannot create a key from string `{key}: {reason}`")]
    InitializationError { key: String, reason: String },

    #[error("fail to concatenate key `{lhs}` with `{rhs}`")]
    ConcatenationError { lhs: KeyOwned, rhs: String },
}

#[derive(Debug, thiserror::Error)]
#[error("cannot create a ROS topic from string `{topic}: {reason}`")]
pub struct TopicCreationError {
    pub topic: String,
    pub reason: String,
}

#[derive(Debug, thiserror::Error)]
#[error("`{name}` is not a proper interface type name")]
pub struct InvalidInterfaceType {
    pub name: String,
}

#[derive(Debug, thiserror::Error)]
#[error("invalid byte array data: {reason}")]
pub struct InvalidByteArrayData {
    pub reason: String,
}

#[derive(Debug, thiserror::Error)]
#[error("unable to check if `{prefix}` is the prefix of `{checked}`")]
pub struct InvalidKeyPrefixError {
    pub checked: KeyOwned,
    pub prefix: KeyOwned,
}
