use crate::key::KeyOwned;

#[derive(Debug, thiserror::Error)]
pub enum Error {}

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

#[derive(Debug, thiserror::Error)]
pub enum DeserializationError {
    #[error("expect an expression wrapped within a pair of $ or $$$")]
    ExpectExpr,

    #[error("expect a string or an expression wrapped within a pair of $ or $$$")]
    ExpectTextOrExpr,

    #[error("expect a !type tag")]
    ExpectTypeTag,

    #[error("invalid !type tag")]
    InvalidTypeTag,

    #[error("expect a value with !type tag: {error}")]
    ExpectValueWithType { error: serde_yaml::Error },
}

#[derive(Debug, thiserror::Error)]
#[error("unable to parse YAML value")]
pub struct ParseYamlError;

#[derive(Debug, thiserror::Error)]
#[error("unable to parse expression: {reason}")]
pub struct ParseExpressionError {
    pub reason: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifier_creation_error_display() {
        let err = IdentifierCreationError {
            string: "invalid id".to_string(),
        };
        assert!(err.to_string().contains("invalid id"));
    }

    #[test]
    fn parse_arg_def_error_invalid_name() {
        let err = ParseArgDefError::InvalidName {
            name: "bad-name".to_string(),
        };
        assert!(err.to_string().contains("invalid argument name"));
        assert!(err.to_string().contains("bad-name"));
    }

    #[test]
    fn parse_arg_def_error_invalid_type() {
        let err = ParseArgDefError::InvalidType {
            ty: "unknown_type".to_string(),
        };
        assert!(err.to_string().contains("invalid argument type"));
        assert!(err.to_string().contains("unknown_type"));
    }

    #[test]
    fn parse_arg_def_error_malformed() {
        let err = ParseArgDefError::MalformedDecl {
            expr: "bad=expr".to_string(),
        };
        assert!(err.to_string().contains("unable to parse"));
        assert!(err.to_string().contains("bad=expr"));
    }

    #[test]
    fn invalid_argument_declaration_errors() {
        let err = InvalidArgumentDeclaration::ValueTypeMismatch;
        assert!(err.to_string().contains("type mismatches"));

        let err = InvalidArgumentDeclaration::ValueTypeNotSupported;
        assert!(err.to_string().contains("not supported"));

        let err = InvalidArgumentDeclaration::RepeatedDefinition {
            name: "count".to_string(),
        };
        assert!(err.to_string().contains("repeated"));
        assert!(err.to_string().contains("count"));
    }

    #[test]
    fn invalid_node_declaration_error() {
        let err = InvalidNodeDeclaration::RepeatedDefinition {
            name: "talker".to_string(),
        };
        assert!(err.to_string().contains("repeated"));
        assert!(err.to_string().contains("talker"));
    }

    #[test]
    fn invalid_link_declaration_error() {
        let err = InvalidLinkDeclaration::RepeatedDefinition {
            name: "link1".to_string(),
        };
        assert!(err.to_string().contains("repeated"));
        assert!(err.to_string().contains("link1"));
    }

    #[test]
    fn invalid_socket_declaration_error() {
        let err = InvalidSocketDeclaration::RepeatedDefinition {
            name: "output".to_string(),
        };
        assert!(err.to_string().contains("repeated"));
        assert!(err.to_string().contains("output"));
    }

    #[test]
    fn invalid_subplan_declaration_repeated() {
        let key: KeyOwned = "test/subplan".parse().unwrap();
        let err = InvalidSubplanDeclaration::RepeatedDefinition { key };
        assert!(err.to_string().contains("repeated"));
        assert!(err.to_string().contains("test/subplan"));
    }

    #[test]
    fn invalid_subplan_declaration_non_disjoint() {
        let short: KeyOwned = "robot".parse().unwrap();
        let long: KeyOwned = "robot/subsystem".parse().unwrap();
        let err = InvalidSubplanDeclaration::NonDisjointKeysNotAllowed { short, long };
        assert!(err.to_string().contains("disjoint"));
        assert!(err.to_string().contains("prefix"));
    }

    #[test]
    fn parse_param_def_errors() {
        let err = ParseParamDefError::InvalidName {
            name: "bad-param".to_string(),
        };
        assert!(err.to_string().contains("invalid parameter name"));

        let err = ParseParamDefError::InvalidType {
            ty: "bad_type".to_string(),
        };
        assert!(err.to_string().contains("invalid parameter type"));

        let err = ParseParamDefError::MalformedDecl {
            expr: "malformed".to_string(),
        };
        assert!(err.to_string().contains("unable to parse parameter"));
    }

    #[test]
    fn invalid_parameter_value_errors() {
        let err = InvalidParameterValue::EmptyValueNotAllowed;
        assert!(err.to_string().contains("empty"));

        let err = InvalidParameterValue::MultipleTypesNotAllowed;
        assert!(err.to_string().contains("multiple types"));
    }

    #[test]
    fn key_creation_error_initialization() {
        let err = KeyCreationError::InitializationError {
            key: "bad/key/".to_string(),
            reason: "trailing slash".to_string(),
        };
        assert!(err.to_string().contains("cannot create a key"));
        assert!(err.to_string().contains("bad/key/"));
        assert!(err.to_string().contains("trailing slash"));
    }

    #[test]
    fn topic_creation_error() {
        let err = TopicCreationError {
            topic: "//bad".to_string(),
            reason: "double slash".to_string(),
        };
        assert!(err.to_string().contains("ROS topic"));
        assert!(err.to_string().contains("//bad"));
    }

    #[test]
    fn invalid_interface_type_error() {
        let err = InvalidInterfaceType {
            name: "BadType".to_string(),
        };
        assert!(err.to_string().contains("not a proper interface type"));
        assert!(err.to_string().contains("BadType"));
    }

    #[test]
    fn deserialization_errors() {
        let err = DeserializationError::ExpectExpr;
        assert!(err.to_string().contains("expect an expression"));

        let err = DeserializationError::ExpectTextOrExpr;
        assert!(err.to_string().contains("string or an expression"));

        let err = DeserializationError::ExpectTypeTag;
        assert!(err.to_string().contains("expect a !type tag"));

        let err = DeserializationError::InvalidTypeTag;
        assert!(err.to_string().contains("invalid !type tag"));
    }

    #[test]
    fn parse_expression_error() {
        let err = ParseExpressionError {
            reason: "imbalanced $".to_string(),
        };
        assert!(err.to_string().contains("unable to parse expression"));
        assert!(err.to_string().contains("imbalanced $"));
    }
}
