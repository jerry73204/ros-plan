use crate::{
    error::InvalidArgumentDeclaration,
    expr::{ValueOrExpr, ValueType},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedArgEntry", into = "SerializedArgEntry")]
pub struct ArgEntry {
    pub help: Option<String>,
    pub ty: ValueType,
    pub default: Option<ValueOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct SerializedArgEntry {
    pub help: Option<String>,
    #[serde(rename = "type")]
    pub ty: Option<ValueType>,
    pub default: Option<ValueOrExpr>,
}

impl TryFrom<SerializedArgEntry> for ArgEntry {
    type Error = InvalidArgumentDeclaration;

    fn try_from(entry: SerializedArgEntry) -> Result<Self, Self::Error> {
        let SerializedArgEntry {
            ty: expect_ty,
            default,
            help,
        } = entry;

        let (ty, default) = match (expect_ty, default) {
            (None, None) => unreachable!(),
            (None, Some(default)) => (default.ty(), Some(default)),
            (Some(ty), None) => (ty, None),
            (Some(expect_ty), Some(default)) => {
                if expect_ty != default.ty() {
                    return Err(InvalidArgumentDeclaration::ValueTypeMismatch);
                }
                (expect_ty, Some(default))
            }
        };

        let entry = ArgEntry { help, ty, default };
        Ok(entry)
    }
}

impl From<ArgEntry> for SerializedArgEntry {
    fn from(entry: ArgEntry) -> Self {
        let ArgEntry { help, ty, default } = entry;

        SerializedArgEntry {
            help,
            ty: Some(ty),
            default,
        }
    }
}
