use crate::error::InvalidInterfaceType;
use pomsky_macro::pomsky;
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::{borrow::Borrow, ops::Deref, str::FromStr, sync::LazyLock};

pub static RE_INTERFACE_TYPE: LazyLock<Regex> = LazyLock::new(|| {
    let rule = pomsky! {
        ^ [ascii_word]+ "/" ("msg" | "srv" | "action") "/" [ascii_word]+ $
    };
    Regex::new(rule).expect("invalid regex for key")
});

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct InterfaceType(str);

impl InterfaceType {
    pub fn package(&self) -> &str {
        let (package, _) = self.0.split_once('/').unwrap();
        package
    }

    pub fn kind(&self) -> InterfaceTypeKind {
        let kind_str = self.0.split('/').skip(1).take(1).next().unwrap();
        match kind_str {
            "msg" => InterfaceTypeKind::Msg,
            "srv" => InterfaceTypeKind::Srv,
            "action" => InterfaceTypeKind::Action,
            _ => unreachable!(),
        }
    }

    pub fn ty(&self) -> &str {
        let (_, ty) = self.0.rsplit_once('/').unwrap();
        ty
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl FromStr for &InterfaceType {
    type Err = InvalidInterfaceType;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        if !RE_INTERFACE_TYPE.is_match(text) {
            return Err(InvalidInterfaceType {
                name: text.to_string(),
            });
        }
        let ty: &InterfaceType = unsafe { std::mem::transmute(text) };
        Ok(ty)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(transparent)]
pub struct InterfaceTypeOwned(String);

impl InterfaceTypeOwned {
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl AsRef<InterfaceType> for InterfaceTypeOwned {
    fn as_ref(&self) -> &InterfaceType {
        unsafe { std::mem::transmute(self.as_str()) }
    }
}

impl Borrow<InterfaceType> for InterfaceTypeOwned {
    fn borrow(&self) -> &InterfaceType {
        self.as_ref()
    }
}

impl Deref for InterfaceTypeOwned {
    type Target = InterfaceType;

    fn deref(&self) -> &Self::Target {
        self.as_ref()
    }
}

impl FromStr for InterfaceTypeOwned {
    type Err = InvalidInterfaceType;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        if !RE_INTERFACE_TYPE.is_match(text) {
            return Err(InvalidInterfaceType {
                name: text.to_string(),
            });
        }
        Ok(Self(text.to_string()))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InterfaceTypeKind {
    Msg,
    Srv,
    Action,
}
